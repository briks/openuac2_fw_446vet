#include "ak4490r.h"
#include "usbd_audio_if.h"
#include "main.h"
#include "cmsis_os.h"
#define LOG_LOCAL_LEVEL LOG_LEVEL_DBG
#include "log.h"

//#define MAX_RECEIVED_VOLUME 100 // Max value send by the driver, assuming the min is 0
#define MAX_ATTENUATION     64  // in dB, knowing that the step is 0.5dB in the register, max 127.5, so max 127 here.
#define TIMEOUT_I2C_DELAY   10  // in ms (ticks), could be HAL_MAX_DELAY for infinite delay

extern I2C_HandleTypeDef AK4490R_I2C_HANDLE;

static uint8_t play;
int16_t requested_volume = AUDIO_CUR_VOL; // Volume set after init, q8.8 format
volatile int16_t configured_volume = AUDIO_CUR_VOL + 1; // Set a differente value, to force a set @requested_volume after init.
bool requested_mute = false; // starts unmuted when amp switch on
volatile bool configured_mute = false; // will be set true as amp is off at init, and switch back when amp is started
volatile bool audio_stop_pending = false;
AUDIO_FormatTypeDef requested_format = AUDIO_FORMAT_PCM;
AUDIO_FormatTypeDef configured_format = AUDIO_FORMAT_DSD; // Force a set @requested_format after init.
uint8_t regread;
uint8_t status_register = 0;
uint8_t registre;

AUDIO_CodecTypeDef ak4490r_instance =
{
        AK4490R_DAC_Init,
        NULL,
        AK4490R_DAC_Play,
        AK4490R_DAC_SetFormat,
        AK4490R_DAC_Stop,
        NULL,
        AK4490R_DAC_SetMute,
        AK4490R_DAC_SetVolume
};

//static AK4490R_RegisterTypeDef reg;

uint8_t AK4490R_DAC_Init()
{
    LOG_INFO("DAC init");
    HAL_Delay(10); // called from interrupt, osDelay not allowed

    LL_GPIO_ResetOutputPin(PDN_GPIO_Port, PDN_Pin);
    LL_GPIO_SetOutputPin(PDN_GPIO_Port, PDN_Pin);
    HAL_Delay(100); // Delay in interrupt, not so good
    //reg1 input selection: set 32bits data default and i2s 1100 set to i2s input (no auto detect) 0000
	//reg7 filter bw and system mute: set the mute b10000001 or normal b10000000
	//reg8: set gpio1 to spdif input & gpio2 to whatever analog input for shutdown d13d8
	//reg11: set wich input use when decoding SPDIF data GPIO1: d3d0
	//reg27 general configuration: set ch1 volume on to share volume between ch1 and ch2 b11011100

	//shut down the oscillator b11110000
	registre = 0xf0;
	//HAL_I2C_Mem_Write_IT(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&registre, sizeof(registre));

    HAL_I2C_Mem_Read(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, AK4490R_REG14_ADDR, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&regread, sizeof(regread), TIMEOUT_I2C_DELAY);
    LOG_DBG("reg14 before write: 0x%02X", regread);
	//reg14 normal operation b10001010
    // 0x8A with reserved bits at defaults:
    //   bit 7 = 0 (normal operation)
    //   [6:5] = 00 (reserved default)
    //   bit 4 = 0 (reserved default)
    //   bit 3 = 0 (reserved default)
    //   bit 2 = 0 (reserved default)
    //   [1:0] = 10 (normal operation)
    // = 0000 1010 = 0x0A
    registre = 0x0A;
    st = HAL_I2C_Mem_Write(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, AK4490R_REG14_ADDR,
                           I2C_MEMADD_SIZE_8BIT, &registre, 1, TIMEOUT_I2C_DELAY);
    if (st != HAL_OK) {
        LOG_ERR("DAC init: REG14 write failed (st=%d)", st);
    }
    HAL_I2C_Mem_Read(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, AK4490R_REG14_ADDR, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&regread, sizeof(regread), TIMEOUT_I2C_DELAY);
    LOG_DBG("reg14 after write: 0x%02X", regread);


    //reg1 input selection: set 32bits data default and i2s 1100 set to i2s input (no auto detect) 0000
	registre = 0xcc;
	//HAL_I2C_Mem_Write(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, 0x01, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&registre, sizeof(registre), HAL_MAX_DELAY);

	//reg8: set gpio1 to spdif input & gpio2 to whatever analog input for shutdown d13d8
	registre = 0xd8;
	//HAL_I2C_Mem_Write_IT(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, 0x08, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&registre, sizeof(registre));

	//reg11: set wich input use when decoding SPDIF data GPIO1: d3d0
	registre = 0x30;
	//HAL_I2C_Mem_Write_IT(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, 0x0b, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&registre, sizeof(registre));

	//reg27 no +18db gain (not good idea) made volumne ch2 same a ch1, and allow volume update
	//registre = 0xdc; // wrong value for +18db, should be 8F

	//reg27 setasrc enable, volume ch2 same as ch1, and allow volume update
    // 0x8C with reserved bits at defaults:
    //   bit 7 = 1 (asrc_en)
    //   [6:5] = 10 (reserved default)
    //   bit 4 = 1 (reserved default)
    //   bit 3 = 1 (ch1_volume = link L/R)
    //   bit 2 = 1 (latch_volume)
    //   [1:0] = 00 (no 18dB gain)
    // = 1011 1100 = 0xBC
    registre = 0xBC;
    st = HAL_I2C_Mem_Write(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, AK4490R_REG27_ADDR,
                           I2C_MEMADD_SIZE_8BIT, &registre, 1, TIMEOUT_I2C_DELAY);
    if (st != HAL_OK) {
        LOG_ERR("DAC init: REG27 write failed (st=%d)", st);
    }

    AK4490R_DAC_SetMute_Force();  // startup muted, waiting for amp power ON, even if windows starts
                                  // unmuted
    LOG_INFO("DAC init done");
    return 0;
}


uint8_t AK4490R_DAC_SetVolume(int16_t vol) // Q8.8 format
{
    /* Clamp to declared range to ignore special values like 0x8000. */
    if (vol < AUDIO_MIN_VOL)
    {
        vol = AUDIO_MIN_VOL;
    }
    if (vol > AUDIO_MAX_VOL)
    {
        vol = AUDIO_MAX_VOL;
    }
    requested_volume = vol;
    return 0;
}

uint8_t AK4490R_DAC_SetMute(uint8_t mute) // mute = 1 when mute is requested
{
    requested_mute = (mute == 1); // true or false

    return 0;
}

HAL_StatusTypeDef AK4490R_DAC_SetMute_Immediate(uint8_t mute) // mute = 1 when mute is requested
{
    HAL_StatusTypeDef I2C_Status = HAL_OK;
    registre = mute ? 0x81 : 0x80;

    do {
        I2C_Status = HAL_I2C_Mem_Write(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR,
                                       AK4490R_REG7_ADDR, I2C_MEMADD_SIZE_8BIT,
                                       &registre, 1, TIMEOUT_I2C_DELAY);
        if (I2C_Status == HAL_BUSY) {
            osDelay(1);
        }
    } while (I2C_Status == HAL_BUSY);

    if (I2C_Status != HAL_OK) {
        LOG_ERR("mute %s write failed (st=%d)", mute ? "ON" : "OFF", I2C_Status);
    }
    return I2C_Status;
}

// force mute in case of amp power off, but keep track of requested mute state
void AK4490R_DAC_SetMute_Force(void)
{
    // store mute state to restore it in case of power on
    requested_mute = configured_mute || requested_mute;
    AK4490R_DAC_SetMute_Immediate(true);
    
    configured_mute = true;
    USBD_AUDIO_signal_mute_change();
}

uint8_t AK4490R_DAC_SetFormat(uint8_t format)
{
    requested_format = format;
	return 0;
}

uint8_t AK4490R_DAC_Play()
{
	play = 1;
	return 0;
}

uint8_t AK4490R_DAC_Stop()
{
	// if (AK4490R_I2C_HANDLE.State == HAL_I2C_STATE_READY)
	// {
	// 	registre = 0x81;
	// }

	//HAL_I2C_Mem_Write_IT(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, 0x07, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&registre, 1);
	return 0;
}

void AK4490R_ProcessEvents()
{// Process audio events in the task context, in order of priority
    static HAL_StatusTypeDef I2C_Status = HAL_OK;
    static uint32_t cnt = 0;

    /* Read status
    [3] dop_valid Contains the status of the DoP decoder (DSD over PCM)
        b0 : The DoP decoder has not detected a valid DoP signal
        b1 : The DoP decoder has detected a valid DoP signaI2S input
    [2] spdif_valid Contains the status of the SPDIF decoder.
        b0 : The SPDIF decoder has not found a valid SPDIF signal.
        b1 : The SPDIF decoder has detected a valid SPDIF
    [1]   i2s_select   Contains the status of the I2S decoder.
        0: The I2S decoder has not found a valid frame clock or bit clock.
        b1: The I2S decoder has detected a valid frame clock and bit clock arrangement
    [0] dsd select Contains the status of the DSD decoder.
        b0: The DSD decoder is not being used.
        b1: The DSD decoder is being used as a fallback option if I2S has failed to decode their respective input signals.
    */
    cnt++;
    if (cnt % 10 == 0)
    {// Do the check only once out of 10
        uint8_t prev_status = status_register;
        I2C_Status |= HAL_I2C_Mem_Read(&AK4490R_I2C_HANDLE,
                                       AK4490R_I2C_DEV_ADDR,
                                       AK4490R_REG96_ADDR,
                                       I2C_MEMADD_SIZE_8BIT,
                                       &status_register,
                                       1,
                                       TIMEOUT_I2C_DELAY);
        if (I2C_Status != HAL_OK)
        {
            Error_Handler_nonBlocking("I2C read failure", ERROR_I2C);
            MX_I2C1_Init();
            I2C_Status = HAL_OK;  // reinit to see if better after init
        }
        else
        {
            Error_cancel_nonBlocking(ERROR_I2C);
            if (status_register != prev_status)
            {
                LOG_DBG("DAC status 0x%02X → 0x%02X (dop=%d spdif=%d i2s=%d dsd=%d)",
                        prev_status,
                        status_register,
                        !!(status_register & 0x08),
                        !!(status_register & 0x04),
                        !!(status_register & 0x02),
                        !!(status_register & 0x01));
            }
        }
    }

    if (audio_stop_pending)
    {
        audio_stop_pending = false;
        USBD_AUDIO_fops.AUDIO_Cmd(NULL, 0, AUDIO_CMD_STOP);
    }

    if (play)
    {
        // osDelay(20);
        // registre = 0x80;
		//while (HAL_I2C_Mem_Write(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, 0x07, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&registre, 1, 1000) != HAL_OK);
		play = 0;
	}

    if (requested_mute != configured_mute)
    {// adjust mute state
        if (   (AK4490R_I2C_HANDLE.State == HAL_I2C_STATE_READY)
            && (EtatAmp) )
        {
            configured_mute = requested_mute;
            I2C_Status |= AK4490R_DAC_SetMute_Immediate(configured_mute);
            LOG_INFO("mute %s", configured_mute ? "ON" : "OFF");
            USBD_AUDIO_signal_mute_change();
        } else {
            /* don't spam: log only once per change */
            static bool warned = false;
            if (!warned) {
                LOG_WARN("mute change deferred (i2c_state=%d amp=%d)",
                         AK4490R_I2C_HANDLE.State, EtatAmp);
                warned = true;
            }
        }
    }

    if (requested_format != configured_format)
    { // adjust audio format (DSD or not)
        configured_format = requested_format;
        // Set by default in auto mode, see AK4490R_REG1_ADDR:auto_select
        // if (AK4490R_I2C_HANDLE.State == HAL_I2C_STATE_READY)
        // {
        //     HAL_I2C_Mem_Read(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, AK4490R_REG7_ADDR,
        //     I2C_MEMADD_SIZE_8BIT, &registre, 1, TIMEOUT_I2C_DELAY); if (configured_format ==
        //     AUDIO_FORMAT_DSD)
        //     {
        //         registre |= AK4490R_DP;
        //     }
        //     else
        //     {
        //         registre &= ~AK4490R_DP;
        //     }

        //     //		reg.control1 &= ~AK4490R_RSTN;
        //     HAL_I2C_Mem_Write(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, AK4490R_CONTROL3_ADDR,
        //     I2C_MEMADD_SIZE_8BIT, &registre, 1, TIMEOUT_I2C_DELAY);
        //     //		reg_reset = 1;
        // }
        LOG_INFO("audio format → %s",
                 configured_format == AUDIO_FORMAT_DSD ? "DSD" : "PCM");
    }

    if (requested_volume != configured_volume)
    {  // adjust volume
        if (AK4490R_I2C_HANDLE.State == HAL_I2C_STATE_READY)
        {
            configured_volume = requested_volume;
            // uint32_t vol = MAX_RECEIVED_VOLUME - (uint32_t)configured_volume;
            // uint8_t configured_attenuation = (vol * vol) * MAX_ATTENUATION * 2 /
            // (MAX_RECEIVED_VOLUME * MAX_RECEIVED_VOLUME); // attenuation in range 0-255
            // // Write an attenuation in the register in range 0-255
            // I2C_Status |= HAL_I2C_Mem_Write(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR,
            // AK4490R_REG15_ADDR, I2C_MEMADD_SIZE_8BIT,
            //                                 &configured_attenuation, 1, TIMEOUT_I2C_DELAY);
            // not needed to update volume2, see AK4490R_REG27_ADDR configuration (ch1_volume)
            // HAL_I2C_Mem_Write(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, AK4490R_REG16_ADDR,
            // I2C_MEMADD_SIZE_8BIT, (uint8_t*)&registre, 1, TIMEOUT_I2C_DELAY);
            // HAL_I2C_Mem_Read(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, AK4490R_REG15_ADDR,
            // I2C_MEMADD_SIZE_8BIT, (uint8_t*)&registre, 1, TIMEOUT_I2C_DELAY );

            /* replace the quadratic mapping with a linear dB→register conversion. */
            /* configured_volume is signed Q8.8 dB, range [AUDIO_MIN_VOL .. AUDIO_MAX_VOL]
             * (i.e., -60 dB .. 0 dB).
             * ES9038Q2M REG15 attenuation = -0.5 dB per step.
             * register_value = -2 * dB = -2 * (q88 / 256) = -q88 / 128
             */
            int32_t attenuation = -((int32_t) configured_volume) / 128;
            if (attenuation < 0)
            {
                attenuation = 0;
            }
            if (attenuation > 255)
            {
                attenuation = 255;
            }

            uint8_t reg_val = (uint8_t) attenuation;

            HAL_StatusTypeDef st = HAL_I2C_Mem_Write(&AK4490R_I2C_HANDLE,
                                                     AK4490R_I2C_DEV_ADDR,
                                                     AK4490R_REG15_ADDR,
                                                     I2C_MEMADD_SIZE_8BIT,
                                                     &reg_val,
                                                     1,
                                                     TIMEOUT_I2C_DELAY);
            if (st != HAL_OK)
            {
                LOG_ERR("volume write failed (vol=%d att=%u st=%d)",
                        configured_volume,
                        reg_val,
                        st);
            }
            else
            {
                /* Volume in 0.5 dB units: Q8.8 dB / 128 = dB * 2 */
                int dB_x10 = (int) configured_volume * 10 / 256;
                LOG_INFO("volume %d.%d dB (att=0x%02X)",
                         dB_x10 / 10,
                         (dB_x10 < 0 ? -dB_x10 : dB_x10) % 10,
                         reg_val);
            }
        }
    }
}
