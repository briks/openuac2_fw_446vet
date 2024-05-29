#include "ak4490r.h"
#include "usbd_audio_if.h"
#include "main.h"
#include "cmsis_os.h"

#define MAX_RECEIVED_VOLUME 100 // Max value send by the driver, assuming the min is 0
#define MAX_ATTENUATION     64  // in dB, knowing that the step is 0.5dB in the register, max 127.5, so max 127 here.
#define TIMEOUT_I2C_DELAY 10    // in ms (ticks), could be HAL_MAX_DELAY for infinite delay

extern I2C_HandleTypeDef AK4490R_I2C_HANDLE;

static uint8_t play;
uint8_t requested_attenuation  = 255; // Set the max
uint8_t configured_attenuation = 0;   // Set the min, to force a set @requested_attenuation after init.
uint8_t requested_mute = 0;
uint8_t configured_mute = 1; // Force a set @requested_mute after init.
AUDIO_FormatTypeDef requested_format = AUDIO_FORMAT_PCM;
AUDIO_FormatTypeDef configured_format = AUDIO_FORMAT_DSD; // Force a set @requested_format after init.
uint8_t regread;
uint8_t status_register = 0;
uint8_t registre;

AUDIO_CodecTypeDef ak4490r_instance =
{
        AK4490R_Init,
        NULL,
        AK4490R_Play,
        AK4490R_SetFormat,
        AK4490R_Stop,
        NULL,
        AK4490R_SetMute,
        AK4490R_SetVolume
};

//static AK4490R_RegisterTypeDef reg;

uint8_t AK4490R_Init()
{
	LL_GPIO_ResetOutputPin(PDN_GPIO_Port, PDN_Pin);
    HAL_Delay(10); // called from interrupt, osDelay not allowed
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

	//reg14 normal operation b10001010 
	registre = 0x8a;
    HAL_I2C_Mem_Read(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, AK4490R_REG14_ADDR, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&regread, sizeof(regread), TIMEOUT_I2C_DELAY);
    HAL_I2C_Mem_Write(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, AK4490R_REG14_ADDR, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&registre, sizeof(registre), TIMEOUT_I2C_DELAY);
    HAL_I2C_Mem_Read(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, AK4490R_REG14_ADDR, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&regread, sizeof(regread), TIMEOUT_I2C_DELAY);

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
	//registre = 0xdc;
	//reg27 setasrc enable, volume ch2 same as ch1, and allow volume update
	registre = 0x8C;
    HAL_I2C_Mem_Write(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, AK4490R_REG27_ADDR, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&registre, sizeof(registre), TIMEOUT_I2C_DELAY);
    return 0;
}


uint8_t AK4490R_SetVolume(uint8_t vol) // receive a value between 0 and 100. Compare to windows : 0->0, 1->0, then n-1 up to 99, and 100->100
{
    vol = MAX_RECEIVED_VOLUME - (uint32_t)vol;
    requested_attenuation = (vol * vol) * MAX_ATTENUATION * 2 / (MAX_RECEIVED_VOLUME * MAX_RECEIVED_VOLUME); // attenuation in range 0-255
	return 0;
}

uint8_t AK4490R_SetMute(uint8_t mute) // mute = 1 when mute is requested
{
    requested_mute = mute;
	return 0;
}

uint8_t AK4490R_SetFormat(uint8_t format)
{
    requested_format = format;
	return 0;
}

uint8_t AK4490R_Play()
{
	play = 1;
	return 0;
}

uint8_t AK4490R_Stop()
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
    HAL_StatusTypeDef I2C_Status = HAL_OK;
    if (play)
    {
        // osDelay(20);
        // registre = 0x80;
		//while (HAL_I2C_Mem_Write(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, 0x07, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&registre, 1, 1000) != HAL_OK);
		play = 0;
	}

    if (requested_mute != configured_mute)
    {// adjust mute state
        if (AK4490R_I2C_HANDLE.State == HAL_I2C_STATE_READY)
        {
            configured_mute = requested_mute;
            if (configured_mute)
            {
                registre = 0x81;
            }
            else
            {
                registre = 0x80;
            }
            // reg7 filter bw and system mute: set the mute b10000001 or normal b10000000
            I2C_Status |= HAL_I2C_Mem_Write(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, AK4490R_REG7_ADDR, I2C_MEMADD_SIZE_8BIT, &registre, 1, TIMEOUT_I2C_DELAY);
        }
    }

    if (requested_format != configured_format)
    { // adjust audio format (DSD or not)
        configured_format = requested_format;
        // Set by default in auto mode, see AK4490R_REG1_ADDR:auto_select
        // if (AK4490R_I2C_HANDLE.State == HAL_I2C_STATE_READY)
        // {
        //     HAL_I2C_Mem_Read(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, AK4490R_REG7_ADDR, I2C_MEMADD_SIZE_8BIT, &registre, 1, TIMEOUT_I2C_DELAY);
        //     if (configured_format == AUDIO_FORMAT_DSD)
        //     {
        //         registre |= AK4490R_DP;
        //     }
        //     else
        //     {
        //         registre &= ~AK4490R_DP;
        //     }

        //     //		reg.control1 &= ~AK4490R_RSTN;
        //     HAL_I2C_Mem_Write(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, AK4490R_CONTROL3_ADDR, I2C_MEMADD_SIZE_8BIT, &registre, 1, TIMEOUT_I2C_DELAY);
        //     //		reg_reset = 1;
        // }
    }

    if (requested_attenuation != configured_attenuation)
    { // adjust volume
        if (AK4490R_I2C_HANDLE.State == HAL_I2C_STATE_READY)
        {
            configured_attenuation = requested_attenuation;
            // Write an attenuation in the register in range 0-255
            I2C_Status |= HAL_I2C_Mem_Write(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, AK4490R_REG15_ADDR, I2C_MEMADD_SIZE_8BIT, &configured_attenuation, 1, TIMEOUT_I2C_DELAY);
            // not needed to update volume2, see AK4490R_REG27_ADDR configuration (ch1_volume)
            // HAL_I2C_Mem_Write(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, AK4490R_REG16_ADDR, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&registre, 1, TIMEOUT_I2C_DELAY);
            // HAL_I2C_Mem_Read(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, AK4490R_REG15_ADDR, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&registre, 1, TIMEOUT_I2C_DELAY );
        }
    }

    /* Read status
    [3] dop_valid Contains the status of the DoP decoder (DSD over PCM)
        1’b0 : The DoP decoder has not detected a valid DoP signal
        1’b1 : The DoP decoder has detected a valid DoP signaI2S input
    [2] spdif_valid Contains the status of the SPDIF decoder.
        1’b0 : The SPDIF decoder has not found a valid SPDIF signal.
        1’b1 : The SPDIF decoder has detected a valid SPDIF
    [1]   i2s_select   Contains the status of the I2S decoder.
        1’b0: The I2S decoder has not found a valid frame clock or bit clock.
        1’b1: The I2S decoder has detected a valid frame clock and bit clock arrangement
    [0] dsd select Contains the status of the DSD decoder.
        1’b0: The DSD decoder is not being used.
        1’b1: The DSD decoder is being used as a fallback option if I2S has failed to decode their respective input signals.
    */
    I2C_Status |= HAL_I2C_Mem_Read(&AK4490R_I2C_HANDLE, AK4490R_I2C_DEV_ADDR, AK4490R_REG96_ADDR, I2C_MEMADD_SIZE_8BIT, &status_register, 1, TIMEOUT_I2C_DELAY);
    if (I2C_Status != HAL_OK)
    {
        Error_Handler_nonBlocking("I2C read failure", ERROR_I2C);
    }
    else
    {
        Error_cancel_nonBlocking(ERROR_I2C);
    }
}
