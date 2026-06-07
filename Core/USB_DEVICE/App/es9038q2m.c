#include "es9038q2m.h"
#include "main.h"
#include "cmsis_os.h"
#define LOG_LEVEL LOG_LEVEL_INFO
#include "log.h"

#define TIMEOUT_I2C_DELAY   10   /* ms; could be HAL_MAX_DELAY for infinite */

extern I2C_HandleTypeDef ES9038Q2M_I2C_HANDLE;

static uint8_t play;

volatile int16_t requested_volume_ch1 = AUDIO_CUR_VOL;                /* set at boot, host range */
volatile int16_t requested_volume_ch2 = AUDIO_CUR_VOL;                /* set at boot, host range */
volatile int16_t es9038q2m_configured_volume_ch1 = AUDIO_CUR_VOL + 1; /* differ → force apply on first ProcessEvents */
volatile int16_t es9038q2m_configured_volume_ch2 = AUDIO_CUR_VOL + 1; /* differ → force apply on first ProcessEvents */
volatile bool requested_mute = true;                                  /* unmuted when amp powers on */
volatile bool es9038q2m_configured_mute = false;                      /* will be switched at startup since amp is off */
volatile bool es9038q2m_audio_stop_pending = false;

AUDIO_FormatTypeDef requested_format  = AUDIO_FORMAT_PCM;
AUDIO_FormatTypeDef configured_format = AUDIO_FORMAT_DSD;  /* differ → force apply on first call */

uint8_t regread;
uint8_t status_register = 0;
uint8_t registre;

const AUDIO_CodecTypeDef es9038q2m_instance =
    {
        ES9038Q2M_DAC_Init,
        ES9038Q2M_DAC_DeInit,
        ES9038Q2M_DAC_Play,
        ES9038Q2M_DAC_Format_set,
        ES9038Q2M_DAC_Stop,
        NULL,
        ES9038Q2M_DAC_Mute_set,
        ES9038Q2M_DAC_Volume_set};

uint8_t ES9038Q2M_DAC_Init(void)
{
    LOG_INFO("ES9038Q2M_DAC_Init...");
    LL_GPIO_ResetOutputPin(PDN_GPIO_Port, PDN_Pin);
    HAL_Delay(10);                    /* may be called from ISR context; osDelay not allowed */
    LL_GPIO_SetOutputPin(PDN_GPIO_Port, PDN_Pin);
    HAL_Delay(100);

    /* REG14 = 0x8A: normal operation (read-modify-write pattern kept for diagnostics) */
    registre = 0x8a;
    HAL_I2C_Mem_Read (&ES9038Q2M_I2C_HANDLE, ES9038Q2M_I2C_DEV_ADDR, ES9038Q2M_REG14_ADDR,
                      I2C_MEMADD_SIZE_8BIT, &regread,  1, TIMEOUT_I2C_DELAY);
    HAL_I2C_Mem_Write(&ES9038Q2M_I2C_HANDLE, ES9038Q2M_I2C_DEV_ADDR, ES9038Q2M_REG14_ADDR,
                      I2C_MEMADD_SIZE_8BIT, &registre, 1, TIMEOUT_I2C_DELAY);
    HAL_I2C_Mem_Read (&ES9038Q2M_I2C_HANDLE, ES9038Q2M_I2C_DEV_ADDR, ES9038Q2M_REG14_ADDR,
                      I2C_MEMADD_SIZE_8BIT, &regread,  1, TIMEOUT_I2C_DELAY);

    /* REG27 = 0xBC: ASRC enable, link L/R volume, latch volume, no +18 dB gain.
     *   bit 7   = 1  : asrc_en
     *   [6:5]   = 10 : reserved defaults
     *   bit 4   = 1  : reserved default
     *   bit 3   = 0  : ch1_volume (Allow independent control)
     *   bit 2   = 1  : latch_volume
     *   [1:0]   = 00 : no +18 dB gain
     */
    registre = 0xD4;
    HAL_I2C_Mem_Write(&ES9038Q2M_I2C_HANDLE, ES9038Q2M_I2C_DEV_ADDR, ES9038Q2M_REG27_ADDR,
                      I2C_MEMADD_SIZE_8BIT, &registre, 1, TIMEOUT_I2C_DELAY);

    /* REG6 = 0x44: increase volume ramp rate */
    registre = 0x40;
    HAL_I2C_Mem_Write(&ES9038Q2M_I2C_HANDLE, ES9038Q2M_I2C_DEV_ADDR, ES9038Q2M_REG6_ADDR,
                      I2C_MEMADD_SIZE_8BIT, &registre, 1, TIMEOUT_I2C_DELAY);
    HAL_I2C_Mem_Read (&ES9038Q2M_I2C_HANDLE, ES9038Q2M_I2C_DEV_ADDR, ES9038Q2M_REG6_ADDR,
                      I2C_MEMADD_SIZE_8BIT, &regread,  1, TIMEOUT_I2C_DELAY);
    LOG_DBG("REG6 after init: 0x%02X", regread);


    /* Start muted; will unmute when amp powers on (even if host starts unmuted). */
    ES9038Q2M_DAC_Mute_set(true);
    return 0;
}

uint8_t ES9038Q2M_DAC_DeInit(void)
{
    LOG_INFO("ES9038Q2M_DAC_DeInit, nothing to do.");
    return 0;
}

void ES9038Q2M_DAC_Volume_change(int8_t delta)
{
    ES9038Q2M_DAC_Volume_set(requested_volume_ch1 + delta * AUDIO_VOL_RES, CHANNEL_1);
    ES9038Q2M_DAC_Volume_set(requested_volume_ch2 + delta * AUDIO_VOL_RES, CHANNEL_2);
    // Avoid to send two interrupts at the same time. Works ok, get cur done on both channels.
    USBD_AUDIO_signal_volume_change(CHANNEL_1);
}

uint8_t ES9038Q2M_DAC_Volume_set(int16_t vol, uint8_t channel) /* Q8.8 dB from USB Audio class */
{
    LOG_INFO("requested volume %d change: %d", channel, vol);
    if (vol < AUDIO_MIN_VOL)
        vol = AUDIO_MIN_VOL;
    if (vol > AUDIO_MAX_VOL)
        vol = AUDIO_MAX_VOL;
    if (channel == CHANNEL_1)
    {
        requested_volume_ch1 = vol;
    }
    else
    {
        requested_volume_ch2 = vol;
    }
    return 0;
}

/* From host, no signaling */
uint8_t ES9038Q2M_DAC_Mute_set(bool mute)
{
    if (!EtatAmp && !mute)
    {
        LOG_WARN("DAC_Mute_set request while amp is off, forcing mute");
        requested_mute = true;
        return 0; // could return USBD_FAIL ?
    }
    LOG_WARN("DAC_Mute_set request: %s", mute ? "MUTED" : "UNMUTED");
    requested_mute = mute;
    return 0;
}

HAL_StatusTypeDef ES9038Q2M_DAC_SetMute_Immediate(bool mute)
{
    HAL_StatusTypeDef st = HAL_OK;

    LOG_WARN("DAC_SetMute_Immediate: %s", mute ? "MUTED" : "UNMUTED");

    /* REG7: 0x81 = mute, 0x80 = unmute (filter bw + system mute) */
    registre = mute ? 0x81 : 0x80;

    do
    {
        st = HAL_I2C_Mem_Write(&ES9038Q2M_I2C_HANDLE, ES9038Q2M_I2C_DEV_ADDR,
                               ES9038Q2M_REG7_ADDR, I2C_MEMADD_SIZE_8BIT,
                               &registre, 1, TIMEOUT_I2C_DELAY);
        if (st == HAL_BUSY)
        {
            osDelay(1); /* yield, let other tasks free the bus */
        }
    } while (st == HAL_BUSY);

    return st;
}

/* Force mute when amp powers on/off.
 Not a host request, so signal mute change to host */
void ES9038Q2M_DAC_SetMute_Force(bool mute)
{
    LOG_WARN("DAC_SetMute_Force %s", mute ? "MUTE" : "UNMUTE");
    requested_mute = mute;
    es9038q2m_configured_mute = mute;
    ES9038Q2M_DAC_SetMute_Immediate(mute);
    USBD_AUDIO_signal_mute_change();
}

uint8_t ES9038Q2M_DAC_Format_set(uint8_t format)
{
    requested_format = format;
    return 0;
}

uint8_t ES9038Q2M_DAC_Play(void)
{
    play = 1;
    return 0;
}

uint8_t ES9038Q2M_DAC_Stop(void)
{
    return 0;
}

uint8_t convert_vol_to_register(int16_t volume_q88)
{
    /* REG15 REG16 attenuation = -0.5 dB / step.
     * es9038q2m_configured_volume is signed Q8.8 dB in [AUDIO_MIN_VOL .. AUDIO_MAX_VOL] = [-64 dB .. 0 dB].
     *   register_value = -2 * dB = -2 * (q88 / 256) = -q88 / 128
     *   yielding 0..254 for 0..-64 dB.
     */
    int32_t attenuation = -((int32_t)volume_q88) / 64; //  /256 for q8.8, and x4 from -64/0 to 0/255
    if (attenuation < 0)
        attenuation = 0;
    if (attenuation > 255)
        attenuation = 255;
    LOG_INFO("applying volume change: %d (registre=%u, %ddB)",
        volume_q88,
        (uint8_t)attenuation,
         -attenuation / 2);
    return (uint8_t)attenuation;
}

void ES9038Q2M_ProcessEvents(void)
{
    /* Process audio events in task context, in order of priority.
     * REG96 status bits (read every 10 calls):
     *   [3] dop_valid    : DoP decoder has detected a valid DoP signal
     *   [2] spdif_valid  : SPDIF decoder has detected a valid SPDIF signal
     *   [1] i2s_valid    : I²S decoder has valid frame clock + bit clock
     *   [0] dsd_select   : DSD decoder is being used as fallback
     */
    static HAL_StatusTypeDef I2C_Status = HAL_OK;
    static uint32_t cnt = 0;

    osDelay(5); // ms
    cnt++;
    if (cnt % 10 == 0)
    {
        I2C_Status = HAL_I2C_Mem_Read(&ES9038Q2M_I2C_HANDLE, ES9038Q2M_I2C_DEV_ADDR,
                                      ES9038Q2M_REG96_ADDR, I2C_MEMADD_SIZE_8BIT,
                                      &status_register, 1, TIMEOUT_I2C_DELAY);
        if (I2C_Status != HAL_OK)
        {
            Error_Handler_nonBlocking("I2C read failure", ERROR_I2C);
            HAL_I2C_DeInit(&ES9038Q2M_I2C_HANDLE);
            MX_I2C1_Init();
            I2C_Status = HAL_OK; /* retry after re-init */
        }
        else
        {
            Error_cancel_nonBlocking(ERROR_I2C);
        }
    }

    if (es9038q2m_audio_stop_pending)
    {
        es9038q2m_audio_stop_pending = false;
        USBD_AUDIO_fops.AUDIO_Cmd(NULL, 0, AUDIO_CMD_STOP);
    }

    if (play)
    {
        play = 0;
    }

    if (requested_mute != es9038q2m_configured_mute)
    {
        if (ES9038Q2M_I2C_HANDLE.State == HAL_I2C_STATE_READY) 
        {
            /* enforce mute if amp is off, even if host requested unmute */
            
            es9038q2m_configured_mute = requested_mute;
            LOG_WARN("applying mute change: %s", es9038q2m_configured_mute ? "MUTED" : "UNMUTED");
            I2C_Status = ES9038Q2M_DAC_SetMute_Immediate(es9038q2m_configured_mute);
        }
    }

    if (requested_format != configured_format)
    {
        /* Format is auto-detected by ES9038 via REG1 auto_select; nothing to write. */
        configured_format = requested_format;
    }

    // Handle volume changes:
    if (   (cnt % 20 == 0)
        && (ES9038Q2M_I2C_HANDLE.State == HAL_I2C_STATE_READY)
        && (   (requested_volume_ch1 != es9038q2m_configured_volume_ch1)
            || (requested_volume_ch2 != es9038q2m_configured_volume_ch2)))
    {
        uint8_t reg_val;
        es9038q2m_configured_volume_ch1 = requested_volume_ch1;
        reg_val = convert_vol_to_register(es9038q2m_configured_volume_ch1);
        I2C_Status = HAL_I2C_Mem_Write(&ES9038Q2M_I2C_HANDLE, ES9038Q2M_I2C_DEV_ADDR,
                                       ES9038Q2M_REG15_ADDR, I2C_MEMADD_SIZE_8BIT,
                                       &reg_val, 1, TIMEOUT_I2C_DELAY);

        es9038q2m_configured_volume_ch2 = requested_volume_ch2;
        reg_val = convert_vol_to_register(es9038q2m_configured_volume_ch2);
        I2C_Status = HAL_I2C_Mem_Write(&ES9038Q2M_I2C_HANDLE, ES9038Q2M_I2C_DEV_ADDR,
                                       ES9038Q2M_REG16_ADDR, I2C_MEMADD_SIZE_8BIT,
                                       &reg_val, 1, TIMEOUT_I2C_DELAY);
    }
}
