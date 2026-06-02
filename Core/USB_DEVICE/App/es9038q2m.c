#include "es9038q2m.h"
#include "main.h"
#include "cmsis_os.h"
#define LOG_LEVEL LOG_LEVEL_INFO
#include "log.h"

#define TIMEOUT_I2C_DELAY   10   /* ms; could be HAL_MAX_DELAY for infinite */

extern I2C_HandleTypeDef ES9038Q2M_I2C_HANDLE;

static uint8_t play;

volatile int16_t requested_volume = AUDIO_CUR_VOL;                /* set at boot, host range */
volatile int16_t es9038q2m_configured_volume = AUDIO_CUR_VOL + 1; /* differ → force apply on first ProcessEvents */
volatile bool requested_mute = false;                             /* unmuted when amp powers on */
volatile bool es9038q2m_configured_mute = false;                  /* set true at init since amp is off */
volatile bool es9038q2m_audio_stop_pending = false;

AUDIO_FormatTypeDef requested_format  = AUDIO_FORMAT_PCM;
AUDIO_FormatTypeDef configured_format = AUDIO_FORMAT_DSD;  /* differ → force apply on first call */

uint8_t regread;
uint8_t status_register = 0;
uint8_t registre;

const AUDIO_CodecTypeDef es9038q2m_instance =
    {
        ES9038Q2M_DAC_Init,
        NULL,
        ES9038Q2M_DAC_Play,
        ES9038Q2M_DAC_Format_set,
        ES9038Q2M_DAC_Stop,
        NULL,
        ES9038Q2M_DAC_Mute_set,
        ES9038Q2M_DAC_Volume_set};

uint8_t ES9038Q2M_DAC_Init(void)
{
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
     *   bit 3   = 1  : ch1_volume (ch2 follows ch1)
     *   bit 2   = 1  : latch_volume
     *   [1:0]   = 00 : no +18 dB gain
     */
    registre = 0xBC;
    HAL_I2C_Mem_Write(&ES9038Q2M_I2C_HANDLE, ES9038Q2M_I2C_DEV_ADDR, ES9038Q2M_REG27_ADDR,
                      I2C_MEMADD_SIZE_8BIT, &registre, 1, TIMEOUT_I2C_DELAY);

    /* Start muted; will unmute when amp powers on (even if host starts unmuted). */
    ES9038Q2M_DAC_SetMute_Force();
    return 0;
}

uint8_t ES9038Q2M_DAC_Volume_set(int16_t vol)   /* Q8.8 dB from USB Audio class */
{
    LOG_INFO("requested volume change: %d", vol);
    if (vol < AUDIO_MIN_VOL)
        vol = AUDIO_MIN_VOL;
    if (vol > AUDIO_MAX_VOL)
        vol = AUDIO_MAX_VOL;
    requested_volume = vol;
    return 0;
}

uint8_t ES9038Q2M_DAC_Mute_set(uint8_t mute)
{
    requested_mute = (mute == 1);
    return 0;
}

HAL_StatusTypeDef ES9038Q2M_DAC_SetMute_Immediate(uint8_t mute)
{
    HAL_StatusTypeDef st = HAL_OK;

    /* REG7: 0x81 = mute, 0x80 = unmute (filter bw + system mute) */
    registre = mute ? 0x81 : 0x80;

    do {
        st = HAL_I2C_Mem_Write(&ES9038Q2M_I2C_HANDLE, ES9038Q2M_I2C_DEV_ADDR,
                               ES9038Q2M_REG7_ADDR, I2C_MEMADD_SIZE_8BIT,
                               &registre, 1, TIMEOUT_I2C_DELAY);
        if (st == HAL_BUSY) {
            osDelay(1);                /* yield, let other tasks free the bus */
        }
    } while (st == HAL_BUSY);

    return st;
}

/* Force mute when amp powers off, but remember the host's requested state
 * so we can restore it when the amp powers back on. */
void ES9038Q2M_DAC_SetMute_Force(void)
{
    requested_mute = es9038q2m_configured_mute || requested_mute;
    ES9038Q2M_DAC_SetMute_Immediate(true);

    es9038q2m_configured_mute = true;
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
        if ((ES9038Q2M_I2C_HANDLE.State == HAL_I2C_STATE_READY) && EtatAmp)
        {
            es9038q2m_configured_mute = requested_mute;
            LOG_INFO("applying mute change: %d", es9038q2m_configured_mute);
            I2C_Status = ES9038Q2M_DAC_SetMute_Immediate(es9038q2m_configured_mute);
            USBD_AUDIO_signal_mute_change();
        }
    }

    if (requested_format != configured_format)
    {
        /* Format is auto-detected by ES9038 via REG1 auto_select; nothing to write. */
        configured_format = requested_format;
    }

    if (requested_volume != es9038q2m_configured_volume)
    {
        if (ES9038Q2M_I2C_HANDLE.State == HAL_I2C_STATE_READY)
        {
            es9038q2m_configured_volume = requested_volume;

            /* REG15 attenuation = -0.5 dB / step.
             * es9038q2m_configured_volume is signed Q8.8 dB in [AUDIO_MIN_VOL .. AUDIO_MAX_VOL] = [-60 dB .. 0 dB].
             *   register_value = -2 * dB = -2 * (q88 / 256) = -q88 / 128
             *   yielding 0..120 for 0..-60 dB.
             * REG16 not written — REG27 ch1_volume bit makes ch2 follow ch1.
             */
            int32_t attenuation = -((int32_t)es9038q2m_configured_volume) / 128;
            if (attenuation < 0)
                attenuation = 0;
            if (attenuation > 255)
                attenuation = 255;
            uint8_t reg_val = (uint8_t)attenuation;
            LOG_INFO("applying volume change: %d (attenuation=%u)", es9038q2m_configured_volume, reg_val);

            I2C_Status = HAL_I2C_Mem_Write(&ES9038Q2M_I2C_HANDLE, ES9038Q2M_I2C_DEV_ADDR,
                                           ES9038Q2M_REG15_ADDR, I2C_MEMADD_SIZE_8BIT,
                                           &reg_val, 1, TIMEOUT_I2C_DELAY);
        }
    }
}
