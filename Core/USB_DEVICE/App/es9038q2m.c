#include "es9038q2m.h"
#include "main.h"
#include "cmsis_os.h"
#define LOG_LEVEL LOG_LEVEL_DBG
#include "log.h"

#define TIMEOUT_I2C_DELAY   10   /* ms; could be HAL_MAX_DELAY for infinite */

static uint8_t play;

volatile int16_t requested_volume_ch1 = AUDIO_CUR_VOL;                /* set at boot, host range */
volatile int16_t requested_volume_ch2 = AUDIO_CUR_VOL;                /* set at boot, host range */
volatile int16_t es9038q2m_configured_volume_ch1 = AUDIO_CUR_VOL + 1; /* differ → force apply on first ProcessEvents */
volatile int16_t es9038q2m_configured_volume_ch2 = AUDIO_CUR_VOL + 1; /* differ → force apply on first ProcessEvents */
volatile bool requested_mute = true;                                  /* unmuted when amp powers on */
volatile bool es9038q2m_configured_mute = true;                       /* will be forced at startup as amps are off */
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
    HAL_I2C_Mem_Read (&DAC_I2C_Handle, ES9038Q2M_I2C_DEV_ADDR, ES9038Q2M_REG14_ADDR,
                      I2C_MEMADD_SIZE_8BIT, &regread,  1, TIMEOUT_I2C_DELAY);
    HAL_I2C_Mem_Write(&DAC_I2C_Handle, ES9038Q2M_I2C_DEV_ADDR, ES9038Q2M_REG14_ADDR,
                      I2C_MEMADD_SIZE_8BIT, &registre, 1, TIMEOUT_I2C_DELAY);
    HAL_I2C_Mem_Read (&DAC_I2C_Handle, ES9038Q2M_I2C_DEV_ADDR, ES9038Q2M_REG14_ADDR,
                      I2C_MEMADD_SIZE_8BIT, &regread,  1, TIMEOUT_I2C_DELAY);

    /* REG27 = 0xBC: ASRC enable, link L/R volume, latch volume, no +18 dB gain.
     *   bit 7   = 1  : asrc_en
     *   [6:5]   = 10 : reserved defaults
     *   bit 4   = 1  : reserved default
     *   bit 3   = 0  : ch1_volume (Allow independent control)
     *   bit 2   = 1  : latch_volume off
     *   [1:0]   = 00 : no +18 dB gain
     */
    registre = 0xD4;
    HAL_I2C_Mem_Write(&DAC_I2C_Handle, ES9038Q2M_I2C_DEV_ADDR, ES9038Q2M_REG27_ADDR,
                      I2C_MEMADD_SIZE_8BIT, &registre, 1, TIMEOUT_I2C_DELAY);

    /* REG6 = 0x44: increase volume ramp rate
        Bit Mnemonic    Description
    [7] auto_deemph
        Automatically engages the de-emphasis filters when SPDIF data is provides and the SPDIF channel status bits contains valid de-emphasis settings.
        1'b1: enables automatic de-emphasis
        1'b0: disables automatic de-emphasis (default)
    [6] deemph_bypass
        Enables or disables the built-in de-emphasis filters.
        1'b1 disabled de-emphasis filters (default)
        1'b0 enables de-emphasis filters
    [5:4] deemph_sel
        Selects which de-emphasis filter is used.
        2'b11: reserved
        2'b10: 48kHz
        2'b01: 44.1kHz
        2'b00: 32kHz (default)
    [3] dop_enable
        Selects whether the DSD over PCM (DOP) logic is enabled.
        1'b0: disables the DoP logic
        1'b1: enables the DoP logic
    [2:0] volume_rate
        Selects a volume ramp rate to use when transitioning between different volume levels. The volume ramp rate is measured in decibels per second (dB/s).
    */
    registre = 0xC8; // auto_deemph + dop_enable + volume_rate=0b000 (slowest ramp rate)
    HAL_I2C_Mem_Write(&DAC_I2C_Handle, ES9038Q2M_I2C_DEV_ADDR, ES9038Q2M_REG6_ADDR,
                      I2C_MEMADD_SIZE_8BIT, &registre, 1, TIMEOUT_I2C_DELAY);
    HAL_I2C_Mem_Read (&DAC_I2C_Handle, ES9038Q2M_I2C_DEV_ADDR, ES9038Q2M_REG6_ADDR,
                      I2C_MEMADD_SIZE_8BIT, &regread,  1, TIMEOUT_I2C_DELAY);
    LOG_DBG("REG6 after init: 0x%02X", regread);


    /* Start muted; will unmute when amp powers on (even if host starts unmuted). */
    ES9038Q2M_DAC_SetMute_Force(true);
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
    USBD_AUDIO_signal_volume_change();
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
    else if (channel == CHANNEL_2)
    {
        requested_volume_ch2 = vol;
    }
    else
    {
        requested_volume_ch1 = vol;
        requested_volume_ch2 = vol;
    }
    return 0;
}

/* From host, no signaling */
uint8_t ES9038Q2M_DAC_Mute_set(bool mute)
{
    if (EtatAmp != AMP_ON && !mute)
    {
        LOG_WARN("DAC_Mute_set request while amp not ON, forcing mute");
        requested_mute = true;
        return 0;
    }
    LOG_INFO("DAC_Mute_set request: %s", mute ? "MUTED" : "UNMUTED");
    requested_mute = mute;
    return 0;
}

HAL_StatusTypeDef ES9038Q2M_DAC_SetMute_Immediate(bool mute)
{
    HAL_StatusTypeDef st = HAL_OK;

    LOG_INFO("DAC_SetMute_Immediate: %s", mute ? "MUTED" : "UNMUTED");

    /* REG7: 0x81 = mute, 0x80 = unmute (filter bw + system mute) */
    registre = mute ? 0x81 : 0x80;

    do
    {
        st = HAL_I2C_Mem_Write(&DAC_I2C_Handle, ES9038Q2M_I2C_DEV_ADDR,
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
    LOG_INFO("DAC_SetMute_Force %s", mute ? "MUTE" : "UNMUTE");
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


/* Under the Amps LEDs:
 *   Green: volume-driven (avg of L/R requested volumes), off when muted
 *          or amp off. Linear in dB; gamma LUT in the LED API handles
 *          perceptual linearity.
 *   Red  : on (dim) only when amp is on AND muted; off otherwise.
 */
static void ES9038Q2M_UpdateLeds(void)
{
    uint8_t g_pct, r_pct;

    if (EtatAmp == AMP_OFF)
    {
        g_pct = 0;
        r_pct = 0;
    }
    else if (requested_mute)
    {
        g_pct = MUTE_LED_POWER;
        r_pct = 0;
    }
    else
    {
        int32_t avg = ((int32_t)requested_volume_ch1 +
                       (int32_t)requested_volume_ch2) / 2;

        if (avg < AUDIO_MIN_VOL) avg = AUDIO_MIN_VOL;
        if (avg > AUDIO_MAX_VOL) avg = AUDIO_MAX_VOL;

        int32_t span = (int32_t)AUDIO_MAX_VOL - (int32_t)AUDIO_MIN_VOL;
        r_pct = (uint8_t)(((avg - AUDIO_MIN_VOL) * 100 + (span / 2)) / span);
        g_pct = 0;
    }

    Led_G_SetBrightness(g_pct);
    Led_R_SetBrightness(r_pct);
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

    ES9038Q2M_UpdateLeds();

    if (cnt % 10 == 0)
    {
        uint8_t new_status_register;
        I2C_Status = HAL_I2C_Mem_Read(&DAC_I2C_Handle, ES9038Q2M_I2C_DEV_ADDR,
                                      ES9038Q2M_REG96_ADDR, I2C_MEMADD_SIZE_8BIT,
                                      &new_status_register, 1, TIMEOUT_I2C_DELAY);
        if (I2C_Status != HAL_OK)
        {
            Error_Handler_nonBlocking("I2C read failure", ERROR_I2C);
            HAL_I2C_DeInit(&DAC_I2C_Handle);
            MX_I2C1_Init(&DAC_I2C_Handle);
            I2C_Status = HAL_OK; /* retry after re-init */
        }
        else
        {
            Error_cancel_nonBlocking(ERROR_I2C);
            // Display info on changes :
            if (new_status_register != status_register)
            {
                LOG_DBG("Status change: 0x%02X -> 0x%02X",
                    status_register, new_status_register);
                if ((new_status_register & ES9038Q2M_STAT_DOP_VALID) != (status_register & ES9038Q2M_STAT_DOP_VALID))
                {
                    LOG_WARN("  DOP decoder %s", (new_status_register & ES9038Q2M_STAT_DOP_VALID) ? "VALID" : "INVALID");
                }
                if ((new_status_register & ES9038Q2M_STAT_SPDIF_VALID) != (status_register & ES9038Q2M_STAT_SPDIF_VALID))
                {
                    LOG_WARN("  SPDIF decoder %s", (new_status_register & ES9038Q2M_STAT_SPDIF_VALID) ? "VALID" : "INVALID");
                }
                if ((new_status_register & ES9038Q2M_STAT_I2S_VALID) != (status_register & ES9038Q2M_STAT_I2S_VALID))
                {
                    LOG_WARN("  I2S decoder %s", (new_status_register & ES9038Q2M_STAT_I2S_VALID) ? "VALID" : "INVALID");
                }
                if ((new_status_register & ES9038Q2M_STAT_DSD_VALID) != (status_register & ES9038Q2M_STAT_DSD_VALID))
                {
                    LOG_WARN("  DSD decoder %s", (new_status_register & ES9038Q2M_STAT_DSD_VALID) ? "VALID" : "INVALID");
                }
                status_register = new_status_register;
            }
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
        if (DAC_I2C_Handle.State == HAL_I2C_STATE_READY) 
        {
            /* enforce mute if amp is off, even if host requested unmute */
            
            es9038q2m_configured_mute = requested_mute;
            LOG_INFO("applying mute change: %s", es9038q2m_configured_mute ? "MUTED" : "UNMUTED");
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
        && (DAC_I2C_Handle.State == HAL_I2C_STATE_READY)
        && (   (requested_volume_ch1 != es9038q2m_configured_volume_ch1)
            || (requested_volume_ch2 != es9038q2m_configured_volume_ch2)))
    {
        uint8_t reg_val;
        es9038q2m_configured_volume_ch1 = requested_volume_ch1;
        reg_val = convert_vol_to_register(es9038q2m_configured_volume_ch1);
        I2C_Status = HAL_I2C_Mem_Write(&DAC_I2C_Handle, ES9038Q2M_I2C_DEV_ADDR,
                                       ES9038Q2M_REG15_ADDR, I2C_MEMADD_SIZE_8BIT,
                                       &reg_val, 1, TIMEOUT_I2C_DELAY);

        es9038q2m_configured_volume_ch2 = requested_volume_ch2;
        reg_val = convert_vol_to_register(es9038q2m_configured_volume_ch2);
        I2C_Status = HAL_I2C_Mem_Write(&DAC_I2C_Handle, ES9038Q2M_I2C_DEV_ADDR,
                                       ES9038Q2M_REG16_ADDR, I2C_MEMADD_SIZE_8BIT,
                                       &reg_val, 1, TIMEOUT_I2C_DELAY);
    }
}