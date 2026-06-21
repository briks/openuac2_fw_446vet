#include <string.h>
#include <stdio.h>

#include "es9038q2m.h"
#include "pga2311.h"
#include "main.h"
#include "cmsis_os.h"
#define LOG_LEVEL LOG_LEVEL_DBG
#include "log.h"

#define TIMEOUT_I2C_DELAY   10   /* ms; could be HAL_MAX_DELAY for infinite */

static uint8_t play;
static bool DAC_initialized = false;

volatile int16_t requested_volume_ch1 = AUDIO_CUR_VOL;                /* set at boot, host range */
volatile int16_t requested_volume_ch2 = AUDIO_CUR_VOL;                /* set at boot, host range */
volatile int16_t es9038q2m_configured_volume_ch1 = AUDIO_CUR_VOL + 1; /* differ → force apply on first ProcessEvents */
volatile int16_t es9038q2m_configured_volume_ch2 = AUDIO_CUR_VOL + 1; /* differ → force apply on first ProcessEvents */
volatile bool requested_mute = true;                                  /* unmuted when amp powers on */
volatile bool es9038q2m_configured_mute = true;                       /* will be forced at startup as amps are off */
volatile bool es9038q2m_audio_stop_pending = false;
volatile bool es9038q2m_spdif_present = false;   /* updated from REG70-93 channel status */

AUDIO_FormatTypeDef requested_format  = AUDIO_FORMAT_PCM;
AUDIO_FormatTypeDef configured_format = AUDIO_FORMAT_DSD;  /* differ → force apply on first call */

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
    uint8_t registre;
    uint8_t regread;

    // Init DAC only once.
    // Keep init in USB in case of Deinit/Init is called (reinit)
    if (DAC_initialized)
    {
        LOG_INFO("ES9038Q2M_DAC_Init... skipped, already initialized");
        return USBD_OK;
    }

    LOG_INFO("ES9038Q2M_DAC_Init...");
    LL_GPIO_ResetOutputPin(DAC_RST_GPIO_Port, DAC_RST_Pin);
    HAL_Delay(10);                    /* may be called from ISR context; osDelay not allowed */
    LL_GPIO_SetOutputPin(DAC_RST_GPIO_Port, DAC_RST_Pin);
    HAL_Delay(100);

    /* GPIO1 = Standard Input (high-Z) so the SPDIF decoder can read it.
     * GPIO2 kept at its reset default (4'd13 Analog Input Shutdown) since PA9
     * is wired to it but unused for now. */
    registre = REG8_GPIO2_CFG_ANA_SHUTDOWN | REG8_GPIO1_CFG_STD_INPUT;   /* 0xD8 */
    HAL_I2C_Mem_Write(&DAC_I2C_Handle, ES9038Q2M_I2C_DEV_ADDR, ES9038Q2M_REG8_ADDR,
                      I2C_MEMADD_SIZE_8BIT, &registre, 1, TIMEOUT_I2C_DELAY);

    registre = REG11_SPDIF_SEL_GPIO1;
    HAL_I2C_Mem_Write(&DAC_I2C_Handle, ES9038Q2M_I2C_DEV_ADDR, ES9038Q2M_REG11_ADDR,
                      I2C_MEMADD_SIZE_8BIT, &registre, 1, TIMEOUT_I2C_DELAY);

    registre = REG21_GPIO_SEL1_SPDIF;      /* GPIO2 left default (serial) */
    HAL_I2C_Mem_Write(&DAC_I2C_Handle, ES9038Q2M_I2C_DEV_ADDR, ES9038Q2M_REG21_ADDR,
                      I2C_MEMADD_SIZE_8BIT, &registre, 1, TIMEOUT_I2C_DELAY);

    /* REG1: known input state at boot = I2S (auto-select DSD/serial, SPDIF
     * excluded). SPDIF is exercised on demand only, via Spdif_Probe(). */
    ES9038Q2M_DAC_SetInput(ES9038Q2M_INPUT_I2S);

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

    /* REG12: DPLL bandwidth. The BT module is the I2S master and runs from its
     * own crystal, so its BCLK/WS are asynchronous to the DAC MCLK and carry
     * jitter. The reset default is tuned for a clean synchronous source (USB)
     * and is too narrow here: the residual jitter is audible as crackle that
     * grows with signal frequency (worst in the treble). Widen the serial DPLL
     * bandwidth so the DPLL/ASRC tracks the jittery clock cleanly.
     *   [7:4] = serial bandwidth (try 0x0A, raise to 0x0F if still noisy)
     *   [3:0] = DSD bandwidth (kept the same)
     * Pick the narrowest value that removes the crackle: too wide lets through
     * a bit more noise floor, too narrow lets jitter/unlock artifacts pass. */
    // registre = 0x5A;
    // HAL_I2C_Mem_Write(&DAC_I2C_Handle, ES9038Q2M_I2C_DEV_ADDR, ES9038Q2M_REG12_ADDR,
    //                   I2C_MEMADD_SIZE_8BIT, &registre, 1, TIMEOUT_I2C_DELAY);
    // HAL_I2C_Mem_Read (&DAC_I2C_Handle, ES9038Q2M_I2C_DEV_ADDR, ES9038Q2M_REG12_ADDR,
    //                   I2C_MEMADD_SIZE_8BIT, &regread,  1, TIMEOUT_I2C_DELAY);
    // LOG_WARN("REG12 (DPLL bw) after init: 0x%02X", regread);

    /* Start muted; will unmute when amp powers on (even if host starts unmuted). */
    ES9038Q2M_DAC_SetMute_Force(true);
    return 0;
}

uint8_t ES9038Q2M_DAC_DeInit(void)
{
    LOG_INFO("ES9038Q2M_DAC_DeInit, nothing to do.");
    DAC_initialized = false;
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
    LOG_DBG("requested volume %d change: %d", channel, vol);
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
    uint8_t registre;

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
    if ((EtatAmp != AMP_ON) && !mute)
    {
        LOG_DBG("DAC_SetMute_Force while amp not ON, ignoring");
        return;
    }
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
    else if (requested_mute && (current_source != SOURCE_LINE))
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

/* Reverse the low 'nbits' of 'val' (LSB<->MSB within that field). */
static uint8_t ES9038Q2M_BitReverse(uint8_t val, uint8_t nbits)
{
    uint8_t r = 0;
    for (uint8_t i = 0; i < nbits; i++)
    {
        r = (uint8_t)((r << 1) | (val & 1U));
        val >>= 1;
    }
    return r;
}

/* Print a 4-char 7-bit-ASCII label (origin/destination), bytes b..b+3.
  * Non-printable bytes are shown as '.'. */
static void ES9038Q2M_FormatAsciiLabel(const uint8_t *p, char out[5])
{
    for (uint8_t i = 0; i < 4; i++)
    {
        uint8_t c = p[i] & 0x7F;        /* top bit is parity-zero per spec */
        out[i] = (c >= 0x20 && c < 0x7F) ? (char)c : '.';
    }
    out[4] = '\0';
}

static void ES9038Q2M_DecodeSpdifChannelStatusPro(const uint8_t *cs);

/* Decode the ES9038Q2M SPDIF channel-status registers (REG70..REG93).
  * Layout per ES9038Q2M datasheet (IEC 60958). 'cs' = 24 raw bytes. */
static void ES9038Q2M_DecodeSpdifChannelStatus(const uint8_t *cs)
{
    /* Byte 0 (REG70): basic control ------------------------------------- */
    bool professional = (cs[0] & 0x01) != 0;   /* bit0: 0=consumer 1=pro    */
    bool data         = (cs[0] & 0x02) != 0;   /* bit1: 0=audio   1=data    */
    bool copyrighted  = (cs[0] & 0x04) == 0;   /* bit2: 0=copyright asserted*/
    bool preemphasis  = (cs[0] & 0x08) != 0;   /* bit3: 0=none 1=pre-emph   */
    bool four_channel = (cs[0] & 0x20) != 0;   /* bit5: 0=2ch 1=4ch         */

    if (professional)
    {
        ES9038Q2M_DecodeSpdifChannelStatusPro(cs);
        return;
    }

    /* Byte 1 (REG71): category code ------------------------------------- */
    const char *cat;
    switch (cs[1])
    {
        case 0x00: cat = "General";              break;
        case 0x01: cat = "Laser-Optical";        break;
        case 0x02: cat = "D/D Converter";        break;
        case 0x03: cat = "Magnetic";             break;
        case 0x04: cat = "Digital Broadcast";    break;
        case 0x05: cat = "Musical Instrument";   break;
        case 0x06: cat = "Present A/D Conv";     break;
        case 0x08: cat = "Solid State Memory";   break;
        case 0x16: cat = "Future A/D Conv";      break;
        case 0x19: cat = "DVD";                  break;
        case 0x40: cat = "Experimental";         break;
        default:   cat = "unknown";              break;
    }

    /* Byte 2 (REG72): channel number (hi nibble) / source number (lo) --- */
    uint8_t chan_num = (cs[2] >> 4) & 0x0F;
    uint8_t src_num  =  cs[2]       & 0x0F;

    /* Byte 3 (REG73): clock accuracy [5:4], sample frequency [3:0] ------ */
    uint8_t clk_acc = (cs[3] >> 4) & 0x03;
    uint8_t fs_code =  cs[3]       & 0x0F;
    const char *fs_str;
    switch (fs_code)
    {
        case 0x0: fs_str = "44.1 kHz";  break;
        case 0x2: fs_str = "48 kHz";    break;
        case 0x3: fs_str = "32 kHz";    break;
        case 0x4: fs_str = "22.05 kHz"; break;
        case 0x6: fs_str = "24 kHz";    break;
        case 0x8: fs_str = "88.2 kHz";  break;
        case 0xA: fs_str = "96 kHz";    break;
        case 0xC: fs_str = "176.4 kHz"; break;
        case 0xE: fs_str = "192 kHz";   break;
        default:  fs_str = "unknown";   break;
    }
    const char *clk_str;
    switch (clk_acc)
    {
        case 0x0: clk_str = "Level2 1000ppm";          break;
        case 0x1: clk_str = "Level1 50ppm";            break;
        case 0x2: clk_str = "Level3 variable pitch";   break;
        default:  clk_str = "reserved";                break;
    }

    /* Byte 4 (REG74): word length [3:1], word field size bit0 ----------- */
    bool    max24   = (cs[4] & 0x01) != 0;     /* 0=max20bit 1=max24bit     */
    uint8_t wl_code = (cs[4] >> 1) & 0x07;      /* field value (see mapping) */
    #if defined (REVERSE_ORDER)
    /* Mapping derived from the datasheet's LSB-first code table:
      *   value : max20 / max24
      *     0   : not indicated
      *     1   : 23 / 19 bits
      *     2   : 22 / 18 bits
      *     3   : 21 / 17 bits
      *     4   : 20 / 16 bits
      *     5   : 24 / 20 bits                                              */
    static const uint8_t wl_max20[6] = { 0, 23, 22, 21, 20, 24 };
    static const uint8_t wl_max24[6] = { 0, 19, 18, 17, 16, 20 };
    #else
    /* Mapping derived from the datasheet's:
      *   value : max20 / max24
      *     0   : not indicated
      *     1   : 20 / 16 bits
      *     2   : 22 / 18 bits
      *     3   : 0  / 0 bits
      *     4   : 23 / 19 bits
      *     5   : 24 / 20 bits                                              
      *     6   : 21 / 17 bits
      *     7   : 0  / 0 bits */
    static const uint8_t wl_max20[8] = { 0, 16, 18, 0, 19, 20, 17, 0 };
    static const uint8_t wl_max24[8] = { 0, 20, 22, 0, 23, 24, 21, 0 };
    #endif

    uint8_t word_bits = 0;
    if (wl_code < 6)
        word_bits = max24 ? wl_max24[wl_code] : wl_max20[wl_code];

    /* --- log ---------------------------------------------------------- */
    LOG_INFO("  SPDIF: %s, %s, copyright=%s, pre-emph=%s, %s",
              data ? "DATA" : "audio",
              "consumer",
              copyrighted ? "yes" : "no",
              preemphasis ? "yes" : "no",
              four_channel ? "4ch" : "2ch");
    LOG_INFO("  SPDIF: category=%s (0x%02X), chan=0x%X, src=0x%X",
              cat, cs[1], chan_num, src_num);
    LOG_INFO("  SPDIF: Fs=%s (0x%X), clock=%s", fs_str, fs_code, clk_str);
    if (word_bits)
        LOG_INFO("  SPDIF: word length=%u bits (max %s)",
                  word_bits, max24 ? "24" : "20");
    else
        LOG_INFO("  SPDIF: word length not indicated (max %s)",
                  max24 ? "24" : "20");
}

/* Decode the ES9038Q2M SPDIF channel-status registers in PROFESSIONAL
  * (AES3) configuration. 'cs' = 24 raw bytes (REG70..REG93).
  *
  * NOTE: several fields are transmitted bit-reversed; the code tables below
  * use the value as read by the masks shown. Verify against a known AES3
  * source before trusting word-length / multichannel decode. */
static void ES9038Q2M_DecodeSpdifChannelStatusPro(const uint8_t *cs)
{
    /* Byte 0 (REG70) ---------------------------------------------------- */
    bool    non_audio = (cs[0] & 0x02) != 0;        /* bit1                */
    uint8_t emph      = (cs[0] >> 2) & 0x07;        /* bits[4:2]           */
    bool    unlocked  = (cs[0] & 0x20) != 0;        /* bit5: 0=locked      */
    uint8_t fs0       = (cs[0] >> 6) & 0x03;        /* bits[7:6]           */

    const char *emph_str;
    switch (emph)
    {
        case 0x0: emph_str = "not indicated"; break;
        case 0x1: emph_str = "none";          break;
        case 0x3: emph_str = "CD-type";       break;
        case 0x7: emph_str = "J-17";          break;
        default:  emph_str = "reserved";      break;
    }
    const char *fs0_str;
    switch (fs0)
    {
        case 0x0: fs0_str = "not indicated (see byte4)"; break;
        case 0x2: fs0_str = "48 kHz";                    break;
        case 0x1: fs0_str = "44.1 kHz";                  break;
        case 0x3: fs0_str = "32 kHz";                    break;
        default:  fs0_str = "unknown";                   break;
    }

    /* Byte 1 (REG71): user-bit mgmt [7:4], channel mode [3:0] ----------- */
    uint8_t user_mgmt = (cs[1] >> 4) & 0x0F;
    uint8_t chan_mode =  cs[1]       & 0x0F;
    const char *user_str;
    switch (user_mgmt)
    {
        case 0x0: user_str = "no indication";        break;
        case 0x8: user_str = "192-bit block";        break;
        case 0x4: user_str = "AES18";                break;
        case 0xC: user_str = "user-defined";         break;
        case 0x2: user_str = "IEC60958-3 (consumer)";break;
        default:  user_str = "reserved";             break;
    }
    const char *mode_str;
    switch (chan_mode)
    {
        case 0x0: mode_str = "not indicated (2ch)";   break;
        case 0x8: mode_str = "2 channel";             break;
        case 0x4: mode_str = "1 channel (mono)";      break;
        case 0xC: mode_str = "primary/secondary";     break;
        case 0x2: mode_str = "stereo";                break;
        case 0xE: mode_str = "SCDSR (see byte3)";     break;
        case 0x1: mode_str = "SCDSR stereo left";     break;
        case 0x9: mode_str = "SCDSR stereo right";    break;
        case 0xF: mode_str = "multichannel (byte3)";  break;
        default:  mode_str = "reserved/user";         break;
    }

    /* Byte 2 (REG72): align [7:6], src word len [5:3], aux use [2:0] ---- */
    uint8_t align_lvl = (cs[2] >> 6) & 0x03;
    uint8_t wl_code   = (cs[2] >> 3) & 0x07;
    uint8_t aux_use   =  cs[2]       & 0x07;
    const char *align_str;
    switch (align_lvl)
    {
        case 0x0: align_str = "not indicated"; break;
        case 0x2: align_str = "-20 dBFS";      break;
        case 0x1: align_str = "-18.06 dBFS";   break;
        default:  align_str = "reserved";      break;
    }
    /* aux_use also tells us the max word length:
      *   0x4 = main audio, max 24 bits; else max 20 bits */
    bool max24 = (aux_use == 0x4);
    const char *aux_str;
    switch (aux_use)
    {
        case 0x0: aux_str = "not defined (audio max20)"; break;
        case 0x4: aux_str = "main audio (max24)";        break;
        case 0x2: aux_str = "coordination (audio max20)";break;
        case 0x6: aux_str = "reserved";                  break;
        default:  aux_str = "reserved";                  break;
    }
    /* Source word length: same code table as consumer byte4 */
    uint8_t word_bits;
    switch (wl_code)
    {
        case 0x4: word_bits = max24 ? 19 : 23; break;
        case 0x2: word_bits = max24 ? 18 : 22; break;
        case 0x6: word_bits = max24 ? 17 : 21; break;
        case 0x1: word_bits = max24 ? 16 : 20; break;
        case 0x5: word_bits = max24 ? 20 : 24; break;
        default:  word_bits = 0;               break; /* not indicated */
    }

    /* Byte 3 (REG73): channel identification --------------------------- */
    /* bit7=0: channel number = 1 + bitreverse(bits[6:0])
      * bit7=1: bits[6:4] = multichannel mode, bits[3:0] bit-reversed = number */
    char chan_id_str[48];
    if ((cs[3] & 0x80) == 0)
    {
        uint8_t num = ES9038Q2M_BitReverse(cs[3] & 0x7F, 7);
        snprintf(chan_id_str, sizeof(chan_id_str), "channel %u", (unsigned)(num + 1));
    }
    else
    {
        uint8_t mc_mode = (cs[3] >> 4) & 0x07;
        uint8_t mc_num  = ES9038Q2M_BitReverse(cs[3] & 0x0F, 4);
        snprintf(chan_id_str, sizeof(chan_id_str),
                  "multichannel mode %u, channel %u",
                  (unsigned)mc_mode, (unsigned)mc_num);
    }

    /* Byte 4 (REG74): fs scaling [7], sample freq [6:3], DARS [1:0] ----- */
    bool    fs_scaled = (cs[4] & 0x80) != 0;
    uint8_t fs4       = (cs[4] >> 3) & 0x0F;
    uint8_t dars      =  cs[4]       & 0x03;
    const char *fs4_str;
    switch (fs4)
    {
        case 0x0: fs4_str = "not indicated"; break;
        case 0x1: fs4_str = "24 kHz";        break;
        case 0x2: fs4_str = "96 kHz";        break;
        case 0x9: fs4_str = "22.05 kHz";     break;
        case 0xA: fs4_str = "88.2 kHz";      break;
        case 0xB: fs4_str = "176.4 kHz";     break;
        case 0x3: fs4_str = "192 kHz";       break;
        case 0xF: fs4_str = "user defined";  break;
        default:  fs4_str = "unknown";       break;
    }
    const char *dars_str;
    switch (dars)
    {
        case 0x0: dars_str = "not a DARS";        break;
        case 0x1: dars_str = "DARS grade2 10ppm"; break;
        case 0x2: dars_str = "DARS grade1 1ppm";  break;
        default:  dars_str = "reserved";          break;
    }

    /* Bytes 6-9 origin label, 10-13 destination label ------------------ */
    char origin[5], dest[5];
    ES9038Q2M_FormatAsciiLabel(&cs[6],  origin);
    ES9038Q2M_FormatAsciiLabel(&cs[10], dest);

    /* Bytes 14-17 local sample address, 18-21 time-of-day (LSB first) --- */
    uint32_t sample_addr = (uint32_t)cs[14]        | ((uint32_t)cs[15] << 8) |
                            ((uint32_t)cs[16] << 16)| ((uint32_t)cs[17] << 24);
    uint32_t tod         = (uint32_t)cs[18]        | ((uint32_t)cs[19] << 8) |
                            ((uint32_t)cs[20] << 16)| ((uint32_t)cs[21] << 24);

    /* Byte 22 reliability, byte 23 CRCC -------------------------------- */
    uint8_t reliability = cs[22];
    uint8_t crcc        = cs[23];

    /* --- log ---------------------------------------------------------- */
    LOG_INFO("  SPDIF: PROFESSIONAL, %s, %s, emphasis=%s",
              non_audio ? "non-audio" : "audio",
              unlocked  ? "UNLOCKED"  : "locked",
              emph_str);
    LOG_INFO("  SPDIF: Fs(byte0)=%s, user-bits=%s, mode=%s",
              fs0_str, user_str, mode_str);
    LOG_INFO("  SPDIF: chan-id: %s", chan_id_str);
    LOG_INFO("  SPDIF: align=%s, aux=%s", align_str, aux_str);
    if (word_bits)
        LOG_INFO("  SPDIF: word length=%u bits (max %s)",
                  word_bits, max24 ? "24" : "20");
    else
        LOG_INFO("  SPDIF: word length not indicated (max %s)",
                  max24 ? "24" : "20");
    LOG_INFO("  SPDIF: Fs(byte4)=%s%s, %s",
              fs4_str, fs_scaled ? " x(1/1.001)" : "", dars_str);
    LOG_INFO("  SPDIF: origin='%s' dest='%s'", origin, dest);
    LOG_INFO("  SPDIF: sample_addr=%lu, time_of_day=%lu",
              (unsigned long)sample_addr, (unsigned long)tod);
    LOG_INFO("  SPDIF: reliability=0x%02X, CRCC=0x%02X%s",
              reliability, crcc, (crcc == 0) ? " (not implemented)" : "");
}

/* Read REG70..REG93 (SPDIF channel status / user status) and log them.
  * The ES9038Q2M auto-increments the register pointer on multi-byte reads,
  * so all 24 bytes are fetched in one I2C transaction.
  * Values are cached and only logged when they change, to avoid log spam. */
void ES9038Q2M_LogSpdifChannelStatus(void)
{
    static uint8_t cached[ES9038Q2M_SPDIF_STATUS_COUNT] = {0};
    uint8_t        buf[ES9038Q2M_SPDIF_STATUS_COUNT];

    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&DAC_I2C_Handle, ES9038Q2M_I2C_DEV_ADDR,
                                            ES9038Q2M_REG70_ADDR, I2C_MEMADD_SIZE_8BIT,
                                            buf, ES9038Q2M_SPDIF_STATUS_COUNT,
                                            TIMEOUT_I2C_DELAY);
    if (st != HAL_OK)
    {
        LOG_WARN("SPDIF channel status read failed (%d)", (int)st);
        return;
    }

    /* SPDIF presence: REG72/73/74 (channel-status payload) all zero means no
     * real SPDIF stream is connected, even if the valid bit briefly flickers. */
    bool present = !(buf[2] == 0 && buf[3] == 0 && buf[4] == 0);
    if (present != es9038q2m_spdif_present)
    {
        LOG_INFO("SPDIF stream %s on %s input (REG72-74: %02X %02X %02X)",
                    present ? "PRESENT" : "absent",
                    Spdif_GetInputTypeStr(),
                    buf[2], buf[3], buf[4]);
    }
    es9038q2m_spdif_present = present;

    /* Only print when something actually changed */
    if (memcmp(cached, buf, sizeof(buf)) == 0)
        return;

    memcpy(cached, buf, sizeof(buf));

    LOG_INFO("SPDIF Channel/User Status (REG70-93):");
    for (uint8_t i = 0; i < ES9038Q2M_SPDIF_STATUS_COUNT; i += 4)
    {
        LOG_INFO("  REG%02u-%02u: %02X %02X %02X %02X",
                  (unsigned)(ES9038Q2M_REG70_ADDR + i),
                  (unsigned)(ES9038Q2M_REG70_ADDR + i + 3),
                  buf[i], buf[i + 1], buf[i + 2], buf[i + 3]);
    }

    ES9038Q2M_DecodeSpdifChannelStatus(buf);
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
    static uint8_t status_register = 0;

    cnt++;

    ES9038Q2M_UpdateLeds();

    if (cnt % 20 == 0)
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
                LOG_DBG("Status change: 0x%02X -> 0x%02X", status_register, new_status_register);
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
                    LOG_WARN("  DSD decoder %s, or fallback", (new_status_register & ES9038Q2M_STAT_DSD_VALID) ? "VALID" : "INVALID");
                }
                status_register = new_status_register;
            }

            /* When a valid SPDIF stream is present, dump its channel/user
              * status registers (only logs on change). */
            if (new_status_register & ES9038Q2M_STAT_SPDIF_VALID)
            {
                ES9038Q2M_LogSpdifChannelStatus();
            }
            else if (es9038q2m_spdif_present)
            {
                LOG_INFO("SPDIF stream absent on %s input (valid bit cleared)",
                         Spdif_GetInputTypeStr());
                es9038q2m_spdif_present = false;
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
        uint8_t reg_val_ch1;
        uint8_t reg_val_ch2;
        es9038q2m_configured_volume_ch1 = requested_volume_ch1;
        reg_val_ch1 = convert_vol_to_register(es9038q2m_configured_volume_ch1);
        I2C_Status = HAL_I2C_Mem_Write(&DAC_I2C_Handle, ES9038Q2M_I2C_DEV_ADDR,
                                        ES9038Q2M_REG15_ADDR, I2C_MEMADD_SIZE_8BIT,
                                        &reg_val_ch1, 1, TIMEOUT_I2C_DELAY);

        es9038q2m_configured_volume_ch2 = requested_volume_ch2;
        reg_val_ch2 = convert_vol_to_register(es9038q2m_configured_volume_ch2);
        I2C_Status = HAL_I2C_Mem_Write(&DAC_I2C_Handle, ES9038Q2M_I2C_DEV_ADDR,
                                        ES9038Q2M_REG16_ADDR, I2C_MEMADD_SIZE_8BIT,
                                        &reg_val_ch2, 1, TIMEOUT_I2C_DELAY);

        LOG_INFO("Applying volume change: %d/%d dB",
          -reg_val_ch1 / 2,
          -reg_val_ch2 / 2);
        LOG_DBG("reg_val_ch1=%u, reg_val_ch2=%u", reg_val_ch1, reg_val_ch2);

        /* Keep the analog LINE volume (PGA2311) in sync with USB/encoder volume.
         * Harmless for digital sources (PGA path is muted/relay open then). */
        PGA2311_SetVolume(es9038q2m_configured_volume_ch1,
                          es9038q2m_configured_volume_ch2);
    }
}

uint8_t ES9038Q2M_DAC_SetInput(ES9038Q2M_Input_t input)
{
    HAL_StatusTypeDef st;

    /* GPIO1/SPDIF routing (REG8, REG11, REG21) is configured once in
     * ES9038Q2M_DAC_Init. Write REG1 directly from config + defaults:
     *   serial_length = 32-bit, serial_mode = I2S, input_select = SPDIF
     *   (input_select only applies when auto_select is disabled).
     * auto_select:
     *   - I2S/USB : DSD/serial (DAC auto-picks DSD vs serial)
     *   - SPDIF   : disabled -> input_select (SPDIF) takes effect */
    uint8_t reg1 = REG1_SERIAL_LENGTH_32BIT
                 | REG1_SERIAL_MODE_I2S
                 | REG1_INPUT_SELECT_SPDIF
                 | ((input == ES9038Q2M_INPUT_SPDIF) ? REG1_AUTO_SELECT_DISABLE
                                                     : REG1_AUTO_SELECT_DSD);

    st = HAL_I2C_Mem_Write(&DAC_I2C_Handle, ES9038Q2M_I2C_DEV_ADDR,
                           ES9038Q2M_REG1_ADDR, I2C_MEMADD_SIZE_8BIT,
                           &reg1, 1, TIMEOUT_I2C_DELAY);
    if (st != HAL_OK) return 1;

    LOG_INFO("DAC input -> %s (REG1=0x%02X)",
             (input == ES9038Q2M_INPUT_SPDIF) ? "SPDIF (GPIO1, forced)"
                                              : "I2S (auto DSD/serial)",
             reg1);
    return 0;
}

bool ES9038Q2M_SpdifPresent(void)
{
    return es9038q2m_spdif_present;
}