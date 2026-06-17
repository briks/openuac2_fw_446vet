#include "pga2311.h"
#include "main.h"
#define LOG_LEVEL LOG_LEVEL_DBG
#include "log.h"

#define PGA2311_SPI_TIMEOUT  10  /* ms */

extern SPI_HandleTypeDef hspi4;

/* Convert host Q8.8 dB volume to a PGA2311 8-bit code.
 *   gain_dB = 0.5 * N - 95.5   ->   N = 2 * gain_dB + 192
 *   volume_q88 is Q8.8 dB      ->   N = volume_q88 / 128 + 192
 * N = 0 mutes; we clamp to the valid 0..255 range. */
uint8_t PGA2311_VolToReg(int16_t volume_q88)
{
    int32_t n = ((int32_t)volume_q88) / 128 + 192;
    if (n < 0)   n = 0;     /* mute */
    if (n > 255) n = 255;   /* +31.5 dB */
    return (uint8_t)n;
}

/* Send the 16-bit word: first byte = right channel, second byte = left. */
void PGA2311_SetGainRaw(uint8_t right, uint8_t left)
{
    uint8_t buf[2] = { right, left };

    LL_GPIO_ResetOutputPin(SPI4_CS_GPIO_Port, SPI4_CS_Pin);   /* CS low  */
    HAL_SPI_Transmit(&hspi4, buf, sizeof(buf), PGA2311_SPI_TIMEOUT);
    LL_GPIO_SetOutputPin(SPI4_CS_GPIO_Port, SPI4_CS_Pin);     /* CS high latches */
}

void PGA2311_SetVolume(int16_t vol_ch1_q88, int16_t vol_ch2_q88)
{
    uint8_t left  = PGA2311_VolToReg(vol_ch1_q88);   /* channel 1 = left  */
    uint8_t right = PGA2311_VolToReg(vol_ch2_q88);   /* channel 2 = right */
    LOG_DBG("PGA2311 set L=%u R=%u (q88 L=%d R=%d)",
            left, right, vol_ch1_q88, vol_ch2_q88);
    PGA2311_SetGainRaw(right, left);
}

/* Hardware mute via the PGA_M pin (active-low: low = muted, high = unmuted).
 * Independent of the SPI gain registers, so volume can stay in sync while muted. */
void PGA2311_Mute(bool mute)
{
    LOG_DBG("PGA2311 %s", mute ? "MUTE" : "UNMUTE");
    if (mute)
        LL_GPIO_ResetOutputPin(PGA_M_GPIO_Port, PGA_M_Pin);   /* low = muted   */
    else
        LL_GPIO_SetOutputPin(PGA_M_GPIO_Port, PGA_M_Pin);     /* high = unmuted */
}

void PGA2311_Init(void)
{
    /* CS idle high. Analog output is muted by the PGA_M pin (low at GPIO init),
     * so we just preload the SPI gain registers; PGA_M gates audibility. */
    LL_GPIO_SetOutputPin(SPI4_CS_GPIO_Port, SPI4_CS_Pin);
    PGA2311_Mute(true);
    LOG_INFO("PGA2311 init (muted)");
}