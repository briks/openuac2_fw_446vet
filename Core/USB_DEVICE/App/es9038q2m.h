#ifndef _ES9038Q2M_H_
#define _ES9038Q2M_H_

#include <stdint.h>
#include <stdbool.h>

/* ES9038Q2M DAC driver
	* --------------------
	* I²C-controlled 32-bit stereo DAC. Datasheet register set used here:
	*   REG7  : filter bandwidth / system mute (bit 0 = mute)
	*   REG14 : general configuration (oscillator drop)
	*   REG15 : channel 1 volume, -0.5 dB per step (0=0 dB, 255=-127.5 dB)
	*   REG16 : channel 2 volume (linked to ch1 via REG27)
	*   REG27 : ASRC enable, ch2-follows-ch1, latch volume, optional +18 dB
	*   REG96 : input status (DoP / SPDIF / I²S / DSD valid flags)
	*/

#define ES9038Q2M_I2C_HANDLE       hi2c1
#define ES9038Q2M_I2C_DEV_ADDR     (0x48 << 1)

#define ES9038Q2M_REG7_ADDR        7
#define ES9038Q2M_REG14_ADDR       14
#define ES9038Q2M_REG15_ADDR       15
#define ES9038Q2M_REG16_ADDR       16
#define ES9038Q2M_REG27_ADDR       27
#define ES9038Q2M_REG96_ADDR       96

/* REG96 status bits */
#define ES9038Q2M_STAT_DOP_VALID    (1U << 3)
#define ES9038Q2M_STAT_SPDIF_VALID  (1U << 2)
#define ES9038Q2M_STAT_I2S_VALID    (1U << 1)
#define ES9038Q2M_STAT_DSD_VALID    (1U << 0)

extern volatile int16_t es9038q2m_configured_volume;
extern volatile bool es9038q2m_configured_mute;
extern volatile bool es9038q2m_audio_stop_pending;
extern const AUDIO_CodecTypeDef es9038q2m_instance;

uint8_t ES9038Q2M_DAC_Init(void);
uint8_t ES9038Q2M_DAC_SetVolume(int16_t vol);
uint8_t ES9038Q2M_DAC_SetMute(uint8_t mute);
void    ES9038Q2M_DAC_SetMute_Force(void);
uint8_t ES9038Q2M_DAC_SetFormat(uint8_t format);
uint8_t ES9038Q2M_DAC_Play(void);
uint8_t ES9038Q2M_DAC_Stop(void);
void    ES9038Q2M_ProcessEvents(void);

#endif /* _ES9038Q2M_H_ */
