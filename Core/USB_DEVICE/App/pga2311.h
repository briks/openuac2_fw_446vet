#ifndef _PGA2311_H_
#define _PGA2311_H_

#include <stdint.h>
#include <stdbool.h>

/* PGA2311 stereo volume control on SPI4 (CS = SPI4_CS_Pin).
 * Volume input is the same Q8.8 dB used by the USB Audio class so the
 * analog LINE path tracks the DAC/USB volume.
 * Audibility is gated by the PGA_M hardware mute pin (active-low). */

void    PGA2311_Init(void);
uint8_t PGA2311_VolToReg(int16_t volume_q88);
void    PGA2311_SetGainRaw(uint8_t right, uint8_t left);
void    PGA2311_SetVolume(int16_t vol_ch1_q88, int16_t vol_ch2_q88); /* ch1=L, ch2=R */
void    PGA2311_Mute(bool mute);   /* drives PGA_M hardware mute line */

#endif /* _PGA2311_H_ */