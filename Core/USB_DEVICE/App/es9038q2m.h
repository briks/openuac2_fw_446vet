#ifndef _ES9038Q2M_H_
#define _ES9038Q2M_H_

#include <stdint.h>
#include <stdbool.h>
#include "usbd_audio_if.h"
#include "main.h"

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

#define ES9038Q2M_I2C_DEV_ADDR     (0x48 << 1)

#define ES9038Q2M_REG6_ADDR   6 /* De amphasis, DOP and volume ramp rate */
#define ES9038Q2M_REG7_ADDR   7
#define ES9038Q2M_REG14_ADDR 14
#define ES9038Q2M_REG15_ADDR 15 /* Ch1 volume */
#define ES9038Q2M_REG16_ADDR 16 /* Ch2 volume */
#define ES9038Q2M_REG27_ADDR 27 /* General configuration*/
#define ES9038Q2M_REG70_ADDR 70 /* RO : SPDIF channel status / user status (start) */
#define ES9038Q2M_REG93_ADDR 93 /* RO : SPDIF channel status / user status (end)   */
#define ES9038Q2M_SPDIF_STATUS_COUNT (ES9038Q2M_REG93_ADDR - ES9038Q2M_REG70_ADDR + 1) /* 24 bytes */
#define ES9038Q2M_REG96_ADDR 96 /* RO : Input selection and automute status */
#define ES9038Q2M_REG100_ADDR 100 /* RO : LSB ADC readback (latch) */
#define ES9038Q2M_REG101_ADDR 101 /* RO :     ADC readback */
#define ES9038Q2M_REG102_ADDR 102 /* RO : MSB ADC readback */

/* REG96 status bits 
Bit Mnemonic Description
[7:6] reserved
[5:4] reserved
[3]dop_valid
    Contains the status of the DoP decoder.
    1'b0: The DoP decoder has not detected a valid DoP signal.
    1'b1: The DoP decoder has detected a valid DoP signal on the 12S input.
[2]
    spdif_valid
    Contains the status of the SPDIF decoder.
    1'b0: The SPDIF decoder has not found a valid SPDIF signal.
[1]
    i2s_select
    1'b1: The SPDIF decoder has detected a valid SPDIF signal.
    Contains the status of the I2S decoder.
    1'b0: The I2S decoder has not found a valid frame clock or bit clock.
    1'b1: The I2S decoder has detected a valid frame clock and bit clock arrangement.
[0]
    dsd_select
    Contains the status of the DSD decoder.
    1'b0: The DSD decoder is not being used.
    1'b1: The DSD decoder is being used as a fallback option if I2S has failed to decode their respective input signals. */
#define ES9038Q2M_STAT_DOP_VALID    (1U << 3)
#define ES9038Q2M_STAT_SPDIF_VALID  (1U << 2)
#define ES9038Q2M_STAT_I2S_VALID    (1U << 1)
#define ES9038Q2M_STAT_DSD_VALID    (1U << 0)

extern volatile int16_t requested_volume_ch1;
extern volatile int16_t requested_volume_ch2;
extern volatile bool requested_mute;
extern volatile bool es9038q2m_audio_stop_pending;
extern const AUDIO_CodecTypeDef es9038q2m_instance;


uint8_t ES9038Q2M_DAC_Init(void);
uint8_t ES9038Q2M_DAC_DeInit(void);
/* Request a volume change by a relative amount, as detected by the encoder */
void ES9038Q2M_DAC_Volume_change(int8_t delta);
uint8_t ES9038Q2M_DAC_Volume_set(int16_t vol, uint8_t channel);
uint8_t ES9038Q2M_DAC_Mute_set(bool mute);
void ES9038Q2M_DAC_SetMute_Force(bool mute);
uint8_t ES9038Q2M_DAC_Format_set(uint8_t format);
uint8_t ES9038Q2M_DAC_Play(void);
uint8_t ES9038Q2M_DAC_Stop(void);
void    ES9038Q2M_LogSpdifChannelStatus(void);
void    ES9038Q2M_ProcessEvents(void);

#endif /* _ES9038Q2M_H_ */
