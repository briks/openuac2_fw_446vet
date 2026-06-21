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

#define ES9038Q2M_REG1_ADDR   1  /* Input selection (serial_length/mode, auto_select, input_select) */
#define ES9038Q2M_REG6_ADDR   6  /* De-emphasis, DOP and volume ramp rate */
#define ES9038Q2M_REG7_ADDR   7  /* Filter bandwidth / system mute */
#define ES9038Q2M_REG8_ADDR   8  /* GPIO1-2 configuration */
#define ES9038Q2M_REG11_ADDR 11  /* SPDIF source select (spdif_sel [7:4]) */
#define ES9038Q2M_REG12_ADDR 12  /* DPLL bandwidth: [7:4]=serial, [3:0]=DSD */
#define ES9038Q2M_REG14_ADDR 14
#define ES9038Q2M_REG15_ADDR 15  /* Ch1 volume */
#define ES9038Q2M_REG16_ADDR 16  /* Ch2 volume */
#define ES9038Q2M_REG21_ADDR 21  /* GPIO input selection (gpio_sel1/2) */
#define ES9038Q2M_REG27_ADDR 27  /* General configuration*/
#define ES9038Q2M_REG70_ADDR 70 /* RO : SPDIF channel status / user status (start) */
#define ES9038Q2M_REG93_ADDR 93 /* RO : SPDIF channel status / user status (end)   */
#define ES9038Q2M_SPDIF_STATUS_COUNT (ES9038Q2M_REG93_ADDR - ES9038Q2M_REG70_ADDR + 1) /* 24 bytes */
#define ES9038Q2M_REG96_ADDR 96 /* RO : Input selection and automute status */
#define ES9038Q2M_REG100_ADDR 100 /* RO : LSB ADC readback (latch) */
#define ES9038Q2M_REG101_ADDR 101 /* RO :     ADC readback */
#define ES9038Q2M_REG102_ADDR 102 /* RO : MSB ADC readback */

/* ---- REG1: Input selection ----------------------------------------------
 * NOTE: auto_select [3:2] must be 2'b00 for input_select [1:0] to function. */
#define REG1_SERIAL_LENGTH_MASK   (0x3U << 6)
#define REG1_SERIAL_LENGTH_16BIT  (0x0U << 6)
#define REG1_SERIAL_LENGTH_24BIT  (0x1U << 6)
#define REG1_SERIAL_LENGTH_32BIT  (0x3U << 6)   /* default */

#define REG1_SERIAL_MODE_MASK     (0x3U << 4)
#define REG1_SERIAL_MODE_I2S      (0x0U << 4)   /* default */
#define REG1_SERIAL_MODE_LJ       (0x1U << 4)
#define REG1_SERIAL_MODE_RJ       (0x3U << 4)

#define REG1_AUTO_SELECT_MASK     (0x3U << 2)
#define REG1_AUTO_SELECT_DISABLE  (0x0U << 2)   /* required for manual input_select */
#define REG1_AUTO_SELECT_DSD      (0x1U << 2)
#define REG1_AUTO_SELECT_SPDIF    (0x2U << 2)
#define REG1_AUTO_SELECT_ALL      (0x3U << 2)   /* chip default */

#define REG1_INPUT_SELECT_MASK    (0x3U << 0)
#define REG1_INPUT_SELECT_SERIAL  (0x0U << 0)   /* I2S / LJ / RJ (default) */
#define REG1_INPUT_SELECT_SPDIF   (0x1U << 0)
#define REG1_INPUT_SELECT_DSD     (0x3U << 0)

/* ---- REG8: GPIO configuration (gpio1_cfg = low nibble, gpio2_cfg = high) --
 * Per-GPIO config codes (gpioX_cfg):
 *   4'd8  : Standard Input (high-Z, read back via REG65 / used by SPDIF decoder)
 *   4'd13 : Analog Input Shutdown (RESET DEFAULT for both GPIOs) */
#define REG8_GPIO1_CFG_MASK         0x0FU
#define REG8_GPIO1_CFG_STD_INPUT    0x08U          /* 4'd8  */
#define REG8_GPIO1_CFG_ANA_SHUTDOWN 0x0DU          /* 4'd13 (default) */
#define REG8_GPIO2_CFG_MASK         0xF0U
#define REG8_GPIO2_CFG_STD_INPUT    (0x08U << 4)
#define REG8_GPIO2_CFG_ANA_SHUTDOWN (0x0DU << 4)   /* 4'd13 (default) */

/* ---- REG11: SPDIF source select (spdif_sel [7:4], [3:0] reserved) --------
 *   4'd0: DATA_CLK (default)  4'd1: DATA1  4'd2: DATA2
 *   4'd3: GPIO1               4'd4: GPIO2 */
#define REG11_SPDIF_SEL_MASK      0xF0U
#define REG11_SPDIF_SEL_DATA_CLK  (0x0U << 4)
#define REG11_SPDIF_SEL_DATA1     (0x1U << 4)
#define REG11_SPDIF_SEL_DATA2     (0x2U << 4)
#define REG11_SPDIF_SEL_GPIO1     (0x3U << 4)
#define REG11_SPDIF_SEL_GPIO2     (0x4U << 4)

/* ---- REG21: GPIO Input Selection -----------------------------------------
 * Selects the input *type* assigned to a GPIO when that GPIO acts as an input.
 *   gpio_sel2 [7:6], gpio_sel1 [5:4], [3:0] reserved
 *     2'd0: serial (I2S/LJ) (default)  2'd1: SPDIF
 *     2'd2: reserved                   2'd3: DSD */
#define REG21_GPIO_SEL2_MASK      (0x3U << 6)
#define REG21_GPIO_SEL2_SERIAL    (0x0U << 6)
#define REG21_GPIO_SEL2_SPDIF     (0x1U << 6)
#define REG21_GPIO_SEL2_DSD       (0x3U << 6)

#define REG21_GPIO_SEL1_MASK      (0x3U << 4)
#define REG21_GPIO_SEL1_SERIAL    (0x0U << 4)   /* default */
#define REG21_GPIO_SEL1_SPDIF     (0x1U << 4)
#define REG21_GPIO_SEL1_DSD       (0x3U << 4)

typedef enum
{
    ES9038Q2M_INPUT_I2S = 0,
    ES9038Q2M_INPUT_SPDIF,
} ES9038Q2M_Input_t;

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
extern volatile bool es9038q2m_spdif_present;   /* true when a real SPDIF stream is detected */
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
bool    ES9038Q2M_SpdifPresent(void);
void    ES9038Q2M_ProcessEvents(void);
uint8_t ES9038Q2M_DAC_SetInput(ES9038Q2M_Input_t input);

#endif /* _ES9038Q2M_H_ */
