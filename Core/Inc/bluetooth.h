#ifndef _BLUETOOTH_H_
#define _BLUETOOTH_H_

#include <stdbool.h>
#include <stdint.h>

/* FSC-BT1026x driver
 * ------------------
 *   SYS_CTRL (power) : BT_PWR_Pin  (PB4), active HIGH, keep high while ON (>=20ms to start)
 *   RESET            : BT_RST_Pin  (PD7), active LOW
 *   Control/eventos  : USART2 (huart2), 115200 8N1, HW flow control RTS/CTS
 *   Audio            : I2S routed through the analog/I2S mux (MUX_SEL)
 *
 * NOTE: BT_Init()/BT_PowerOn() use osDelay(): call them from a thread, not
 *       before the scheduler is started.
 */

void BT_Init(void);          /* power on + start UART reception */
void BT_QueryInfo(void);     /* send AT+VER/NAME/LENAME (call after scheduler start) */
void BT_PowerOn(void);
void BT_PowerOff(void);
bool BT_IsPoweredOn(void);
bool BT_IsConnected(void);    /* used by the source-selection logic */
void BT_Process(void);        /* call periodically (drains UART, parses events) */
void BT_SendCommand(const char *cmd); /* sends "<cmd>\r\n" (AT commands) */
void BT_Pause(void);                  /* AVRCP: pause playback on the phone */
void BT_Play(void);                   /* AVRCP: resume playback on the phone */

#endif /* _BLUETOOTH_H_ */