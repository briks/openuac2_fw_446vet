#include <string.h>

#include "bluetooth.h"
#include "main.h"
#include "cmsis_os.h"
#define LOG_LEVEL LOG_LEVEL_DBG
#include "log.h"

/* huart2 is a file-scope global defined in main.c */
extern UART_HandleTypeDef huart2;

/* ---- RX ring buffer (filled in ISR, drained in BT_Process) --------------- */
#define BT_RX_RING_SIZE 256
static volatile uint8_t  bt_rx_ring[BT_RX_RING_SIZE];
static volatile uint16_t bt_rx_head;     /* written by ISR  */
static uint16_t          bt_rx_tail;     /* read by task    */
static uint8_t           bt_rx_byte;     /* HAL IT scratch  */

/* ---- line assembly ------------------------------------------------------- */
#define BT_LINE_MAX 128
static char     bt_line[BT_LINE_MAX];
static uint16_t bt_line_len;

/* ---- state --------------------------------------------------------------- */
static volatile bool bt_powered   = false;
static volatile bool bt_connected = false;

/* -------------------------------------------------------------------------- */

bool BT_IsPoweredOn(void) { return bt_powered; }
bool BT_IsConnected(void) { return bt_connected; }

void BT_PowerOn(void)
{
    LOG_INFO("BT power ON sequence (SYS_CTRL=PB4, RST=PD7)");

    /* hold module in reset while power rail comes up */
    LL_GPIO_ResetOutputPin(BT_RST_GPIO_Port, BT_RST_Pin);   /* reset asserted (active low) */
    LL_GPIO_ResetOutputPin(BT_PWR_GPIO_Port, BT_PWR_Pin);   /* SYS_CTRL low                */

    /* wait for the supply to be stable before driving SYS_CTRL */
    osDelay(100);
    LOG_DBG("BT 100ms power settle done, asserting SYS_CTRL");

    LL_GPIO_SetOutputPin(BT_PWR_GPIO_Port, BT_PWR_Pin);     /* SYS_CTRL high -> power on    */
    osDelay(30);                                            /* >= 20 ms hold               */

    LL_GPIO_SetOutputPin(BT_RST_GPIO_Port, BT_RST_Pin);     /* release reset               */
    LOG_DBG("BT reset released, waiting for module boot");
    osDelay(500);                                           /* let the module boot         */

    bt_powered = true;
    LOG_INFO("BT powered on");
}

void BT_PowerOff(void)
{
    LOG_INFO("BT power OFF");
    LL_GPIO_ResetOutputPin(BT_RST_GPIO_Port, BT_RST_Pin);   /* hold in reset */
    LL_GPIO_ResetOutputPin(BT_PWR_GPIO_Port, BT_PWR_Pin);   /* SYS_CTRL low -> power off */
    bt_powered   = false;
    bt_connected = false;
}

void BT_Init(void)
{
    LOG_INFO("BT_Init...");
    bt_rx_head = bt_rx_tail = 0;
    bt_line_len = 0;

    BT_PowerOn();

    /* start interrupt-driven, byte-by-byte reception */
    HAL_StatusTypeDef st = HAL_UART_Receive_IT(&huart2, &bt_rx_byte, 1);
    if (st != HAL_OK)
    {
        LOG_ERR("BT UART RX start failed (%d)", (int)st);
    }
    else
    {
        LOG_DBG("BT UART RX armed");
    }

    /* query module identity / firmware (replies printed via the RX parser) */
    BT_SendCommand("AT+VER");
    osDelay(50);
    BT_SendCommand("AT+NAME");
    osDelay(50);
    BT_SendCommand("AT+LENAME");

    LOG_INFO("BT_Init done");
}

void BT_SendCommand(const char *cmd)
{
    if (!cmd)
    {
        LOG_ERR("BT_SendCommand: NULL cmd");
        return;
    }

    HAL_StatusTypeDef st;
    st = HAL_UART_Transmit(&huart2, (uint8_t *)cmd, (uint16_t)strlen(cmd), 100);
    if (st != HAL_OK)
    {
        LOG_ERR("BT TX failed (%d) for '%s'", (int)st, cmd);
        return;
    }
    st = HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, 100);
    if (st != HAL_OK)
    {
        LOG_ERR("BT TX (CRLF) failed (%d)", (int)st);
        return;
    }
    LOG_DBG("BT >> %s", cmd);
}

/* Parse one complete line coming from the module.
 * Replies are CR/LF framed, so a line looks like:
 *   "+VER=FSC-BT1026C,V4.9.4"
 *   "+NAME=FSC-BT1026C"
 *   "+LENAME=FSC-BT1026C-LE"
 *   "OK" / "ERROR"
 * plus unsolicited connection events.
 * Use prefix matching (strncmp); check DISCONNECT before CONNECT. */
static void BT_ParseLine(const char *line)
{
    if (line[0] == '\0')
        return;

    LOG_DBG("BT << %s", line);

    /* ---- AT query replies ---------------------------------------------- */
    if (strncmp(line, "+VER=", 5) == 0)
    {
        LOG_INFO("BT firmware version: %s", line + 5);
        return;
    }
    if (strncmp(line, "+LENAME=", 8) == 0)   /* test before +NAME= */
    {
        LOG_INFO("BT BLE name: %s", line + 8);
        return;
    }
    if (strncmp(line, "+NAME=", 6) == 0)
    {
        LOG_INFO("BT BR/EDR name: %s", line + 6);
        return;
    }

    /* ---- AT status replies --------------------------------------------- */
    if (strcmp(line, "OK") == 0)
    {
        LOG_DBG("BT command OK");
        return;
    }
    if (strstr(line, "ERROR") || strstr(line, "FAIL"))
    {
        LOG_ERR("BT module reported: %s", line);
        return;
    }

    /* ---- unsolicited connection events --------------------------------- */
    if (strstr(line, "DISCONNECT") || strstr(line, "CLOSE") || strstr(line, "CLOSED"))
    {
        if (bt_connected) LOG_INFO("BT disconnected");
        else              LOG_DBG("BT disconnect event (already disconnected)");
        bt_connected = false;
    }
    else if (strstr(line, "CONNECT") || strstr(line, "A2DP") || strstr(line, "OPEN"))
    {
        if (!bt_connected) LOG_INFO("BT connected");
        else               LOG_DBG("BT connect event (already connected)");
        bt_connected = true;
    }
    else
    {
        LOG_DBG("BT unhandled line: %s", line);
    }
}

void BT_Process(void)
{
    if (!bt_powered)
        return;

    /* drain the ring buffer, assembling CR/LF terminated lines */
    while (bt_rx_tail != bt_rx_head)
    {
        uint8_t c = bt_rx_ring[bt_rx_tail];
        bt_rx_tail = (uint16_t)((bt_rx_tail + 1) % BT_RX_RING_SIZE);

        if (c == '\r' || c == '\n')
        {
            if (bt_line_len > 0)
            {
                bt_line[bt_line_len] = '\0';
                BT_ParseLine(bt_line);
                bt_line_len = 0;
            }
        }
        else if (bt_line_len < (BT_LINE_MAX - 1))
        {
            bt_line[bt_line_len++] = (char)c;
        }
        else
        {
            /* line overflow: drop it */
            LOG_WARN("BT RX line overflow, dropping");
            bt_line_len = 0;
        }
    }
}

/* ---- HAL UART callbacks (USART2) ----------------------------------------- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        uint16_t next = (uint16_t)((bt_rx_head + 1) % BT_RX_RING_SIZE);
        if (next != bt_rx_tail)            /* drop byte if buffer full */
        {
            bt_rx_ring[bt_rx_head] = bt_rx_byte;
            bt_rx_head = next;
        }
        /* else: ring full, byte dropped (logged from task side on overflow) */

        HAL_UART_Receive_IT(huart, &bt_rx_byte, 1);  /* re-arm */
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        uint32_t err = HAL_UART_GetError(huart);
        LOG_ERR("BT UART error 0x%lX", (unsigned long)err);

        /* clear errors and re-arm reception */
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        HAL_UART_Receive_IT(huart, &bt_rx_byte, 1);
    }
}