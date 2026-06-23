#include <string.h>
#include <stdlib.h>   /* atoi  */
#include <stdio.h>    /* sscanf */

#include "bluetooth.h"
#include "main.h"
#include "cmsis_os.h"
#define LOG_LEVEL LOG_LEVEL_DBG
#include "log.h"

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
static volatile bool bt_connected = false;  /* A2DP link up (state >= 3)     */
static volatile bool bt_streaming = false;  /* actively streaming (state == 4) */

static osMutexId bt_uart_mutex = NULL;
osMutexDef(bt_uart_mutex);

#define BT_BOOT_READY_MS 1060U
#define BT_TX_TIMEOUT_MS 10
// Max BT volume on the phone, DAC volume is increased above
#define BT_SPKVOL_MAX  15

static int8_t        bt_phone_vol    = -1;     /* tracked phone volume (-1 unknown) */
static volatile bool bt_spkvol_valid = false;  /* set by parser on +SPKVOL= reply   */

static uint32_t bt_poweron_tick;   /* HAL tick when SYS_CTRL went high (standby exit) */

/* Desired Bluetooth names (written to flash only if they differ) */
#define BT_NAME_BREDR   "BriXamp!"
#define BT_NAME_BLE     "LE-BriXamp!"

/* ---- AT+I2SCFG bit field (base-10 value, per FSC-BT AT spec) -------------
 *   BIT[0]   0:disable        1:enable
 *   BIT[1]   0:master         1:slave
 *   BIT[2]   0:FS=48000Hz     1:FS=44100Hz
 *   BIT[3]   0:left justified 1:right justified
 *   BIT[4]   0:1-bit delay    1:no delay
 *   BIT[5-6] 00:16bit  01:24bit  10:32bit                                   */
#define BT_I2SCFG_ENABLE        (1U << 0)
#define BT_I2SCFG_DISABLE       (0U << 0)
#define BT_I2SCFG_SLAVE         (1U << 1)
#define BT_I2SCFG_MASTER        (0U << 1)
#define BT_I2SCFG_FS_44100      (1U << 2)
#define BT_I2SCFG_FS_48000      (0U << 2)
#define BT_I2SCFG_RIGHT_JUST    (1U << 3)
#define BT_I2SCFG_LEFT_JUST     (0U << 3)
#define BT_I2SCFG_NO_DELAY      (1U << 4)
#define BT_I2SCFG_DELAY_1BIT    (0U << 4)
#define BT_I2SCFG_DEPTH_16BIT   (0U << 5)
#define BT_I2SCFG_DEPTH_24BIT   (1U << 5)
#define BT_I2SCFG_DEPTH_32BIT   (2U << 5)

/* aptX décode en 16 bits natifs. Essayer 16 bits pour éviter un padding
 * 16->32 incorrect côté module. (= enable|master|44.1k|LJ|1-bit|16bit = 5) */
// #define BT_I2SCFG_DESIRED   ( BT_I2SCFG_ENABLE      \
//                             | BT_I2SCFG_MASTER      \
//                             | BT_I2SCFG_FS_44100    \
//                             | BT_I2SCFG_LEFT_JUST   \
//                             | BT_I2SCFG_DELAY_1BIT  \
//                             | BT_I2SCFG_DEPTH_16BIT )

/* LDAC decodes up to 24-bit / 96 kHz, so the I2S port must carry 32-bit
 * samples (16-bit would truncate). Keep master + Philips framing.
 *   = enable | master | 44.1kHz | left-just | 1-bit delay | 32-bit = 69 */
// #define BT_I2SCFG_DESIRED   ( BT_I2SCFG_ENABLE      \
//                             | BT_I2SCFG_MASTER      \
//                             | BT_I2SCFG_FS_44100    \
//                             | BT_I2SCFG_LEFT_JUST   \
//                             | BT_I2SCFG_DELAY_1BIT  \
//                             | BT_I2SCFG_DEPTH_32BIT )

/* The module negotiates aptX HD = 48 kHz / 24-bit (not classic aptX). Output
 * the I2S port at 48 kHz / 32-bit so the module's internal SRC is bypassed
 * (no 48->44.1 conversion) and the 24-bit payload is carried intact; the DAC
 * ASRC does the single final conversion.
 *   = enable | master | 48kHz | left-just | 1-bit delay | 32-bit = 65 */
#define BT_I2SCFG_DESIRED   ( BT_I2SCFG_ENABLE      \
                            | BT_I2SCFG_MASTER      \
                            | BT_I2SCFG_FS_48000    \
                            | BT_I2SCFG_LEFT_JUST   \
                            | BT_I2SCFG_DELAY_1BIT  \
                            | BT_I2SCFG_DEPTH_32BIT )

/* ---- AT+A2DPCFG codec enable bit field (base-10 value, per AT spec) -------
 *   BIT[0] AAC      BIT[1] APTX     BIT[2] APTX-LL
 *   BIT[3] APTX-HD  BIT[4] APTX-AD  BIT[5] LDAC                              */
#define BT_A2DPCFG_AAC          (1U << 0)
#define BT_A2DPCFG_APTX         (1U << 1)
#define BT_A2DPCFG_APTX_LL      (1U << 2)
#define BT_A2DPCFG_APTX_HD      (1U << 3)
#define BT_A2DPCFG_APTX_AD      (1U << 4)
#define BT_A2DPCFG_LDAC         (1U << 5)

/* Desired A2DP codec set. LDAC is intentionally left OFF: it can negotiate
 * 88.2/96 kHz, but the module's I2S port is capped at 48 kHz (AT+I2SCFG), so
 * the module would downsample 96->48 with its mediocre internal SRC -> audible
 * treble crackle. aptX HD (48 kHz / 24-bit native) plays cleanly through the
 * 48 kHz / 32-bit I2S port with no internal resampling.
 *   = AAC | APTX | APTX-LL | APTX-HD | APTX-AD  (LDAC off) = 0x1F = 31 */
// Default value read=47: AAC=1 APTX=1 APTX-LL=1 APTX-HD=1 APTX-AD=0 LDAC=1
#define BT_A2DPCFG_DESIRED      ( BT_A2DPCFG_AAC      \
                                | BT_A2DPCFG_APTX     \
                                | BT_A2DPCFG_APTX_LL  \
                                | BT_A2DPCFG_APTX_HD    )

/* Desired auto get track info + auto get track play progress (in s, 0 to disable)*/
#define BT_AVRCPCFG_DESIRED (1 + (0 << 1))  /* get track ID3 + get track progress freq=0s (off) */

/* ---- AT+A2DPDEC / AT+A2DPENC codec id values (per AT spec) ---------------- */
#define BT_A2DP_CODEC_SBC       1
#define BT_A2DP_CODEC_AAC       3
#define BT_A2DP_CODEC_APTX      5
#define BT_A2DP_CODEC_APTX_HD   7
#define BT_A2DP_CODEC_APTX_LL   8
#define BT_A2DP_CODEC_APTX_AD   9
#define BT_A2DP_CODEC_LDAC      10

/* +A2DPDEC / +A2DPENC Param: active A2DP codec id */
static const char *BT_A2dpCodecStr(int c)
{
    switch (c)
    {
        case BT_A2DP_CODEC_SBC:     return "SBC";
        case BT_A2DP_CODEC_AAC:     return "AAC";
        case BT_A2DP_CODEC_APTX:    return "aptX";
        case BT_A2DP_CODEC_APTX_HD: return "aptX HD";
        case BT_A2DP_CODEC_APTX_LL: return "aptX LL";
        case BT_A2DP_CODEC_APTX_AD: return "aptX Adaptive";
        case BT_A2DP_CODEC_LDAC:    return "LDAC";
        default:                    return "unknown";
    }
}

/* -------------------------------------------------------------------------- */


static bool bt_paused_by_us = false;   /* we issued AT+PAUSE, eligible to resume */

/* Send an AVRCP "pause" to the connected phone (e.g. when the amp powers off,
 * so the phone stops playing into a dead output). No-op if not connected. */
void BT_Pause(void)
{
    if (!bt_powered || !bt_connected)
        return;
    LOG_INFO("BT pause");
    BT_SendCommand("AT+PAUSE");
    //bt_paused_by_us = true;
}

/* Send an AVRCP "play" to the connected phone (e.g. resume on amp power-on
 * when BT is the active source). No-op if not connected. */
void BT_Play(void)
{
    if (!bt_powered || !bt_connected)
        return;
    // if (!bt_paused_by_us)
    // {
    //     LOG_DBG("BT_Play skipped: not paused by us (avoid hijack)");
    //     return;
    // }
    LOG_INFO("BT play"); 
    BT_SendCommand("AT+PLAY"); // Should hijack audio from phone
    //bt_paused_by_us = false;
}

/* Set the phone's A2DP volume to an absolute value (AT+SPKVOL=N). */
void BT_VolumeInit(uint8_t target)
{
    if (!bt_powered || !bt_connected)
        return;
    if (target > BT_SPKVOL_MAX)
        target = BT_SPKVOL_MAX;

    char cmd[16];
    snprintf(cmd, sizeof(cmd), "AT+SPKVOL=%u", (unsigned)target);
    BT_SendCommand(cmd);
    bt_phone_vol = (int8_t)target;
    LOG_INFO("BT phone volume set to %u", (unsigned)target);
}

/* Query the phone's current volume and wait briefly for the +SPKVOL= reply,
 * which is parsed by BT_Process() in another thread. Keeps bt_phone_vol in
 * sync when the user changed volume directly on the phone. */
static void BT_VolumeResync(void)
{
    bt_spkvol_valid = false;
    BT_SendCommand("AT+SPKVOL");          /* get -> +SPKVOL=N */

    uint32_t start = HAL_GetTick();
    while (!bt_spkvol_valid && (HAL_GetTick() - start) < (BT_TX_TIMEOUT_MS * 2))
        osDelay(2);  /* yield so Events_Thread can parse */

    if (!bt_spkvol_valid)
        LOG_WARN("BT SPKVOL resync timeout, using cached %d", bt_phone_vol);
}

/* Apply a rotary delta to the phone volume.
 *  - up   : phone absorbs up to (15 - current); leftover UP steps are returned
 *           for the caller to apply to the DAC (raises the ceiling).
 *  - down : phone takes all steps (to 0); the DAC is never lowered -> returns 0.
 * Resyncs from the phone first, but only on a fresh interaction (idle gap), so
 * a continuous spin isn't slowed by a query on every step. */
int8_t BT_VolumeChange(int8_t delta)
{
    static uint32_t last_tick = 0;

    if (!bt_powered || !bt_connected)
        return (delta > 0) ? delta : 0;   /* phone gone: all up-steps -> DAC */

    uint32_t now = HAL_GetTick();
    if ((now - last_tick) > 1000)         /* new interaction -> trust the phone */
        BT_VolumeResync();
    last_tick = now;

    if (bt_phone_vol < 0) bt_phone_vol = BT_SPKVOL_START;

    if (delta > 0)
    {
        int room = BT_SPKVOL_MAX - bt_phone_vol;
        if (room < 0) room = 0;
        int take = (delta < room) ? delta : room;
        for (int i = 0; i < take; i++)
            BT_SendCommand("AT+SPKVOL=+");
        bt_phone_vol += take;
        LOG_INFO("BT vol up: phone=%d, %d step(s) left for DAC", bt_phone_vol, delta - take);
        return (int8_t)(delta - take);    /* leftover -> DAC up */
    }
    else if (delta < 0)
    {
        int down = -delta;
        int take = (down < bt_phone_vol) ? down : bt_phone_vol;
        for (int i = 0; i < take; i++)
            BT_SendCommand("AT+SPKVOL=-");
        bt_phone_vol -= take;
        LOG_INFO("BT vol down: phone=%d", bt_phone_vol);
        return 0;                         /* DAC never lowered */
    }
    return 0;
}

bool BT_IsPoweredOn(void) { return bt_powered; }
bool BT_IsConnected(void) { return bt_connected; }

void BT_PowerOn(void)
{
    LOG_DBG("BT power ON sequence");

    /* hold module in reset while power rail comes up */
    LL_GPIO_ResetOutputPin(BT_RST_GPIO_Port, BT_RST_Pin);   /* reset asserted (active low) */
    LL_GPIO_ResetOutputPin(BT_PWR_GPIO_Port, BT_PWR_Pin);   /* SYS_CTRL low                */
    HAL_Delay(1);
    LL_GPIO_SetOutputPin(BT_RST_GPIO_Port, BT_RST_Pin);     /* release reset               */
    LOG_DBG("BT reset released, waiting for module boot");

    /* wait for the supply to be stable before driving SYS_CTRL */
    HAL_Delay(100);
    LL_GPIO_SetOutputPin(BT_PWR_GPIO_Port, BT_PWR_Pin);     /* SYS_CTRL high -> power on    */
    bt_poweron_tick = HAL_GetTick();                        /* standby-exit reference time   */
    LOG_DBG("BT 100ms power settle done, asserting SYS_CTRL");

    LOG_DBG("BT powered on, wait for to be ready %u ms", BT_BOOT_READY_MS);
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

    if (bt_uart_mutex == NULL)
        bt_uart_mutex = osMutexCreate(osMutex(bt_uart_mutex));

    BT_PowerOn();

    /* RX is armed in BT_QueryInfo once the module's TX line is stable, to avoid
     * spurious framing/overrun errors from line noise during power-up. */
    LOG_DBG("BT_Init done");
}

/* Query module identity / firmware. Call this once the scheduler is running
 * and BT_Process() is being pumped (Events_Thread), so the replies are parsed
 * and logged right after each request. */
void BT_QueryInfo(void)
{
    uint32_t elapsed = HAL_GetTick() - bt_poweron_tick;   /* wrap-safe */
    if (elapsed < BT_BOOT_READY_MS)
    {
        uint32_t remaining = BT_BOOT_READY_MS - elapsed;
        LOG_DBG("BT not ready yet, waiting %lu ms before AT commands",
                (unsigned long)remaining);
        osDelay(remaining);
    }

    /* discard any noise accumulated on the line during module power-up */
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    __HAL_UART_CLEAR_NEFLAG(&huart2);
    __HAL_UART_CLEAR_FEFLAG(&huart2);
    __HAL_UART_CLEAR_PEFLAG(&huart2);

    /* start interrupt-driven, byte-by-byte reception (line is stable now) */
    HAL_StatusTypeDef st = HAL_UART_Receive_IT(&huart2, &bt_rx_byte, 1);
    if (st != HAL_OK)
        LOG_ERR("BT UART RX start failed (%d)", (int)st);
    else
        LOG_DBG("BT UART RX armed");

    bt_powered = true;   /* module ready: BT_Process() starts draining RX */
    osDelay(20); // get DEVSTAT
    LOG_INFO("BT query info (VER / NAME / LENAME / I2SCFG / A2DPCFG / A2DPDEC)");
    BT_SendCommand("AT+VER");
    osDelay(20);
    BT_SendCommand("AT+NAME"); /* -> +NAME=...  : checked/updated in parser */
    osDelay(20);
    BT_SendCommand("AT+LENAME"); /* -> +LENAME=...: checked/updated in parser */
    osDelay(20);
    BT_SendCommand("AT+I2SCFG");
    osDelay(20);
    BT_SendCommand("AT+A2DPCFG");   /* -> +A2DPCFG=...: enabled codec set */
    osDelay(20);
    BT_SendCommand("AT+A2DPDEC");   /* -> +A2DPDEC=...: active decoder      */
    osDelay(20);
    BT_SendCommand("AT+AVRCPCFG"); /* -> Get/Set AVRCP Configuration     */
}

void BT_SendCommand(const char *cmd)
{
    if (!cmd)
    {
        LOG_ERR("BT_SendCommand: NULL cmd");
        return;
    }

    if (bt_uart_mutex)
        osMutexWait(bt_uart_mutex, osWaitForever);

    HAL_StatusTypeDef st;
    st = HAL_UART_Transmit(&huart2, (uint8_t *)cmd, (uint16_t)strlen(cmd), BT_TX_TIMEOUT_MS);
    if (st != HAL_OK)
    {
        if (st == HAL_TIMEOUT)
            LOG_WARN("BT TX timeout for '%s'", cmd);
        else
            LOG_ERR("BT TX failed (%d) for '%s'", (int)st, cmd);
    }
    else
    {
        st = HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, BT_TX_TIMEOUT_MS);
        if (st != HAL_OK)
            LOG_WARN("BT TX (CRLF) err (%d)", (int)st);
        else
            LOG_DBG("BT >> %s", cmd);
    }

    if (bt_uart_mutex)
        osMutexRelease(bt_uart_mutex);
}

/* +PLAYSTAT / +TRACKSTAT Param1: media player state */
static const char *BT_PlayStateStr(int s)
{
    switch (s)
    {
        case 0: return "Stopped";
        case 1: return "Playing";
        case 2: return "Paused";
        case 3: return "Fast Forwarding";
        case 4: return "Fast Rewinding";
        default: return "unknown";
    }
}

/* +A2DPSTAT Param: A2DP link state */
static const char *BT_A2dpStateStr(int s)
{
    switch (s)
    {
        case 0: return "Unsupported";
        case 1: return "Standby";
        case 2: return "Connecting";
        case 3: return "Connected";
        case 4: return "Streaming";
        default: return "unknown";
    }
}

/* Replace bytes that RTT/terminals choke on (e.g. 0xFF = telnet IAC) or any
 * other non-printable byte with a visible "|" delimiter. Track metadata from
 * the module uses 0xFF as the field separator between title/artist/album. */
static const char *BT_Sanitize(const char *in)
{
    static char buf[BT_LINE_MAX * 3];   /* worst case: each byte -> " | " */
    size_t j = 0;
    for (size_t i = 0; in[i] != '\0' && j < sizeof(buf) - 4; i++)
    {
        unsigned char c = (unsigned char)in[i];
        if (c >= 0x20 && c < 0x7F)
        {
            buf[j++] = (char)c;
        }
        else
        {
            buf[j++] = ' ';
            buf[j++] = '|';
            buf[j++] = ' ';
        }
    }
    buf[j] = '\0';
    return buf;
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

    //LOG_DBG("BT << %s", BT_Sanitize(line));

    /* ---- AT query replies ---------------------------------------------- */
    if (strncmp(line, "+VER=", 5) == 0)
    {
        LOG_INFO("BT firmware version: %s", line + 5);
        return;
    }
    if (strncmp(line, "+LENAME=", 8) == 0)   /* test before +NAME= */
    {
        const char *name = line + 8;
        LOG_INFO("BT BLE name: %s", name);
        if (strcmp(name, BT_NAME_BLE) != 0)
        {
            LOG_WARN("BT BLE name differs from '%s', updating", BT_NAME_BLE);
            BT_SendCommand("AT+LENAME=" BT_NAME_BLE);
            /* uncomment if your firmware needs a reboot to apply:
            BT_SendCommand("AT+REBOOT"); */
        }
        return;
    }
    if (strncmp(line, "+NAME=", 6) == 0)
    {
        const char *name = line + 6;
        LOG_INFO("BT BR/EDR name: %s", name);
        if (strcmp(name, BT_NAME_BREDR) != 0)
        {
            LOG_WARN("BT BR/EDR name differs from '%s', updating", BT_NAME_BREDR);
            BT_SendCommand("AT+NAME=" BT_NAME_BREDR);
            /* uncomment if your firmware needs a reboot to apply:
            BT_SendCommand("AT+REBOOT"); */
        }
        return;
    }

    if (strncmp(line, "+SPKVOL=", 8) == 0)
    {
        int volume = atoi(line + 8);
        if (volume < 0) volume = 0;
        if (volume > BT_SPKVOL_MAX) volume = BT_SPKVOL_MAX;
        bt_phone_vol    = (int8_t)volume;
        bt_spkvol_valid = true;
        LOG_DBG("BT phone vol = %d", bt_phone_vol);
        return;
    }

    /* ---- AT status replies --------------------------------------------- */
        if (strncmp(line, "+I2SCFG=", 8) == 0)
    {
        int cfg = atoi(line + 8);

        bool enabled  = (cfg >> 0) & 0x1;
        bool slave    = (cfg >> 1) & 0x1;
        bool fs44k    = (cfg >> 2) & 0x1;
        bool rjust    = (cfg >> 3) & 0x1;
        bool nodelay  = (cfg >> 4) & 0x1;
        uint8_t depth = (cfg >> 5) & 0x3;   /* 00:16b 01:24b 10:32b */

        const char *depth_str = (depth == 0) ? "16-bit"
                              : (depth == 1) ? "24-bit"
                              : (depth == 2) ? "32-bit"
                                             : "reserved";

        LOG_INFO("BT I2SCFG=%d: %s, %s, Fs=%s, %s, %s, %s",
                 cfg,
                 enabled ? "enabled" : "disabled",
                 slave   ? "slave"   : "master",
                 fs44k   ? "44.1kHz" : "48kHz",
                 rjust   ? "right-justified" : "left-justified",
                 nodelay ? "no bit delay"    : "1-bit delay",
                 depth_str);

        if (cfg != BT_I2SCFG_DESIRED)
        {
            char cmd[24];
            LOG_WARN("BT I2SCFG differs from desired %d, updating", BT_I2SCFG_DESIRED);
            snprintf(cmd, sizeof(cmd), "AT+I2SCFG=%d", BT_I2SCFG_DESIRED);
            BT_SendCommand(cmd);
            /* uncomment if your firmware needs a reboot to apply:
            BT_SendCommand("AT+REBOOT"); */
        }
        return;
    }

    if (strncmp(line, "+SPDIFCFG=", 10) == 0)
    {
        int cfg = atoi(line + 10);
        LOG_INFO("BT SPDIFCFG=%d: SPDIF audio output %s",
                 cfg, cfg ? "enabled" : "disabled");
        return;
    }

    if (strncmp(line, "+A2DPCFG=", 9) == 0)
    {
        int cfg = atoi(line + 9);
        LOG_INFO("BT A2DPCFG=%d: AAC=%d APTX=%d APTX-LL=%d APTX-HD=%d APTX-AD=%d LDAC=%d",
                 cfg,
                 (cfg & BT_A2DPCFG_AAC) ? 1 : 0,
                 (cfg & BT_A2DPCFG_APTX) ? 1 : 0,
                 (cfg & BT_A2DPCFG_APTX_LL) ? 1 : 0,
                 (cfg & BT_A2DPCFG_APTX_HD) ? 1 : 0,
                 (cfg & BT_A2DPCFG_APTX_AD) ? 1 : 0,
                 (cfg & BT_A2DPCFG_LDAC) ? 1 : 0);

        /* To enforce a desired codec set (e.g. disable LDAC to avoid the
         * 96kHz->48kHz internal-SRC crackle), uncomment below. A2DPCFG is
         * persistent and usually needs a reboot to take effect. */
        if (cfg != BT_A2DPCFG_DESIRED)
        {
            char cmd[24];
            LOG_WARN("BT A2DPCFG differs from desired %d, updating", BT_A2DPCFG_DESIRED);
            snprintf(cmd, sizeof(cmd), "AT+A2DPCFG=%d", BT_A2DPCFG_DESIRED);
            BT_SendCommand(cmd);
        }
        return;
    }

    if (strncmp(line, "+A2DPDEC=", 9) == 0)
    {
        int codec = atoi(line + 9);
        LOG_INFO("BT A2DP decoder: %s (%d)", BT_A2dpCodecStr(codec), codec);
        return;
    }

    if (strncmp(line, "+AVRCPCFG=", 10) == 0)
    {
        int cfg = atoi(line + 10);
        LOG_INFO("BT AVRCPCFG=%s: get track ID3=%s track progress freq=%d",
                 cfg,
                 (cfg & 1) ? "auto" : "off",
                 (cfg >> 1) & 0x3);

        /* Keep ID3 for each track, but disable play progress */
        if (cfg != BT_AVRCPCFG_DESIRED)
        {
            char cmd[24];
            LOG_WARN("BT AVRCPCFG differs from desired %d, updating", BT_AVRCPCFG_DESIRED);
            snprintf(cmd, sizeof(cmd), "AT+AVRCPCFG=%d", BT_AVRCPCFG_DESIRED);
            BT_SendCommand(cmd);
        }
        return;
    }

    /* ---- A2DP link state (authoritative connection indicator) ---------- */
    if (strncmp(line, "+A2DPSTAT=", 10) == 0)
    {
        int state = atoi(line + 10);
        bool connected = (state >= 3); /* Connected(3) or Streaming(4) */
        bool streaming = (state == 4); /* actually sending audio to us */

        if (streaming != bt_streaming)
        {
            LOG_INFO("BT %s", streaming ? "streaming" : "idle (connected)");
            if (streaming)
            {
                BT_SendCommand("AT+A2DPDEC");
            }
        }
        else if (connected != bt_connected)
        {
            LOG_INFO("BT %s (A2DP %s)",
                     connected ? "connected" : "disconnected",
                     BT_A2dpStateStr(state));
            if (connected)
            {
                BT_SendCommand("AT+A2DPDEC");
            }
        }
        else
        {
            LOG_DBG("BT A2DP state: %s (%d)", BT_A2dpStateStr(state), state);
        }

        bt_connected = connected;
        bt_streaming = streaming;
        return;
    }

    /* ---- media player state -------------------------------------------- */
    if (strncmp(line, "+PLAYSTAT=", 10) == 0)
    {
        int state = atoi(line + 10);
        LOG_INFO("BT player state: %s (%d)", BT_PlayStateStr(state), state);
        return;
    }

    /* ---- play progress: state, elapsed_ms, total_ms -------------------- */
    if (strncmp(line, "+TRACKSTAT=", 11) == 0)
    {
        int  state = 0;
        long elapsed = 0, total = 0;
        if (sscanf(line + 11, "%d,%ld,%ld", &state, &elapsed, &total) == 3)
            LOG_DBG("BT progress: %s %ld/%ld ms",
                    BT_PlayStateStr(state), elapsed, total);
        else
            LOG_DBG("BT TRACKSTAT parse failed: %s", line + 11);
        return;
    }

    /* ---- track metadata: title,artist,album ---------------------------- */
    if (strncmp(line, "+TRACKINFO=", 11) == 0)
    {
        LOG_INFO("BT track: %s", BT_Sanitize(line + 11));
        return;
    }

    if (strncmp(line, "+DEVSTAT=", 9) == 0)
    {
        LOG_INFO("Rec devStat: %s", line + 9);
        return;
    }
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
    LOG_DBG("BT unhandled line: %s", BT_Sanitize(line));

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