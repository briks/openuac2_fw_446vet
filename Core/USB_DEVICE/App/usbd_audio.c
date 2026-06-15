// USB Audio class 2.0 Implementation based on ST's 1.0 library

#include "usbd_audio.h"
#include "usbd_ctlreq.h"
#include "usbd_audio_if.h"
#include "audio_desc.h"
#include "es9038q2m.h"
#include "usb_device.h"

#ifdef USE_USBD_COMPOSITE
#error "Composite device is unsupported."
#endif

#define LOG_LEVEL LOG_LEVEL_DBG // Set to LOG_LEVEL_DBG for full logs
#include "log.h"

static uint8_t USBD_AUDIO_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_AUDIO_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_AUDIO_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t *USBD_AUDIO_GetCfgDesc(uint16_t *length);
static uint8_t *USBD_AUDIO_GetDeviceQualifierDesc(uint16_t *length);
static uint8_t USBD_AUDIO_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_EP0_RxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_AUDIO_EP0_TxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_AUDIO_SOF(USBD_HandleTypeDef *pdev);
static uint8_t USBD_AUDIO_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_IsoOutIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum);
static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_REQ_GetRange(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void *USBD_AUDIO_GetAudioHeaderDesc(uint8_t *pConfDesc);

static USBD_AUDIO_HandleTypeDef s_Haudio;

static USBD_InterruptControlTypedef s_interrupt_volume_ctrl __attribute__((aligned(4))) = {
    .binfo = 0,                          // 0 (Interface request)
    .bAttribute = 1,                     // 0x1 (CUR request)
    .wValueLowByte = CHANNEL_MASTER,     // CN Channel Number in low byte (0 master)
    .wValueHighByte = FU_VOLUME_CONTROL, // CS Channel Selector 0X1 FU_MUTE_CONTROL OR 0x2 FU_VOLUME_CONTROL
    .wIndexLowByte = AC_INTERFACE_NUM,   // interface 0
    .wIndexHighByte = FEATURE_UNIT_ID,   // 0x2 FEATURE UNIT ID
};

static USBD_InterruptControlTypedef s_interrupt_mute_ctrl __attribute__((aligned(4))) = {
    .binfo = 0,                        // 0 (Interface request)
    .bAttribute = 1,                   // 0x1 (CUR request)
    .wValueLowByte = CHANNEL_MASTER,   // CN Channel Number in low byte (0 master)
    .wValueHighByte = FU_MUTE_CONTROL, // CS Channel Selector 0X1 FU_MUTE_CONTROL OR 0x2 FU_VOLUME_CONTROL
    .wIndexLowByte = AC_INTERFACE_NUM, // interface 0
    .wIndexHighByte = FEATURE_UNIT_ID, // 0x2 FEATURE UNIT ID
};

USBD_ClassTypeDef USBD_AUDIO =
{
  USBD_AUDIO_Init,
  USBD_AUDIO_DeInit,
  USBD_AUDIO_Setup,
  USBD_AUDIO_EP0_TxReady,
  USBD_AUDIO_EP0_RxReady,
  USBD_AUDIO_DataIn,
  USBD_AUDIO_DataOut,
  USBD_AUDIO_SOF,
  USBD_AUDIO_IsoINIncomplete,
  USBD_AUDIO_IsoOutIncomplete,
  USBD_AUDIO_GetCfgDesc,
  USBD_AUDIO_GetCfgDesc,
  USBD_AUDIO_GetCfgDesc,
  USBD_AUDIO_GetDeviceQualifierDesc,
};

void USBD_AUDIO_signal_mute_change(void)
{
    s_Haudio.interrupt_mute_ctrl->wValueLowByte = CHANNEL_MASTER; // master channel only
    LOG_DBG("Interrupt message : 0x%02X%02X %02X%02X %02X%02X",
            s_Haudio.interrupt_mute_ctrl->binfo,
            s_Haudio.interrupt_mute_ctrl->bAttribute,
            s_Haudio.interrupt_mute_ctrl->wValueLowByte,
            s_Haudio.interrupt_mute_ctrl->wValueHighByte,
            s_Haudio.interrupt_mute_ctrl->wIndexLowByte,
            s_Haudio.interrupt_mute_ctrl->wIndexHighByte);

    // force data0
    // USBD_LL_ClearStallEP(&hUsbDeviceHS, INTERRUPT_EP_ADDR);
    if (USBD_LL_Transmit(&hUsbDeviceHS, INTERRUPT_EP_ADDR,
                          (uint8_t *)s_Haudio.interrupt_mute_ctrl,
                          INTERRUPT_PACKET_SIZE) != USBD_OK)
    {
        LOG_ERR("interrupt EP busy on mute signal (host will resync)");
    }
    LOG_INFO("signaled mute change on master channel.");
}

void USBD_AUDIO_signal_volume_change(void)
{
    s_Haudio.interrupt_volume_ctrl->wValueLowByte = CHANNEL_MASTER;

    LOG_DBG("Interrupt message : 0x%02X%02X %02X%02X %02X%02X",
            s_Haudio.interrupt_volume_ctrl->binfo,
            s_Haudio.interrupt_volume_ctrl->bAttribute,
            s_Haudio.interrupt_volume_ctrl->wValueLowByte,
            s_Haudio.interrupt_volume_ctrl->wValueHighByte,
            s_Haudio.interrupt_volume_ctrl->wIndexLowByte,
            s_Haudio.interrupt_volume_ctrl->wIndexHighByte);

    if (USBD_LL_Transmit(&hUsbDeviceHS, INTERRUPT_EP_ADDR,
                          (uint8_t *)s_Haudio.interrupt_volume_ctrl,
                          INTERRUPT_PACKET_SIZE) != USBD_OK)
    {
        LOG_ERR("interrupt EP busy on volume signal");
    }
    LOG_INFO("signaled volume change on CN master");
}

static uint8_t USBD_AUDIO_GetStreamType(USBD_HandleTypeDef* pdev)
{
    USBD_AUDIO_HandleTypeDef *haudio = pdev->pClassDataCmsit[pdev->classId];
    uint32_t *buf = haudio->pkt_buf;
    uint32_t rxSize = USBD_LL_GetRxDataSize(pdev, STREAMING_EP_NUM);

    const uint8_t marker_table[] = {0x05, 0xfa};

    uint8_t idx = 0;

    if (rxSize < AUDIO_DOP_DETECT_COUNT * sizeof(uint32_t))
    {
        return haudio->stream_type; /* keep current type, not enough data */
    }

	switch (*buf >> 24)
	{
	case 0x05:
		idx = 0;
		break;

	case 0xfa:
		idx = 1;
		break;

	default:
		return AUDIO_FORMAT_PCM;
		break;
	}

	for (uint32_t i = 0; i < AUDIO_DOP_DETECT_COUNT; i += 2)
	{
		if (buf[i] >> 24 != marker_table[idx] || buf[i + 1] >> 24 != marker_table[idx])
		{
			return AUDIO_FORMAT_PCM;
		}

		idx ^= 1;
	}

	return AUDIO_FORMAT_DSD;
}

void USBD_AUDIO_UpdateFB(USBD_HandleTypeDef *pdev)
{
    USBD_AUDIO_HandleTypeDef *haudio = pdev->pClassDataCmsit[pdev->classId];

    /* Use signed 64-bit to avoid uint32 underflow during subtraction. */
    int64_t tmp = ((int64_t)haudio->aud_buf.size
                 - (int64_t)(haudio->aud_buf.capacity >> 1)) << 3;

    /* Defensive clamp: limit deviation to ±1/8 of nominal (~12.5%).
     * Steady-state geometry already bounds this to ~6.25% in PCM,
     * so the clamp only acts on transients (sample-rate change,
     * buffer reset, init before sam_freq is known). */
    if (haudio->feedback_base != 0)
    {
        int32_t lim = (int32_t)(haudio->feedback_base >> 3);
        if (tmp >  lim) tmp =  lim;
        if (tmp < -lim) tmp = -lim;

        haudio->feedback_value = haudio->feedback_base - (int32_t)tmp;
    }
    else
    {
        haudio->feedback_value = 0;  /* no valid base yet */
    }
}

static uint8_t USBD_AUDIO_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    UNUSED(cfgidx);

    USBD_AUDIO_HandleTypeDef *haudio = &s_Haudio;
    haudio->interrupt_volume_ctrl = &s_interrupt_volume_ctrl;
    haudio->interrupt_mute_ctrl = &s_interrupt_mute_ctrl;

    pdev->pClassDataCmsit[pdev->classId] = haudio;
    pdev->pClassData = pdev->pClassDataCmsit[pdev->classId];

    if (pdev->dev_speed == USBD_SPEED_HIGH)
    {
        pdev->ep_out[STREAMING_EP_NUM].bInterval = STREAMING_HS_BINTERVAL;
        pdev->ep_in[FEEDBACK_EP_NUM].bInterval = FEEDBACK_HS_BINTERVAL;
        pdev->ep_in[INTERRUPT_EP_NUM].bInterval = INTERRUPT_HS_BINTERVAL;
    }
    else
    {
        LOG_ERR("USB Init: device not in HS mode (speed=%d)", pdev->dev_speed);
        return USBD_FAIL;
    }

    USBD_LL_FlushEP(pdev, STREAMING_EP_ADDR);
    USBD_LL_FlushEP(pdev, FEEDBACK_EP_ADDR);
    USBD_LL_FlushEP(pdev, INTERRUPT_EP_ADDR);
    if (USBD_LL_OpenEP(pdev, STREAMING_EP_ADDR, USBD_EP_TYPE_ISOC, USB_HS_MAX_PACKET_SIZE) != USBD_OK)
    {
        LOG_ERR("OpenEP streaming failed");
        return USBD_FAIL;
    }
    if (USBD_LL_OpenEP(pdev, FEEDBACK_EP_ADDR, USBD_EP_TYPE_ISOC, FEEDBACK_PACKET_SIZE) != USBD_OK)
    {
        LOG_ERR("OpenEP feedback failed");
        return USBD_FAIL;
    }
    if (USBD_LL_OpenEP(pdev, INTERRUPT_EP_ADDR, USBD_EP_TYPE_INTR, INTERRUPT_PACKET_SIZE) != USBD_OK)
    {
        LOG_ERR("OpenEP interrupt failed");
        return USBD_FAIL;
    }

    pdev->ep_out[STREAMING_EP_NUM].is_used = 1U;
    pdev->ep_in[FEEDBACK_EP_NUM].is_used = 1U;
    pdev->ep_in[INTERRUPT_EP_NUM].is_used = 1U;

    haudio->alt_setting = 0;
    haudio->stream_type = AUDIO_FORMAT_PCM;
    haudio->sam_freq = 48000U; /* default until host sets rate */
    AudioBuffer_Init(&haudio->aud_buf, 0);

    /* Initialize the Audio output Hardware layer */
    USBD_AUDIO_ItfTypeDef *itf = pdev->pUserData[pdev->classId];
    if (itf->AUDIO_Init() != USBD_OK)
    {
        LOG_ERR("USB Init: AUDIO_Init failed");
        return USBD_FAIL;
    }

    /* Prepare Out endpoint to receive 1st packet */
    USBD_LL_PrepareReceive(pdev, STREAMING_EP_ADDR, (uint8_t *)haudio->pkt_buf, USB_HS_MAX_PACKET_SIZE);
    USBD_LL_Transmit(pdev, FEEDBACK_EP_ADDR, (uint8_t *)&haudio->feedback_value, FEEDBACK_PACKET_SIZE);

    LOG_INFO("USB audio class init OK (HS)");
    return USBD_OK;
}

static uint8_t USBD_AUDIO_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

  USBD_LL_CloseEP(pdev, STREAMING_EP_ADDR);
  USBD_LL_CloseEP(pdev, FEEDBACK_EP_ADDR);
  USBD_LL_CloseEP(pdev, INTERRUPT_EP_ADDR);
  pdev->ep_out[STREAMING_EP_NUM].is_used = 0U;
  pdev->ep_out[STREAMING_EP_NUM].bInterval = 0U;
  pdev->ep_in[FEEDBACK_EP_NUM].is_used = 0U;
  pdev->ep_in[FEEDBACK_EP_NUM].bInterval = 0U;
  pdev->ep_in[INTERRUPT_EP_NUM].is_used = 0U;
  pdev->ep_in[INTERRUPT_EP_NUM].bInterval = 0U;

  if (pdev->pClassDataCmsit[pdev->classId] != NULL)
  {
      ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData[pdev->classId])->AUDIO_DeInit();
      pdev->pClassDataCmsit[pdev->classId] = NULL;
      pdev->pClassData = NULL;
  }

  LOG_INFO("USB audio class de-init");
  return USBD_OK;
}

static uint8_t USBD_AUDIO_Setup(USBD_HandleTypeDef *pdev,
                                USBD_SetupReqTypedef *req)
{
    LOG_DBG("Setup bmReq=0x%02X bReq=0x%02X wVal=0x%04X wIdx=0x%04X wLen=%u",
            req->bmRequest, req->bRequest, req->wValue, req->wIndex, req->wLength);
    USBD_AUDIO_HandleTypeDef *haudio = pdev->pClassDataCmsit[pdev->classId];
    uint16_t len;
    uint8_t *pbuf;
    uint16_t status_info = 0U;
    uint8_t idx;
    uint8_t ep_addr;

    switch (req->bmRequest & USB_REQ_TYPE_MASK)
    {
    case USB_REQ_TYPE_CLASS:
        switch (req->bRequest)
        {
        case AUDIO_REQ_CUR:
            if (req->bmRequest & 0x80)
            {
                LOG_DBG("SetupGetCurrent");
                AUDIO_REQ_GetCurrent(pdev, req);
            }
            else
            {
                LOG_DBG("SetupSetCurrent");
                AUDIO_REQ_SetCurrent(pdev, req);
            }
            break;

        case AUDIO_REQ_RANGE:
            if (req->bmRequest & 0x80)
            {
                LOG_DBG("SetupGetRange");
                AUDIO_REQ_GetRange(pdev, req);
            }
            else
            {
                goto ret_err;
            }
            break;

        default:
            goto ret_err;
            break;
        }
        break;

    case USB_REQ_TYPE_STANDARD:
        switch (req->bRequest)
        {
        case USB_REQ_GET_STATUS:
            if (pdev->dev_state == USBD_STATE_CONFIGURED)
            {
                LOG_DBG("SetupGetStatus");
                USBD_CtlSendData(pdev, (uint8_t *)&status_info, 2U);
            }
            else
            {
                goto ret_err;
            }
            break;

        case USB_REQ_GET_DESCRIPTOR:
            if (HIBYTE(req->wValue) == CS_DEVICE)
            {
                LOG_DBG("SetupGetDesc");
                pbuf = (uint8_t *)USBD_AUDIO_GetAudioHeaderDesc(pdev->pConfDesc);
                if (pbuf != NULL)
                {
                    len = MIN(USB_AUDIO_DESC_SIZE, req->wLength);
                    USBD_CtlSendData(pdev, pbuf, len);
                }
                else
                {
                    goto ret_err;
                }
            }
            break;

        case USB_REQ_GET_INTERFACE:
            if (pdev->dev_state == USBD_STATE_CONFIGURED)
            {
                LOG_DBG("SetupGetInterface");
                USBD_CtlSendData(pdev, (uint8_t *)&haudio->alt_setting, 1U);
            }
            else
            {
                goto ret_err;
            }
            break;

        case USB_REQ_SET_INTERFACE:
            if (pdev->dev_state == USBD_STATE_CONFIGURED)
            {
                LOG_DBG("SetupSetInterface");
                if ((uint8_t)(req->wValue) <= USBD_MAX_NUM_INTERFACES)
                {
                    uint8_t prev = haudio->alt_setting;
                    haudio->alt_setting = (uint8_t)(req->wValue);
                    haudio->bit_depth = (haudio->alt_setting == 1) ? 32U : 24U;
                    if (prev != haudio->alt_setting)
                    {
                        LOG_INFO("alt setting %u -> %u (bit_depth=%u)",
                                 prev, haudio->alt_setting, haudio->bit_depth);
                    }
                }
                else
                {
                    goto ret_err;
                }
            }
            else
            {
                goto ret_err;
            }
            break;

        case USB_REQ_CLEAR_FEATURE:
            LOG_WARN("SetupClearFeature: feature=0x%04X, index/endpoint=0x%04X", req->wValue, req->wIndex);
            ep_addr = LOBYTE(req->wIndex);
            switch (pdev->dev_state)
            {
            case USBD_STATE_ADDRESSED:
                if ((ep_addr != 0x00U) && (ep_addr != 0x80U))
                {
                    LOG_ERR("ClearFeature ADDRESSED, call StallEP");
                    (void)USBD_LL_StallEP(pdev, ep_addr);
                    (void)USBD_LL_StallEP(pdev, 0x80U);
                }
                else
                {
                    LOG_ERR("ClearFeature else ADDRESSED, call USBD_CtlError");
                    USBD_CtlError(pdev, req);
                }
                break;

            case USBD_STATE_CONFIGURED:
                if (req->wValue == USB_FEATURE_EP_HALT)
                {
                    if ((ep_addr & 0x7FU) != 0x00U)
                    {
                        USBD_LL_FlushEP(pdev, ep_addr);
                        (void)USBD_LL_ClearStallEP(pdev, ep_addr);

                        if (ep_addr == INTERRUPT_EP_ADDR)
                        {
                            /* Full re-init: resets PCD xfer state & data toggle */
                            USBD_LL_CloseEP(pdev, INTERRUPT_EP_ADDR);
                            USBD_LL_OpenEP(pdev, INTERRUPT_EP_ADDR,
                               USBD_EP_TYPE_INTR, INTERRUPT_PACKET_SIZE);
                            pdev->ep_in[INTERRUPT_EP_NUM].is_used = 1U;
                            pdev->ep_in[INTERRUPT_EP_NUM].bInterval = INTERRUPT_HS_BINTERVAL;

                            LOG_WARN("Interrupt EP fully reopened after ClearFeature");
                        }
                    }
                    (void)USBD_CtlSendStatus(pdev);
                }
                break;

            default:
                goto ret_err;
                break;
            }
            break;

        default:
            goto ret_err;
            break;
        }
        break;

    default:
        goto ret_err;
        break;
    }

    return USBD_OK;

ret_err:
    LOG_ERR("USB Setup: unsupported req bmReq=0x%02X bReq=0x%02X wVal=0x%04X wIdx=0x%04X",
            req->bmRequest, req->bRequest, req->wValue, req->wIndex);
    USBD_CtlError(pdev, req);
    return USBD_FAIL;
}

#ifndef USE_USBD_COMPOSITE

static uint8_t *USBD_AUDIO_GetCfgDesc(uint16_t *length)
{
	*length = USB_AUDIO_CONFIG_DESC_SIZE;
  return (uint8_t*)USBD_AUDIO_CfgDesc;
}
#endif /* USE_USBD_COMPOSITE  */

uint8_t rx_epum = 0xFF;

static uint8_t USBD_AUDIO_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    USBD_AUDIO_HandleTypeDef *haudio = pdev->pClassDataCmsit[pdev->classId];

    if (epnum == FEEDBACK_EP_NUM)
    {
        USBD_LL_Transmit(pdev, FEEDBACK_EP_ADDR,
                         (uint8_t *)&haudio->feedback_value, FEEDBACK_PACKET_SIZE);
    }
    else if (epnum == INTERRUPT_EP_NUM)
    {
    #if (LOG_LEVEL >= LOG_LEVEL_DBG)
            static uint32_t int_tx = 0;
            if (++int_tx % 10 == 0)
            {
                LOG_DBG("interrupt EP TX count=%lu", (unsigned long)int_tx);
            }
    #endif
    }
    else
    {
        LOG_ERR("DataIn on unexpected EP %u", epnum);
    }
    return USBD_OK;
}

static uint8_t USBD_AUDIO_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
    USBD_AUDIO_HandleTypeDef *haudio = pdev->pClassDataCmsit[pdev->classId];
    USBD_AUDIO_ItfTypeDef    *itf    = pdev->pUserData[pdev->classId];

    switch (haudio->control.unit)
    {
    case CLOCK_SOURCE_ID:
        if (haudio->control.cmd == CS_SAM_FREQ_CONTROL) {
            uint32_t prev = haudio->sam_freq;
            haudio->sam_freq = *(uint32_t*)haudio->control.data;
            if (prev != haudio->sam_freq) {
                LOG_INFO("sample rate %lu → %lu Hz",
                         (unsigned long)prev, (unsigned long)haudio->sam_freq);
            }
            uint32_t packetSize = (haudio->sam_freq % 48000U == 0) ? (haudio->sam_freq / 1000U) : (haudio->sam_freq / 147U * 160U / 1000U);
            haudio->buf_cap = packetSize * AUDIO_BUFFER_PACKET_NUM;

            AudioBuffer_Reset(&haudio->aud_buf, haudio->buf_cap);

            if (haudio->sam_freq % 48000U == 0)
            {
                haudio->feedback_base = haudio->sam_freq / 48000U * AUDIO_48K_FEEDBACK_VALUE;
            }
            else
            {
                haudio->feedback_base = haudio->sam_freq / 44100U * AUDIO_44K1_FEEDBACK_VALUE;
            }

            haudio->feedback_value = haudio->feedback_base;
            itf->AUDIO_Cmd(haudio->control.data, haudio->control.len, AUDIO_CMD_FREQ);
        } else {
            LOG_ERR("EP0_RxReady: unknown clock cmd 0x%02X", haudio->control.cmd);
            return USBD_FAIL;
        }
        break;

    case FEATURE_UNIT_ID:
        switch (haudio->control.cmd)
        {
        case FU_MUTE_CONTROL:
            LOG_INFO("FU_MUTE_CONTROL: index=0x%04X, value=0x%04X", pdev->request.wIndex, pdev->request.wValue);
            itf->AUDIO_Cmd(haudio->control.data, haudio->control.len, AUDIO_CMD_MUTE);
            break;
        case FU_VOLUME_CONTROL:
            /* 5.2.2 Control Request Layout,
            5.2.5.7 Feature Unit Control Request,
            5.2.5.7.2 Volume Control */
            /* The wValue field specifies the Control Selector (CS) in the high byte and the Channel Number (CN) in the low byte*/
            LOG_INFO("FU_VOLUME_CONTROL: index=0x%04X, value=0x%04X", pdev->request.wIndex, pdev->request.wValue);
            if (LOBYTE(pdev->request.wValue) == 1)
            {
                itf->AUDIO_Cmd(haudio->control.data, haudio->control.len, AUDIO_CMD_VOLUME_CH1);
            }
            else if (LOBYTE(pdev->request.wValue) == 2)
            {
                itf->AUDIO_Cmd(haudio->control.data, haudio->control.len, AUDIO_CMD_VOLUME_CH2);
            }
            else
            {// 0 could be master channel, but we don't support it
                LOG_ERR("EP0_RxReady: unknown channel number %u for volume control", LOBYTE(pdev->request.wValue));
                return USBD_FAIL;
            }
            break;
        default:
            LOG_ERR("EP0_RxReady: unknown feature cmd 0x%02X", haudio->control.cmd);
            return USBD_FAIL;
        }
        break;

    default:
        LOG_ERR("EP0_RxReady: unknown unit 0x%02X", haudio->control.unit);
        return USBD_FAIL;
    }
    return USBD_OK;
}

static uint8_t USBD_AUDIO_EP0_TxReady(USBD_HandleTypeDef *pdev)
{
	UNUSED(pdev);
  return USBD_OK;
}

static uint8_t USBD_AUDIO_SOF(USBD_HandleTypeDef *pdev)
{
	UNUSED(pdev);
  return USBD_OK;
}

void USBD_AUDIO_Sync(USBD_HandleTypeDef *pdev)
{
    USBD_AUDIO_HandleTypeDef* haudio = pdev->pClassDataCmsit[pdev->classId];

    if (haudio->state == AUDIO_STATE_STOPPED)
    return;

    AudioBuffer_Sync(&haudio->aud_buf, AUDIO_SYNC_CLK_DIV << 3);
    USBD_AUDIO_UpdateFB(&hUsbDeviceHS);

#if 0
    static uint32_t dbg_cnt = 0;
    if (++dbg_cnt >= 150) {  /* ~10 Hz at 1.5 kHz Sync rate */
        dbg_cnt = 0;
        LOG_DBG("fb=0x%05lX base=0x%05lX buf=%lu/%lu",
                (unsigned long)haudio->feedback_value,
                (unsigned long)haudio->feedback_base,
                (unsigned long)haudio->aud_buf.size,
                (unsigned long)haudio->aud_buf.capacity);
    }
#endif

    if ((haudio->aud_buf.state == AB_UDFL) && (haudio->state == AUDIO_STATE_PLAYING))
    {
        LOG_INFO("audio -> STOPPED (underflow)");
        haudio->state = AUDIO_STATE_STOPPED;
        haudio->stream_type = AUDIO_FORMAT_PCM;
        es9038q2m_audio_stop_pending = true;
    }

    /* LED logic */
    if (   haudio->aud_buf.capacity != 0
        && haudio->aud_buf.size * 4 > haudio->aud_buf.capacity * 3)
    {
        LOG_WARN("audio buf reach 3/4 capacity, size=%lu capacity=%lu",
                 (unsigned long)haudio->aud_buf.size, (unsigned long)haudio->aud_buf.capacity);
    }
}

static uint8_t USBD_AUDIO_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    USBD_AUDIO_HandleTypeDef* haudio = pdev->pClassDataCmsit[pdev->classId];

    if (epnum == FEEDBACK_EP_NUM)
    {
        USBD_LL_Transmit(pdev, FEEDBACK_EP_ADDR,
                         (uint8_t *)&haudio->feedback_value, FEEDBACK_PACKET_SIZE);
#if 0
        static uint32_t incomp_in = 0;
        if (++incomp_in % 100 == 0)
        {
            LOG_DBG("IsoIN incomplete count=%lu", (unsigned long)incomp_in);
        }
#endif
    }
    return USBD_OK;
}

static uint8_t USBD_AUDIO_IsoOutIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    USBD_AUDIO_HandleTypeDef* haudio = pdev->pClassDataCmsit[pdev->classId];

    if (epnum == STREAMING_EP_NUM) {
        USBD_LL_PrepareReceive(pdev, STREAMING_EP_ADDR,
                               (uint8_t*)haudio->pkt_buf, USB_HS_MAX_PACKET_SIZE);
#if 0
        static uint32_t incomp_out = 0;
        if (++incomp_out % 100 == 0) {
            LOG_DBG("IsoOUT incomplete count=%lu", (unsigned long)incomp_out);
        }
#endif
    }
    return USBD_OK;
}

static uint8_t USBD_AUDIO_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    USBD_AUDIO_HandleTypeDef *haudio = pdev->pClassDataCmsit[pdev->classId];
    USBD_AUDIO_ItfTypeDef* itf = pdev->pUserData[pdev->classId];

    if (haudio == NULL) {
        LOG_ERR("DataOut: haudio NULL");
        return USBD_FAIL;
    }

    if (epnum == STREAMING_EP_NUM)
    {
        uint8_t stream_type = USBD_AUDIO_GetStreamType(pdev);

        if (haudio->stream_type != stream_type)
        {
        	if (stream_type == AUDIO_FORMAT_DSD)
        	{
//    		AudioBuffer_Reset(&haudio->aud_buf, haudio->buf_cap >> 1);
        	}

        	haudio->stream_type = stream_type;
            itf->AUDIO_Cmd(&stream_type, 1, AUDIO_CMD_FORMAT);
        }

    		uint32_t packetSize = USBD_LL_GetRxDataSize(pdev, epnum);

#if 0
        static uint32_t pkt_cnt = 0;
        static uint32_t pkt_bytes = 0;
        pkt_cnt++;
        pkt_bytes += packetSize;
        if (pkt_cnt >= 4000) {  /* ~0.5 s at 8 kHz packet rate */
            LOG_DBG("rx %lu pkts, %lu bytes avg=%lu",
                    (unsigned long)pkt_cnt,
                    (unsigned long)pkt_bytes,
                    (unsigned long)(pkt_bytes / pkt_cnt));
            pkt_cnt = 0;
            pkt_bytes = 0;
        }
#endif

    		if (haudio->stream_type == AUDIO_FORMAT_PCM)
    		{
    			uint32_t* pDst = (uint32_t*)&haudio->aud_buf.mem[haudio->aud_buf.wr_ptr];
    			uint32_t* pSrc = haudio->pkt_buf;
    			uint32_t* pEnd = (uint32_t*)&haudio->aud_buf.mem[haudio->aud_buf.capacity];

    			for (uint32_t i = 0; i < (packetSize >> 2); ++i)
    			{
    				union
    				{
    					uint16_t x[2];
    					uint32_t y;
    				} tmp;

    				tmp.y = *pSrc++;
    				*pDst++ = (tmp.x[0] << 16) | tmp.x[1];

    				if (pDst == pEnd)
    				{
    					pDst = (uint32_t*)haudio->aud_buf.mem;
    				}
    			}

    			AudioBuffer_Receive(&haudio->aud_buf, packetSize);
    		}
    		else
    		{
    			uint16_t* pDst[2];
    			pDst[0] = (uint16_t*)&haudio->aud_buf.mem[haudio->aud_buf.wr_ptr];
    			pDst[1] =	(uint16_t*)&haudio->aud_buf.mem[haudio->aud_buf.wr_ptr + haudio->aud_buf.capacity];
    			uint32_t* pSrc = haudio->pkt_buf;
    			uint16_t* pEnd = (uint16_t*)&haudio->aud_buf.mem[haudio->aud_buf.capacity];
    			uint8_t idx = 1;

    			for (uint32_t i = 0; i < (packetSize >> 2); ++i)
    			{
    				*pDst[idx]++ = (*pSrc++ >> 8) & 0xffff;
    				idx ^= 1;

    				if (pDst[0] == pEnd)
    				{
    					pDst[0] = (uint16_t*)haudio->aud_buf.mem;
    					pDst[1] = pEnd;
    				}
    			}

    			AudioBuffer_Receive(&haudio->aud_buf, packetSize >> 2);
    		}

    		if ((haudio->state == AUDIO_STATE_STOPPED) && (haudio->aud_buf.size > haudio->aud_buf.capacity >> 1))
    		{
                itf->AUDIO_Cmd(NULL, 0, AUDIO_CMD_PLAY);
                haudio->state = AUDIO_STATE_PLAYING;
    		}

    		if (haudio->aud_buf.size > haudio->aud_buf.capacity - (haudio->aud_buf.capacity >> 2))
    		{ // never occurs ?
    			//LL_GPIO_ResetOutputPin(LED3_LINE_GPIO_Port, LED3_LINE_Pin);
    		}
    		else
    		{
    			//LL_GPIO_SetOutputPin(LED3_LINE_GPIO_Port, LED3_LINE_Pin);
    		}

    		USBD_LL_PrepareReceive(pdev, STREAMING_EP_ADDR, (uint8_t*)haudio->pkt_buf, USB_HS_MAX_PACKET_SIZE);
    }

    return USBD_OK;
}

static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    USBD_AUDIO_HandleTypeDef *haudio = pdev->pClassDataCmsit[pdev->classId];
    bool get_current_mute_received = false;

    if (haudio == NULL)
    {
        return;
    }

    USBD_memset(haudio->control.data, 0, USB_MAX_EP0_SIZE);
    uint8_t *pbuf = haudio->control.data;

    LOG_DBG("GetCurrent: cmd=0x%02X, unit=0x%02X, len=%u", haudio->control.cmd, haudio->control.unit, haudio->control.len);

    switch (HIBYTE(req->wIndex))
    {
    case FEATURE_UNIT_ID:
        if (HIBYTE(req->wValue) == FU_VOLUME_CONTROL)
        {
            // Use resquested values to match interrupt sent on change.
            // Send new volume even if not applied yet
            if (LOBYTE(req->wValue) == CHANNEL_MASTER)
            {
                LOG_INFO("GetCurrent: volume requested by host, channel=%u, value=%d, size=%u",
                         LOBYTE(req->wValue), requested_volume_ch1, req->wLength);
                SET_DATA(pbuf, int16_t, (requested_volume_ch1 + requested_volume_ch2) / 2);
            }
            else if (LOBYTE(req->wValue) == CHANNEL_1)
            {
                LOG_INFO("GetCurrent: volume requested by host, channel=%u, value=%d, size=%u",
                         LOBYTE(req->wValue), requested_volume_ch1, req->wLength);
                SET_DATA(pbuf, int16_t, requested_volume_ch1);
            }
            else if (LOBYTE(req->wValue) == CHANNEL_2)
            {
                LOG_INFO("GetCurrent: volume requested by host, channel=%u, value=%d, size=%u", 
                    LOBYTE(req->wValue), requested_volume_ch2, req->wLength);
                SET_DATA(pbuf, int16_t, requested_volume_ch2);
            }
            else
            { // 0 could be master channel, but we don't support it
                LOG_ERR("GetCurrent: unknown channel number %u for volume control", 
                    LOBYTE(req->wValue));
                USBD_CtlError(pdev, req);
                return;
            }
        }
        else if (HIBYTE(req->wValue) == FU_MUTE_CONTROL)
        {
            // Use resquested values to match interrupt sent on change.
            // Send new mute even if not applied yet
            LOG_INFO("GetCurrent: mute state requested by host, channel=%u, returning %s, size=%u",
                     LOBYTE(req->wValue), requested_mute ? "ON" : "OFF", req->wLength);
            SET_DATA(pbuf, uint8_t, requested_mute ? 1 : 0); // indicate to windows the mute state to display at startup, should reflect the internal state.
            get_current_mute_received = true;
        }
        else
        {
            LOG_ERR("GetCurrent: unknown feature wValue=0x%04X", req->wValue);
            USBD_CtlError(pdev, req);
            return;
        }
        break;

    case CLOCK_SOURCE_ID:
        if (HIBYTE(req->wValue) == CS_SAM_FREQ_CONTROL)
        {
            LOG_INFO("GetCurrent: sample frequency requested by host, value=%lu Hz", (unsigned long)haudio->sam_freq);
            SET_DATA(pbuf, uint32_t, haudio->sam_freq);
        }
        else
        {
            LOG_ERR("GetCurrent: unknown clock wValue=0x%04X", req->wValue);
            USBD_CtlError(pdev, req);
            return;
        }
        break;

    default:
        LOG_ERR("GetCurrent: unknown unit 0x%02X", HIBYTE(req->wIndex));
        USBD_CtlError(pdev, req);
        break;
    }

    USBD_CtlSendData(pdev, haudio->control.data, MIN(req->wLength, USB_MAX_EP0_SIZE));
}

static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_HandleTypeDef* haudio = pdev->pClassDataCmsit[pdev->classId];

  if (haudio == NULL)
  {
    return;
  }

  if (req->wLength != 0U)
  {
  	haudio->control.cmd = HIBYTE(req->wValue);
  	haudio->control.len = (uint8_t)MIN(req->wLength, USB_MAX_EP0_SIZE);
  	haudio->control.unit = HIBYTE(req->wIndex);

  	USBD_CtlPrepareRx(pdev, haudio->control.data, haudio->control.len);
  }
}

static void AUDIO_REQ_GetRange(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
	USBD_AUDIO_HandleTypeDef* haudio = pdev->pClassDataCmsit[pdev->classId];

	if (haudio == NULL)
	{
		return;
	}

	USBD_memset(haudio->control.data, 0, USB_MAX_EP0_SIZE);

	uint8_t* pbuf = haudio->control.data;

	switch (HIBYTE(req->wIndex))
	{
	case CLOCK_SOURCE_ID:
		if (HIBYTE(req->wValue) == CS_SAM_FREQ_CONTROL)
		{
			SET_DATA(pbuf, uint16_t, 1U);
			SET_DATA(pbuf, uint32_t, AUDIO_MIN_FREQ);
			SET_DATA(pbuf, uint32_t, AUDIO_MAX_FREQ);
			SET_DATA(pbuf, uint32_t, AUDIO_FREQ_RES);
		}
		else {
			LOG_ERR("GetRange: unknown clock wValue=0x%04X", req->wValue);
			USBD_CtlError(pdev, req);
			return;
		}
		break;

	case FEATURE_UNIT_ID:
		if (HIBYTE(req->wValue) == FU_VOLUME_CONTROL)
		{
			SET_DATA(pbuf, uint16_t, 1U); // Number of subrange below
			SET_DATA(pbuf, int16_t, AUDIO_MIN_VOL);
			SET_DATA(pbuf, int16_t, AUDIO_MAX_VOL);
			SET_DATA(pbuf, int16_t, AUDIO_VOL_RES);
		}
		else {
			LOG_ERR("GetRange: unknown feature wValue=0x%04X", req->wValue);
			USBD_CtlError(pdev, req);
			return;
		}
		break;

	default:
		LOG_ERR("GetRange: unknown unit 0x%02X", HIBYTE(req->wIndex));
		USBD_CtlError(pdev, req);
		return;
		break;
	}

	USBD_CtlSendData(pdev, haudio->control.data, MIN(req->wLength, USB_MAX_EP0_SIZE));
}

#ifndef USE_USBD_COMPOSITE

static uint8_t *USBD_AUDIO_GetDeviceQualifierDesc(uint16_t *length)
{
  *length = USB_LEN_DEV_QUALIFIER_DESC;

  return (uint8_t*)USBD_AUDIO_DeviceQualifierDesc;
}

#endif /* USE_USBD_COMPOSITE  */

uint8_t USBD_AUDIO_RegisterInterface(USBD_HandleTypeDef *pdev,
                                     USBD_AUDIO_ItfTypeDef *fops)
{
  if (fops == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  pdev->pUserData[pdev->classId] = fops;

  return (uint8_t)USBD_OK;
}

static void *USBD_AUDIO_GetAudioHeaderDesc(uint8_t *pConfDesc)
{
  USBD_ConfigDescTypeDef *desc = (USBD_ConfigDescTypeDef *)(void *)pConfDesc;
  USBD_DescHeaderTypeDef *pdesc = (USBD_DescHeaderTypeDef *)(void *)pConfDesc;
  uint8_t *pAudioDesc =  NULL;
  uint16_t ptr;

  if (desc->wTotalLength > desc->bLength)
  {
    ptr = desc->bLength;

    while (ptr < desc->wTotalLength)
    {
      pdesc = USBD_GetNextDesc((uint8_t *)pdesc, &ptr);
      if ((pdesc->bDescriptorType == CS_INTERFACE) &&
          (pdesc->bDescriptorSubType == HEADER))
      {
        pAudioDesc = (uint8_t *)pdesc;
        break;
      }
    }
  }
  return pAudioDesc;
}
