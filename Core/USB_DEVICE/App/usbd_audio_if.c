#include "es9038q2m.h"
#include "usbd_audio_if.h"
#include "main.h"
#include "usb_device.h"
#define LOG_LEVEL LOG_LEVEL_INFO
#include "log.h"


extern I2S_HandleTypeDef AUDIO_I2S_MSTR_HANDLE;
extern I2S_HandleTypeDef AUDIO_I2S_SLAVE_HANDLE;
extern DMA_HandleTypeDef hdma_memtomem_dma2_stream0;

static uint8_t AUDIO_Init();
static uint8_t AUDIO_DeInit();
static uint8_t AUDIO_Cmd(uint8_t* pbuf, uint32_t size, uint8_t cmd);
static uint8_t AUDIO_GetState();

static const AUDIO_CodecTypeDef* const codec = &es9038q2m_instance;

USBD_AUDIO_ItfTypeDef USBD_AUDIO_fops =
{
  AUDIO_Init,
  AUDIO_DeInit,
  AUDIO_Cmd,
  AUDIO_GetState,
};

static void RCC_I2S_SetFreq(uint32_t freq)
{
    __HAL_I2S_DISABLE(&AUDIO_I2S_MSTR_HANDLE);
    LL_RCC_PLLI2S_Disable();

    uint32_t divisor;
    if (freq % 48000U == 0)
    {
        LL_RCC_PLLI2S_ConfigDomain_I2S(LL_RCC_PLLSOURCE_HSE,
                                       LL_RCC_PLLI2SM_DIV_16,
                                       128,
                                       LL_RCC_PLLI2SR_DIV_2);
        divisor = PLLI2SQ_48K / (freq << 7U);
        MODIFY_REG(AUDIO_I2S_MSTR_HANDLE.Instance->I2SPR, SPI_I2SPR_I2SDIV_Msk, divisor);
        LOG_DBG("PLLI2S → 48k family, N=128, I2SDIV=%lu", (unsigned long) divisor);
    }
    else
    {
        LL_RCC_PLLI2S_ConfigDomain_I2S(LL_RCC_PLLSOURCE_HSE,
                                       LL_RCC_PLLI2SM_DIV_20,
                                       147,
                                       LL_RCC_PLLI2SR_DIV_2);
        divisor = PLLI2SQ_44K1 / (freq << 7U);
        MODIFY_REG(AUDIO_I2S_MSTR_HANDLE.Instance->I2SPR, SPI_I2SPR_I2SDIV_Msk, divisor);
        LOG_DBG("PLLI2S → 44k1 family, N=147, I2SDIV=%lu", (unsigned long) divisor);
    }

    LL_RCC_PLLI2S_Enable();
    uint32_t timeout = 100000;
    while (!LL_RCC_PLLI2S_IsReady() && --timeout)
    {
        __NOP();
    }
    if (timeout == 0)
    {
        LOG_ERR("PLLI2S failed to lock for freq=%lu Hz", (unsigned long) freq);
    }
    __HAL_I2S_ENABLE(&AUDIO_I2S_MSTR_HANDLE);
}

static uint8_t AUDIO_Init()
{
    if (codec->DAC_Init != NULL)
    {
        codec->DAC_Init();
    }

  return USBD_OK;
}

static uint8_t AUDIO_DeInit()
{
    if (codec->DAC_DeInit != NULL)
    {
        codec->DAC_DeInit();
    }

  return USBD_OK;
}

/* Called from USBD_AUDIO_EP0_RxReady, USBD_AUDIO_Sync, USBD_AUDIO_DataOut */
static uint8_t AUDIO_Cmd(uint8_t* pbuf, uint32_t size, uint8_t cmd)
{
	USBD_AUDIO_HandleTypeDef* haudio = hUsbDeviceHS.pClassDataCmsit[hUsbDeviceHS.classId];
	AudioBuffer* aud_buf = &haudio->aud_buf;

  switch (cmd)
  {
	case AUDIO_CMD_FORMAT:
        if (codec->DAC_Format != NULL)
        {
		    codec->DAC_Format(*pbuf);
		}
		/* Format change: tear down current I2S, reconfigure clocks/GPIOs.
		* DMA will be (re)started by PLAY, or here if we're already playing. */
		HAL_I2S_DMAStop(&AUDIO_I2S_MSTR_HANDLE);
		HAL_I2S_DMAStop(&AUDIO_I2S_SLAVE_HANDLE);

		if (*pbuf == AUDIO_FORMAT_DSD)
		{
			RCC_I2S_SetFreq(haudio->sam_freq >> 2);
			LL_GPIO_SetOutputPin(DSDOE_GPIO_Port, DSDOE_Pin);
            LOG_WARN("Switch to DSD");
		}
		else  /* AUDIO_FORMAT_PCM */
		{
			LL_GPIO_ResetOutputPin(DSDOE_GPIO_Port, DSDOE_Pin);
			RCC_I2S_SetFreq(haudio->sam_freq);
            LOG_WARN("Switch to PCM");
		}

		/* If already playing, restart DMA in the new format. */
		if (haudio->state == AUDIO_STATE_PLAYING)
		{
			if (*pbuf == AUDIO_FORMAT_DSD)
			{
				HAL_I2S_Transmit_DMA(&AUDIO_I2S_SLAVE_HANDLE,
									(uint16_t*)&aud_buf->mem[aud_buf->capacity],
									aud_buf->capacity >> 2);
			}
			HAL_I2S_Transmit_DMA(&AUDIO_I2S_MSTR_HANDLE,
								(uint16_t*)aud_buf->mem,
								aud_buf->capacity >> 2);
		}
		break;

    case AUDIO_CMD_PLAY:
        if (codec->DAC_Play != NULL)
            codec->DAC_Play();
        if (haudio->stream_type == AUDIO_FORMAT_DSD)
        {
            if (HAL_I2S_Transmit_DMA(&AUDIO_I2S_SLAVE_HANDLE,
                                     (uint16_t *) &aud_buf->mem[aud_buf->capacity],
                                     aud_buf->capacity >> 2)
                != HAL_OK)
            {
                LOG_ERR("I2S slave DMA start failed (DSD play)");
            }
        }
        if (HAL_I2S_Transmit_DMA(&AUDIO_I2S_MSTR_HANDLE,
                                 (uint16_t *) aud_buf->mem,
                                 aud_buf->capacity >> 2)
            != HAL_OK)
        {
            LOG_ERR("I2S master DMA start failed (play)");
        }
        break;

    case AUDIO_CMD_STOP:
        if (codec->DAC_Stop != NULL)
        {
            codec->DAC_Stop();
        }
		HAL_I2S_DMAStop(&AUDIO_I2S_MSTR_HANDLE);
		HAL_I2S_DMAStop(&AUDIO_I2S_SLAVE_HANDLE);   /* harmless if not running */
		LL_GPIO_ResetOutputPin(DSDOE_GPIO_Port, DSDOE_Pin);
		break;

	case AUDIO_CMD_FREQ:
        if (codec->DAC_Freq != NULL)
        {
            codec->DAC_Freq(*(uint32_t *)pbuf);
        }
		RCC_I2S_SetFreq(*(uint32_t*)pbuf);
		break;

	case AUDIO_CMD_MUTE:
        if (codec->DAC_Mute != NULL)
        {
            codec->DAC_Mute(*pbuf == 1);
        }
		break;

    case AUDIO_CMD_VOLUME_MASTER:
	case AUDIO_CMD_VOLUME_CH1:
    case AUDIO_CMD_VOLUME_CH2:
        if (codec->DAC_Volume != NULL && size == 2)
        {
            /* little-endian q8.8 */
            int16_t vol = (int16_t)pbuf[0];
            vol |= ((int16_t)pbuf[1]) << 8;
            LOG_INFO("Volume command CN %d with vol %d",
                     cmd - AUDIO_CMD_VOLUME_MASTER, vol);
            codec->DAC_Volume(vol, cmd - AUDIO_CMD_VOLUME_MASTER);
        }
        else
        {
            LOG_ERR("Volume command with invalid size %lu", (unsigned long)size);
        }
        break;

	default:
		break;
  }

  return USBD_OK;
}

/* not called */
static uint8_t AUDIO_GetState()
{
	return USBD_OK;
}


