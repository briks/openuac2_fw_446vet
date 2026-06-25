#include "es9038q2m.h"
#include "usbd_audio_if.h"
#include "main.h"
#include "usb_device.h"
#define LOG_LEVEL LOG_LEVEL_INFO
#include "log.h"
#include "usbd_audio.h"     /* AUDIO_MIN_VOL / AUDIO_MAX_VOL */

/* Windows volume taper, full 1% resolution (index = slider%, 0..100),
 * captured at RES=32 (0.125 dB grid). value = exact Q8.8 dB Windows sends.
 * Shape ~33*log10(p) but not a clean formula -> table is authoritative. */
#define WIN_TAPER_N     101
#define WIN_TAPER_STEPS (WIN_TAPER_N - 1)   /* 100 */
static const int16_t win_taper[WIN_TAPER_N] = {
  -16384,-14400,-13088,-12128,-11328,-10688,-10144, -9632, -9216, -8832, /*  0.. 9% */
   -8480, -8160, -7840, -7584, -7296, -7072, -6848, -6624, -6432, -6208, /* 10..19% */
   -6048, -5856, -5696, -5536, -5376, -5216, -5088, -4928, -4800, -4672, /* 20..29% */
   -4544, -4416, -4320, -4192, -4096, -3968, -3872, -3776, -3680, -3584, /* 30..39% */
   -3488, -3392, -3296, -3200, -3136, -3040, -2944, -2880, -2784, -2720, /* 40..49% */
   -2624, -2560, -2496, -2432, -2336, -2272, -2208, -2144, -2080, -2016, /* 50..59% */
   -1952, -1888, -1824, -1760, -1696, -1632, -1600, -1536, -1472, -1408, /* 60..69% */
    -1376, -1312, -1248, -1216, -1152, -1088, -1056,  -992,  -960,  -896, /* 70..79% */
    -864,  -800,  -768,  -704,  -672,  -608,  -576,  -544,  -480,  -448, /* 80..89% */
    -416,  -352,  -320,  -288,  -224,  -192,  -160,  -128,   -64,   -32, /* 90..99% */
       0                                                                  /* 100% */
};

/* curved q88 (Windows) -> slider fraction Q16 (0..65536), via inverse search */
static int32_t win_curved_to_fracQ16(int16_t v)
{
    if (v <= win_taper[0])               return 0;
    if (v >= win_taper[WIN_TAPER_N - 1]) return 65536;

    int lo = 0, hi = WIN_TAPER_N - 1;          /* table monotonic non-decreasing */
    while (hi - lo > 1) {
        int mid = (lo + hi) >> 1;
        if (win_taper[mid] <= v) lo = mid; else hi = mid;
    }
    int32_t span = win_taper[hi] - win_taper[lo];
    int64_t fin  = span ? (((int64_t)(v - win_taper[lo]) << 16) + span / 2) / span : 0;
    int64_t pos  = ((int64_t)lo << 16) + fin;  /* 0..STEPS, Q16 */
    return (int32_t)((pos + WIN_TAPER_STEPS / 2) / WIN_TAPER_STEPS);
}

/* slider fraction Q16 -> curved q88 (for GetCurrent round-trip) */
static int16_t win_fracQ16_to_curved(int32_t frac)
{
    if (frac <= 0)     return win_taper[0];
    if (frac >= 65536) return win_taper[WIN_TAPER_N - 1];

    int64_t pos = (int64_t)frac * WIN_TAPER_STEPS;     /* index, Q16 */
    int     idx = (int)(pos >> 16);
    int32_t rem = (int32_t)(pos & 0xFFFF);
    int16_t a = win_taper[idx], b = win_taper[idx + 1];
    return (int16_t)(a + (((int32_t)(b - a) * rem + 32768) >> 16));
}

/* Public: linearize the curved value Windows sends. */
int16_t USB_VolCurveToLinear(int16_t curved_q88)
{
    int32_t f     = win_curved_to_fracQ16(curved_q88);
    int32_t range = AUDIO_MAX_VOL - AUDIO_MIN_VOL;
    return (int16_t)(AUDIO_MIN_VOL + (int32_t)(((int64_t)range * f + 32768) >> 16));
}

/* Public: inverse, used when reporting Current back to the host. */
int16_t USB_VolLinearToCurve(int16_t linear_q88)
{
    int32_t range = AUDIO_MAX_VOL - AUDIO_MIN_VOL;
    int32_t f = (int32_t)((((int64_t)(linear_q88 - AUDIO_MIN_VOL) << 16) + range / 2) / range);
    return win_fracQ16_to_curved(f);
}

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

static bool RCC_I2S_SetFreq(uint32_t freq)
{
    __HAL_I2S_DISABLE(&AUDIO_I2S_MSTR_HANDLE);
    LL_RCC_PLLI2S_Disable();

    uint32_t divisor;
    if (freq % 48000U == 0)
    {
        LL_RCC_PLLI2S_ConfigDomain_I2S(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLI2SM_DIV_16,
                                       128, LL_RCC_PLLI2SR_DIV_2);
        divisor = PLLI2SQ_48K / (freq << 7U);
    }
    else
    {
        LL_RCC_PLLI2S_ConfigDomain_I2S(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLI2SM_DIV_20,
                                       147, LL_RCC_PLLI2SR_DIV_2);
        divisor = PLLI2SQ_44K1 / (freq << 7U);
    }

    /* I2SDIV 0 and 1 are forbidden on STM32F4. A request that resolves to <2
     * is an unsupportable PCM rate (e.g. 705600 = DSD256 DoP transport rate).
     * Skip it: the DSD path will set the real (sam_freq>>2) clock shortly. */
    if (divisor < 2U)
    {
        LOG_ERR("I2S freq %lu Hz needs I2SDIV=%lu (<2, forbidden) - skipped",
                (unsigned long)freq, (unsigned long)divisor);
        LL_RCC_PLLI2S_Enable();
        return false;
    }

    MODIFY_REG(AUDIO_I2S_MSTR_HANDLE.Instance->I2SPR, SPI_I2SPR_I2SDIV_Msk, divisor);

    LL_RCC_PLLI2S_Enable();
    uint32_t timeout = 100000;
    while (!LL_RCC_PLLI2S_IsReady() && --timeout) { __NOP(); }
    if (timeout == 0)
    {
        LOG_ERR("PLLI2S failed to lock for freq=%lu Hz", (unsigned long)freq);
        return false;
    }

    LOG_WARN("PLLI2S locked, freq=%lu Hz, I2SDIV=%lu",
             (unsigned long)freq, (unsigned long)divisor);

    __HAL_I2S_ENABLE(&AUDIO_I2S_MSTR_HANDLE);
    return true;
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
            codec->DAC_Format(*pbuf);

        HAL_I2S_DMAStop(&AUDIO_I2S_MSTR_HANDLE);
        HAL_I2S_DMAStop(&AUDIO_I2S_SLAVE_HANDLE);

        if (*pbuf == AUDIO_FORMAT_DSD)
        {
            RCC_I2S_SetFreq(haudio->sam_freq >> 2);
            LL_GPIO_SetOutputPin(DSDOE_GPIO_Port, DSDOE_Pin);
            LOG_WARN("Switch to DSD");
        }
        else
        {
            LL_GPIO_ResetOutputPin(DSDOE_GPIO_Port, DSDOE_Pin);
            RCC_I2S_SetFreq(haudio->sam_freq);
            LOG_WARN("Switch to PCM");
        }
        /* No DMA restart here: state is STOPPED; PLAY re-primes and restarts. */
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
            codec->DAC_Freq(*(uint32_t *)pbuf);
        /* This sets the PCM clock; record whether the rate is PCM-playable.
         * DSD-only rates (e.g. 705600) will report false here. */
        haudio->pcm_clock_ok = RCC_I2S_SetFreq(*(uint32_t*)pbuf);
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
            int16_t vol = (int16_t)(pbuf[0] | (pbuf[1] << 8));
            vol = USB_VolCurveToLinear(vol);          /* undo Windows taper */
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


