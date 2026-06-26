/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"
#include "usbd_conf.h"
#include "es9038q2m.h"
#include "pga2311.h"
#include "bluetooth.h"
#include "SEGGER_RTT.h"
#include "stm32f4xx_it.h"
#include "mx_init.h"
#define LOG_LEVEL LOG_LEVEL_DBG
#include "log.h"

/* Private variables ---------------------------------------------------------*/
osThreadId defaultTaskHandle;
#define defaultTaskBufferSize 2048
uint32_t defaultTaskBuffer[defaultTaskBufferSize / sizeof(uint32_t)];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId VolumeHandle;
#define VolumeBufferSize 1024
uint32_t VolumeBuffer[VolumeBufferSize / sizeof(uint32_t)];
osStaticThreadDef_t VolumeControlBlock;
osThreadId LedsHandle;
#define LedsBufferSize 256
uint32_t LedsBuffer[LedsBufferSize / sizeof(uint32_t)];
osStaticThreadDef_t LedsControlBlock;
osThreadId SourceHandle;
#define SourceBufferSize 1024
uint32_t SourceBuffer[SourceBufferSize / sizeof(uint32_t)];
osStaticThreadDef_t SourceControlBlock;
osThreadId OnOffHandle;
#define OnOffBufferSize 1024
uint32_t OnOffBuffer[OnOffBufferSize / sizeof(uint32_t)];
osStaticThreadDef_t OnOffControlBlock;

uint32_t errors_mask = 0;
volatile bool CommandeAmp = false;
volatile AmpState_t EtatAmp = AMP_OFF;
volatile int8_t last_encoder_counter = 0;

volatile bool short_press_pending = false;
volatile bool long_press_pending  = false;

volatile AudioSource_t current_source = SOURCE_USB;

/* Private function prototypes -----------------------------------------------*/
void Events_Thread(void const *argument);
void Volume_Thread(void const *argument);
void Leds_Thread(void const *argument);
void Source_Thread(void const *argument);
void AmpOnOff_Thread(void const *argument);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief  Clear a non-blocking error bit.
 */
void Error_cancel_nonBlocking(errorNbr errorBit_nBr)
{
    if ((errorBit_nBr & errors_mask) != 0)
    {
        LOG_WARN("Error canceled, bit %u", (unsigned)errorBit_nBr);
        errors_mask &= ~(1 << errorBit_nBr);
        if (errors_mask == 0)
        {
            LOG_INFO("Errors cleared");
        }
    }
}

/**
 * @brief  Record a non-blocking error (does not reset the MCU).
 */
void Error_Handler_nonBlocking(char *errorStr, errorNbr errorBit_nBr)
{
    errors_mask |= 1 << errorBit_nBr;
    LOG_ERR("%s (bit %u)", errorStr ? errorStr : "?", (unsigned)errorBit_nBr);
}

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* Reset of all peripherals, init the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Configure the peripherals common clocks */
    PeriphCommonClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();

    Leds_PWM_Init();
    Led_G_SetBrightness(0);
    Led_R_SetBrightness(0);

    MX_I2C1_Init(&DAC_I2C_Handle);
    MX_I2S1_Init();
    MX_TIM3_Init();
    MX_USB_OTG_HS_PCD_Init();
    MX_I2S3_Init();
    MX_TIM4_Init(); // encoder timer
    MX_SPI4_Init();
    MX_USART2_UART_Init();

    SEGGER_RTT_Init();

    LOG_INFO("\n");
    LOG_INFO("");
    LOG_INFO("======= BriXamp boot, sysclk=%lu Hz ==================", (unsigned long)SystemCoreClock);
    LOG_INFO("");
    LOG_INFO("__________        .__ __    ____  ___                    ._.");
    LOG_INFO("\\______   \\_______|__|  | __\\   \\/  /____    _____ ______| |");
    LOG_INFO(" |    |  _/\\_  __ \\  |  |/ / \\     /\\__  \\  /     \\\\____ \\ |");
    LOG_INFO(" |    |   \\ |  | \\/  |    <  /     \\  / _ \\|  Y Y  \\  |_> >|");
    LOG_INFO(" |______  / |__|  |__|__|__|/__/__  \\(____/\\__|_|  /   __/__");
    LOG_INFO("        \\/                        \\_/            \\/|__|   \\/");
    LOG_INFO("");
    LOG_INFO("");

    PGA2311_Init();
    BT_Init();            // power up BT module + arm UART RX (uses HAL_Delay)

    LL_GPIO_SetOutputPin(ANALOG_ON_GPIO_Port, ANALOG_ON_Pin);
    HAL_Delay(100);
    LL_GPIO_ResetOutputPin(DAC_RST_GPIO_Port, DAC_RST_Pin);
    LL_GPIO_ResetOutputPin(MUX_EN_GPIO_Port, MUX_EN_Pin);
    LL_GPIO_ResetOutputPin(MUX_SEL_GPIO_Port, MUX_SEL_Pin);
    ES9038Q2M_ProcessEvents(); // call once to init values, before starting USB
    ES9038Q2M_DAC_Init();      // also done in USB, but in case USB is not started
    MX_USB_DEVICE_Init();
    LOG_INFO("USB device init done");
    LL_TIM_EnableIT_UPDATE(TIM3);
    LL_TIM_EnableCounter(TIM3);

    /* Create the thread(s) */
    osThreadStaticDef(defaultTask, Events_Thread, osPriorityNormal, 0, defaultTaskBufferSize / sizeof(uint32_t), defaultTaskBuffer, &defaultTaskControlBlock);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    osThreadStaticDef(Volume, Volume_Thread, osPriorityNormal, 0, VolumeBufferSize / sizeof(uint32_t), VolumeBuffer, &VolumeControlBlock);
    VolumeHandle = osThreadCreate(osThread(Volume), NULL);

    osThreadStaticDef(Leds, Leds_Thread, osPriorityNormal, 0, LedsBufferSize / sizeof(uint32_t), LedsBuffer, &LedsControlBlock);
    LedsHandle = osThreadCreate(osThread(Leds), NULL);

    osThreadStaticDef(Source, Source_Thread, osPriorityNormal, 0, SourceBufferSize / sizeof(uint32_t), SourceBuffer, &SourceControlBlock);
    SourceHandle = osThreadCreate(osThread(Source), NULL);

    /* OnOff prio higher than Events_Thread to handle I2C mute request */
    osThreadStaticDef(OnOff, AmpOnOff_Thread, osPriorityAboveNormal, 0, OnOffBufferSize / sizeof(uint32_t), OnOffBuffer, &OnOffControlBlock);
    OnOffHandle = osThreadCreate(osThread(OnOff), NULL);

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */
    while (1)
    {
    }
}

void source_leds_off(void)
{
    LL_GPIO_ResetOutputPin(LED4_USB_GPIO_Port, LED4_USB_Pin);
    LL_GPIO_ResetOutputPin(LED1_SPDIF_GPIO_Port, LED1_SPDIF_Pin);
    LL_GPIO_ResetOutputPin(LED2_BT_GPIO_Port, LED2_BT_Pin);
    LL_GPIO_ResetOutputPin(LED3_LINE_GPIO_Port, LED3_LINE_Pin);
}

/* --- LED PWM via TIM9 -------------------------------------------------------
 * PE5 -> TIM9_CH1 (AF3) -> Led_G
 * PE6 -> TIM9_CH2 (AF3) -> Led_R
 *
 * Brightness API takes a perceptual 0..100 %.
 * A gamma curve (gamma = 2.4) maps it to PWM duty so equal % steps
 * look like equal brightness steps to the eye.
 * -------------------------------------------------------------------------- */
#define LEDS_PWM_ARR   999U

/* Precomputed CCR values for 0..100 %, gamma = 2.4, output range 0..1000.
 *   ccr[i] = round( (i/100)^2.4 * 1000 )
 * Python to regenerate:
 *   g = 2.4
 *   [round((i/100)**g * 1000) for i in range(101)]
 */
static const uint16_t leds_gamma_lut[101] = {
       0,    0,    0,    0,    0,    1,    1,    2,    2,    3,
       4,    5,    6,    7,    9,   11,   12,   14,   16,   19,
      21,   24,   26,   29,   33,   36,   39,   43,   47,   51,
      56,   60,   65,   70,   75,   80,   86,   92,   98,  104,
     111,  118,  125,  132,  139,  147,  155,  163,  172,  180,
     189,  199,  208,  218,  228,  238,  249,  259,  271,  282,
     294,  305,  317,  330,  343,  356,  369,  382,  396,  410,
     425,  440,  455,  470,  486,  501,  518,  534,  551,  568,
     585,  603,  621,  639,  658,  677,  696,  716,  736,  756,
     777,  798,  819,  840,  862,  884,  907,  930,  953,  976,
    1000
};

static inline uint32_t leds_percent_to_ccr(uint8_t percent)
{
    if (percent > 100U) percent = 100U;
    return leds_gamma_lut[percent];
}

void Leds_PWM_Init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    LL_TIM_InitTypeDef  TIM_InitStruct  = {0};
    LL_TIM_OC_InitTypeDef OC_InitStruct = {0};

    /* GPIOE clock already enabled in MX_GPIO_Init, but be safe */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM9);

    /* PE5, PE6 as AF3 (TIM9_CH1, TIM9_CH2) */
    GPIO_InitStruct.Pin        = Led_G_Pin | Led_R_Pin;
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate  = LL_GPIO_AF_3;
    LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* Time base: 1 kHz PWM, 1000 steps */
    TIM_InitStruct.Prescaler     = 191;
    TIM_InitStruct.CounterMode   = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload    = LEDS_PWM_ARR;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    LL_TIM_Init(TIM9, &TIM_InitStruct);
    LL_TIM_EnableARRPreload(TIM9);

    /* PWM mode 1, active high, start at 0% */
    OC_InitStruct.OCMode       = LL_TIM_OCMODE_PWM1;
    OC_InitStruct.OCState      = LL_TIM_OCSTATE_ENABLE;
    OC_InitStruct.OCPolarity   = LL_TIM_OCPOLARITY_HIGH;
    OC_InitStruct.CompareValue = 0;

    LL_TIM_OC_Init(TIM9, LL_TIM_CHANNEL_CH1, &OC_InitStruct);
    LL_TIM_OC_EnablePreload(TIM9, LL_TIM_CHANNEL_CH1);

    LL_TIM_OC_Init(TIM9, LL_TIM_CHANNEL_CH2, &OC_InitStruct);
    LL_TIM_OC_EnablePreload(TIM9, LL_TIM_CHANNEL_CH2);

    LL_TIM_GenerateEvent_UPDATE(TIM9);  /* load preloaded registers */
    LL_TIM_EnableCounter(TIM9);
}

void Led_G_SetBrightness(uint8_t percent)
{
    static uint8_t last = 0xFF;
    if (percent > 100U) percent = 100U;
    if (percent == last) return;
    last = percent;
    LL_TIM_OC_SetCompareCH1(TIM9, leds_percent_to_ccr(percent));
}

void Led_R_SetBrightness(uint8_t percent)
{
    static uint8_t last = 0xFF;
    if (percent > 100U) percent = 100U;
    if (percent == last) return;
    last = percent;
    LL_TIM_OC_SetCompareCH2(TIM9, leds_percent_to_ccr(percent));
}

/**
 * @brief  Function implementing the defaultTask thread.
 */
void Events_Thread(void const *argument)
{
    /* LED check at startup */
    LL_GPIO_SetOutputPin(LED4_USB_GPIO_Port, LED4_USB_Pin);
    osDelay(100);
    LL_GPIO_SetOutputPin(LED1_SPDIF_GPIO_Port, LED1_SPDIF_Pin);
    osDelay(100);
    LL_GPIO_SetOutputPin(LED2_BT_GPIO_Port, LED2_BT_Pin);
    osDelay(100);
    LL_GPIO_SetOutputPin(LED3_LINE_GPIO_Port, LED3_LINE_Pin);
    osDelay(300);

    source_leds_off();

    for (;;)
    {
        osDelay(5); // ms
        ES9038Q2M_ProcessEvents();
        BT_Process();                /* drain UART, parse BT responses + events */
    }
}

/**
 * @brief Function implementing the Volume thread.
 */
void Volume_Thread(void const *argument)
{
    for (;;)
    {
        if (EtatAmp == AMP_ON)
        {
            int8_t counter = __HAL_TIM_GET_COUNTER(&htim4);
            if (counter != last_encoder_counter)
            {
                int8_t delta = (int8_t)(counter - last_encoder_counter);
                last_encoder_counter = counter;
                LOG_DBG("encoder %d, delta %d", counter, delta);

                if (current_source == SOURCE_BT)
                {
                    /* Phone handles the volume; leftover UP steps (phone at 15)
                     * extend the range upward via the DAC. Down is all phone. */
                    delta = BT_VolumeChange(delta);
                }

                ES9038Q2M_DAC_Volume_change(delta);  /* USB/SPDIF/LINE */
            }
        }
        osDelay(100);
    }
}

/**
 * @brief Function implementing the Leds thread.
 */
void Leds_Thread(void const *argument)
{
    for (;;)
    {
        osDelay(20);
    }
}

/* --- Audio source selection ------------------------------------------------
 * Cycle order: USB -> SPDIF -> BT -> LINE -> USB ...
 * -------------------------------------------------------------------------- */

const char *Spdif_GetInputTypeStr(void)
{
    return (LL_GPIO_IsOutputPinSet(SEL_SPDIF_GPIO_Port, SEL_SPDIF_Pin) == SPDIF_INPUT_COAX)
               ? "coax" : "optical";
}

/* --- SPDIF active probe -----------------------------------------------------
 * Switch the DAC to its SPDIF input and look for a real stream: 500 ms on
 * each physical input. Probing starts with the input that last carried a
 * stream (optical at boot), so a known-good source is rechecked first.
 * Returns true as soon as a stream is detected, leaving SEL_SPDIF on the
 * input where it was found. */
static bool Spdif_ProbeInput(uint8_t input)
{
    if (input == SPDIF_INPUT_COAX)
        LL_GPIO_SetOutputPin(SEL_SPDIF_GPIO_Port, SEL_SPDIF_Pin);
    else
        LL_GPIO_ResetOutputPin(SEL_SPDIF_GPIO_Port, SEL_SPDIF_Pin);

    LOG_DBG("SPDIF probe: %s (500ms)",
            (input == SPDIF_INPUT_COAX) ? "coax" : "optical");
    osDelay(500);

    if (ES9038Q2M_SpdifPresent())
    {
        LOG_INFO("SPDIF stream found on %s",
                 (input == SPDIF_INPUT_COAX) ? "coax" : "optical");
        return true;
    }
    return false;
}

static bool Spdif_Probe(void)
{
    /* Remember which physical input last carried a stream so we recheck the
     * known-good one first. Defaults to optical at boot. */
    static uint8_t last_good_input = SPDIF_INPUT_OPTICAL;

    ES9038Q2M_DAC_SetInput(ES9038Q2M_INPUT_SPDIF);

    uint8_t other_input = (last_good_input == SPDIF_INPUT_COAX)
                              ? SPDIF_INPUT_OPTICAL
                              : SPDIF_INPUT_COAX;

    if (Spdif_ProbeInput(last_good_input))
        return true;

    if (Spdif_ProbeInput(other_input))
    {
        last_good_input = other_input;
        return true;
    }

    LOG_DBG("SPDIF probe: no stream found");
    return false;
}

/* Mute the whole audio path unconditionally (digital DAC + analog PGA).
 * Mute is always safe, so unlike Source_Unmute() it is NOT gated on amp state
 * or current source: both paths are silenced every time. */
static void Source_Mute(void)
{
    ES9038Q2M_DAC_SetMute_Force(true);   /* mute DAC path    */
    PGA2311_Mute(true);                  /* mute PGA (PGA_M) */

    if (current_source == SOURCE_BT)
        BT_Pause();                      /* pause phone (no-op if not connected) */
    
    // wait for volume ramp down to finish.
    // Linked with volume_rate in ES9038Q2M_REG6_ADDR
    // Atvolume_rate == 0, if FSR = 48k, ramp rate = 93 dB/s, so 0.68s for -64dB
    osDelay(500); // 500ms enough
}

/* Unmute the audio path for the given source, but only once the amp is fully
 * ON. While AMP_POWERING/OFF this is a no-op (deferred), so audio never comes
 * up before the power sequence completes.
 *   - LINE  : analog path -> PGA2311
 *   - other : digital path -> DAC (SetMute_Force is itself AMP_ON-gated) */
static void Source_Unmute(AudioSource_t src)
{
    if (EtatAmp != AMP_ON)
    {
        LOG_DBG("Source_Unmute(%s) while amp not ON, deferred", sources[src].name);
        return;
    }

    if (src == SOURCE_BT)
    {
        BT_VolumeInit(BT_SPKVOL_START);   /* phone to mid (7), keep current DAC */
        BT_Play();
        osDelay(500); // wait for play to apply
    }

    if (src == SOURCE_LINE)
        PGA2311_Mute(false);                 /* un-mute PGA (PGA_M) */
    else
        ES9038Q2M_DAC_SetMute_Force(false);  /* un-mute DAC */

}

/* Could return immediately if the source is already active.
 * Must wait up to 1 second before returning false. */
static bool Source_IsAvailable(AudioSource_t src)
{
    switch (src)
    {
    case SOURCE_USB:
        /* "connected" = enumerated and configured by the host */
        if (hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED)
            return true;
        osDelay(1000);                      /* give it a second to enumerate */
        return (hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED);

    case SOURCE_SPDIF:
        /* active probe: 500 ms optical, then 500 ms coax (~1 s total) */
        return Spdif_Probe();

    case SOURCE_BT:
        if (BT_IsConnected())
            return true;
        osDelay(500); 
        if (BT_IsConnected())
            return true;
        osDelay(500);                      /* give it a second to connect */
        return BT_IsConnected();

    case SOURCE_LINE:
    default:
        return true;                        /* always available */
    }
}

/* Try to find a source in one loop starting by the given one.
 * LINE is always available, so the loop always terminates there at worst. */
static AudioSource_t Source_Search(AudioSource_t start_source)
{
    for (uint8_t i = 0; i < SOURCE_COUNT; i++)
    {
        AudioSource_t candidate = (AudioSource_t)((start_source + i) % SOURCE_COUNT);
        LL_GPIO_SetOutputPin(sources[candidate].GPIOx, sources[candidate].PinMask);
        if (Source_IsAvailable(candidate))      /* per-source check + timing */
            return candidate;
        LL_GPIO_ResetOutputPin(sources[candidate].GPIOx, sources[candidate].PinMask);
    }
    return start_source;   /* should not happen: LINE is always available */
}

static void Source_Switch(AudioSource_t start_source)
{
    source_leds_off();

    Source_Mute();

    AudioSource_t new_source = Source_Search(start_source);

    if (new_source == current_source && EtatAmp == AMP_ON)
    {
        LOG_INFO("source == %s, no other source available", sources[current_source].name);
        Source_Unmute(current_source);
        return;
    }

    LL_GPIO_ResetOutputPin(RELAY_ON_GPIO_Port, RELAY_ON_Pin); /* open LINE relay  */
    LL_GPIO_ResetOutputPin(MUX_SEL_GPIO_Port, MUX_SEL_Pin);   /* mux -> I2S3_SD (USB) path */
    /* MUX_EN is active-low and left enabled (low) at all times */
    osDelay(50);

    switch (new_source) // Unmute is deferred until AMP_ON via Source_Unmute()
    {
        case SOURCE_USB:
            ES9038Q2M_DAC_SetInput(ES9038Q2M_INPUT_I2S);      /* USB via I2S */
            break;

        case SOURCE_SPDIF:
            /* SEL_SPDIF already on the working input (from Spdif_Probe) */
            ES9038Q2M_DAC_SetInput(ES9038Q2M_INPUT_SPDIF);
            break;

        case SOURCE_BT:
            ES9038Q2M_DAC_SetInput(ES9038Q2M_INPUT_I2S);
            LL_GPIO_SetOutputPin(MUX_SEL_GPIO_Port, MUX_SEL_Pin);
            break;

        case SOURCE_LINE:
            /* analog LINE path: close input relay, settle. Unmute is deferred. */
            LL_GPIO_SetOutputPin(RELAY_ON_GPIO_Port, RELAY_ON_Pin);
            osDelay(50);
            break;

        default:
            LOG_ERR("invalid source");
            return;
    }

    Source_Unmute(new_source);   /* no-op unless AMP_ON */

    current_source = new_source;
    LOG_INFO("source -> %s", sources[new_source].name);
    LL_GPIO_SetOutputPin(sources[new_source].GPIOx, sources[new_source].PinMask);
}

/**
 * @brief Function implementing the Source thread.
 */
void Source_Thread(void const *argument)
{
    /* Module readiness wait is handled inside BT_QueryInfo (tick-based). */
    BT_QueryInfo();

    for (;;)
    {
        osDelay(50);

        PowerButton_Process();

        if (EtatAmp != AMP_ON)
            continue;

        if (short_press_pending && EtatAmp == AMP_ON)
        {
            short_press_pending = false;
            Source_Switch(current_source + 1);
        }
    }
}

/**
 * @brief Function implementing the OnOff thread.
 */
void AmpOnOff_Thread(void const *argument)
{
    for (;;)
    {
        osDelay(100);

        /* From OFF: any press (short or long) powers ON */
        if (EtatAmp == AMP_OFF)
        {
            if (short_press_pending || long_press_pending)
            {
                short_press_pending = false;
                long_press_pending  = false;
                CommandeAmp = true;
            }
        }
        else
        {
            /* Running: long press powers OFF */
            if (long_press_pending)
            {
                long_press_pending = false;
                CommandeAmp = false;
            }
            /* short press while ON is handled by Source_Thread (source cycle) */
        }

        if (CommandeAmp && EtatAmp == AMP_OFF)
        {
            LOG_WARN("amp power START sequence");
            EtatAmp = AMP_POWERING;

            /* 1) Start the amp power-on immediately */
            LL_GPIO_SetOutputPin(Light_fire_L_GPIO_Port, Light_fire_L_Pin);
            LL_GPIO_SetOutputPin(Light_fire_R_GPIO_Port, Light_fire_R_Pin);
            osDelay(100);
            LL_GPIO_SetOutputPin(On_L_GPIO_Port, On_L_Pin);
            LL_GPIO_SetOutputPin(On_R_GPIO_Port, On_R_Pin);
            uint32_t power_on_tick = HAL_GetTick();
            LOG_INFO("amp power START, wait for power to stabilize");

            /* 2) Select source while the power rails stabilize (may take ~3s) */
            Source_Switch(current_source);

            __HAL_TIM_SET_COUNTER(&htim4, 0);
            last_encoder_counter = 0;
            HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

            /* 3) Wait for the remaining of the 5s stabilization timeout
             *    (source switch delay is now part of this window) */
            uint32_t elapsed = HAL_GetTick() - power_on_tick;
            if (elapsed < 5000)
            {
                LOG_DBG("power stabilize: %lu ms elapsed, waiting %lu ms more",
                        (unsigned long)elapsed, (unsigned long)(5000 - elapsed));
                osDelay(5000 - elapsed);
            }
            else
            {
                LOG_DBG("power stabilize: source switch took %lu ms (>=5s), no extra wait",
                        (unsigned long)elapsed);
            }

            LL_GPIO_ResetOutputPin(Light_fire_L_GPIO_Port, Light_fire_L_Pin);
            osDelay(100);
            LL_GPIO_ResetOutputPin(Light_fire_R_GPIO_Port, Light_fire_R_Pin);

            EtatAmp = AMP_ON;

            Source_Unmute(current_source);   /* unmute correct path (DAC or PGA) */
            LOG_INFO("amp power ON, unmuted, let's rock...");
        }
        if (!CommandeAmp && EtatAmp != AMP_OFF)
        {
            LOG_WARN("amp power OFF start sequence");
            EtatAmp = AMP_OFF;
            Source_Mute();
            source_leds_off();
            HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);
            LL_GPIO_SetOutputPin(Light_fire_L_GPIO_Port, Light_fire_L_Pin);
            LL_GPIO_SetOutputPin(Light_fire_R_GPIO_Port, Light_fire_R_Pin);
            osDelay(100);
            LL_GPIO_ResetOutputPin(On_L_GPIO_Port, On_L_Pin);
            LL_GPIO_ResetOutputPin(On_R_GPIO_Port, On_R_Pin);
            osDelay(100);
            LL_GPIO_ResetOutputPin(Light_fire_L_GPIO_Port, Light_fire_L_Pin);
            LL_GPIO_ResetOutputPin(Light_fire_R_GPIO_Port, Light_fire_R_Pin);
            LOG_INFO("amp OFF, cool down.");
        }
    }
}

/**
 * @brief  Period elapsed callback in non-blocking mode (HAL time base on TIM6).
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        HAL_IncTick();
    }
}

/**
 * @brief  This function is executed in case of error occurrence.
 */
void Error_Handler_str(const char *where)
{
    LOG_ERR("Error_Handler from %s", where ? where : "?");
    LL_GPIO_SetOutputPin(LED3_LINE_GPIO_Port, LED3_LINE_Pin);

    /* Give RTT a moment to drain before any further action. */
    for (volatile uint32_t i = 0; i < 100000; i++)
    {
        __NOP();
    }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and line where assert_param failed.
 */
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
