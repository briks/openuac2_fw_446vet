/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usb_device.h"
#include "usbd_conf.h"
#include "es9038q2m.h"
#include "pga2311.h"
#include "SEGGER_RTT.h"
#include "stm32f4xx_it.h"
#define LOG_LEVEL LOG_LEVEL_DBG
#include "log.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef DAC_I2C_Handle; // hi2c1

I2S_HandleTypeDef hi2s1;
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi3_tx;

SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

PCD_HandleTypeDef hpcd_USB_OTG_HS;

DMA_HandleTypeDef hdma_memtomem_dma2_stream0;
DMA_HandleTypeDef hdma_memtomem_dma2_stream1;
osThreadId defaultTaskHandle;
#define defaultTaskBufferSize 2048
uint32_t defaultTaskBuffer[defaultTaskBufferSize / sizeof(uint32_t)];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId VolumeHandle;
#define VolumeBufferSize 512
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

/* USER CODE BEGIN PV */
uint32_t errors_mask = 0;
volatile bool CommandeAmp = false;
volatile AmpState_t EtatAmp = AMP_OFF;
volatile int8_t last_encoder_counter = 0;

volatile bool short_press_pending = false;
volatile bool long_press_pending  = false;

volatile AudioSource_t current_source = SOURCE_USB;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USB_OTG_HS_PCD_Init(void);
static void MX_I2S3_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI4_Init(void);
static void MX_USART2_UART_Init(void);
void Events_Thread(void const *argument);
void StartVolume(void const *argument);
void Leds_Thread(void const *argument);
void Source_Thread(void const *argument);
void AmpOnOff_Thread(void const *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  This function is executed in case of error occurrence, but do not reset.
 * @retval None
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
 * @brief  This function is executed in case of error occurrence, but do not reset.
 * @retval None
 */
void Error_Handler_nonBlocking(char *errorStr, errorNbr errorBit_nBr)
{
    errors_mask |= 1 << errorBit_nBr;
    LOG_ERR("%s (bit %u)", errorStr ? errorStr : "?", (unsigned)errorBit_nBr);
    Error_Handler();
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* Configure the peripherals common clocks */
    PeriphCommonClock_Config();

    /* USER CODE BEGIN SysInit */
    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    /* USER CODE BEGIN after MX_GPIO_Init */
    Leds_PWM_Init();
    Led_G_SetBrightness(0);
    Led_R_SetBrightness(0);
    /* USER CODE END after MX_GPIO_Init */
    MX_I2C1_Init(&DAC_I2C_Handle);
    MX_I2S1_Init();
    MX_TIM3_Init();
    MX_USB_OTG_HS_PCD_Init();
    MX_I2S3_Init();
    MX_TIM4_Init(); // encoder timer
    MX_SPI4_Init();
    MX_USART2_UART_Init();
    /* USER CODE BEGIN 2 */
    SEGGER_RTT_Init();
    PGA2311_Init();

    LOG_INFO("\n");
    LOG_INFO("");
    LOG_INFO("======= BriXamp boot, sysclk=%lu Hz ==================", (unsigned long)SystemCoreClock);
    LOG_INFO("");
    LOG_INFO("__________        .__ __    ____  ___                    ._.");
    LOG_INFO("\\______   \\_______|__|  | __\\   \\/  /____    _____ ______| |");
    LOG_INFO(" |    |  _/\\_  __ \\  |  |/ / \\     /\\__  \\  /     \\\\____ \\ |");
    LOG_INFO(" |    |   \\ |  | \\/  |    <  /     \\ / __ \\|  Y Y  \\  |_> >|");
    LOG_INFO(" |______  / |__|  |__|__|__|_ \\/___/\\  (____  /__|_|  /   __/__");
    LOG_INFO("        \\/                \\/      \\_/    \\/      \\/|__|   \\/");
    LOG_INFO("");
    LOG_INFO("");
    LL_GPIO_SetOutputPin(ANALOG_ON_GPIO_Port, ANALOG_ON_Pin);
    HAL_Delay(100);
    LL_GPIO_ResetOutputPin(PDN_GPIO_Port, PDN_Pin);
    LL_GPIO_ResetOutputPin(MUX_EN_GPIO_Port, MUX_EN_Pin);
    LL_GPIO_ResetOutputPin(MUX_SEL_GPIO_Port, MUX_SEL_Pin);
    ES9038Q2M_ProcessEvents(); // Call it one time to init values, before starting usb.
    MX_USB_DEVICE_Init();
    LOG_INFO("USB device init done");
    LL_TIM_EnableIT_UPDATE(TIM3);
    LL_TIM_EnableCounter(TIM3);

    /* USER CODE END 2 */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* definition and creation of defaultTask  */
    osThreadStaticDef(defaultTask, Events_Thread, osPriorityNormal, 0, defaultTaskBufferSize / sizeof(uint32_t), defaultTaskBuffer, &defaultTaskControlBlock);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    /* definition and creation of Volume */
    osThreadStaticDef(Volume, StartVolume, osPriorityNormal, 0, VolumeBufferSize / sizeof(uint32_t), VolumeBuffer, &VolumeControlBlock);
    VolumeHandle = osThreadCreate(osThread(Volume), NULL);

    /* definition and creation of Leds */
    osThreadStaticDef(Leds, Leds_Thread, osPriorityNormal, 0, LedsBufferSize / sizeof(uint32_t), LedsBuffer, &LedsControlBlock);
    LedsHandle = osThreadCreate(osThread(Leds), NULL);

    /* definition and creation of Source */
    osThreadStaticDef(Source, Source_Thread, osPriorityNormal, 0, SourceBufferSize / sizeof(uint32_t), SourceBuffer, &SourceControlBlock);
    SourceHandle = osThreadCreate(osThread(Source), NULL);

    /* definition and creation of OnOff, prio higher than Events_Thread source to handle I2C mute request */
    osThreadStaticDef(OnOff, AmpOnOff_Thread, osPriorityAboveNormal, 0, OnOffBufferSize / sizeof(uint32_t), OnOffBuffer, &OnOffControlBlock);
    OnOffHandle = osThreadCreate(osThread(OnOff), NULL);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_5)
    {
    }
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
    LL_PWR_EnableOverDriveMode();
    LL_RCC_HSE_Enable();

    /* Wait till HSE is ready */
    while (LL_RCC_HSE_IsReady() != 1)
    {
    }
    // LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_16, 250, LL_RCC_PLLP_DIV_4);
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_16, 250, LL_RCC_PLLR_DIV_2);
    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLP, LL_RCC_PLLP_DIV_4); // replace uneffective line above
    LL_RCC_PLL_Enable();

    /* Wait till PLL is ready */
    while (LL_RCC_PLL_IsReady() != 1)
    {
    }
    while (LL_PWR_IsActiveFlag_VOS() == 0)
    {
    }
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLLR);

    /* Wait till System clock is ready */
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLLR)
    {
    }
    LL_SetSystemCoreClock(192000000);

    /* Update the time base */
    if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK)
    {
        Error_Handler();
    }
    LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_PLLCLK, LL_RCC_MCO1_DIV_4);
    LL_RCC_ConfigMCO(LL_RCC_MCO2SOURCE_PLLI2S, LL_RCC_MCO2_DIV_1);
    LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void)
{
    LL_RCC_PLLI2S_ConfigDomain_I2S(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLI2SM_DIV_16, 128, LL_RCC_PLLI2SR_DIV_2);
    LL_RCC_PLLI2S_Enable();

    /* Wait till PLL is ready */
    while (LL_RCC_PLLI2S_IsReady() != 1)
    {
    }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
void MX_I2C1_Init(I2C_HandleTypeDef *hi2c)
{

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c->Instance = I2C1;
    hi2c->Init.ClockSpeed = 100000;
    hi2c->Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c->Init.OwnAddress1 = 0;
    hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c->Init.OwnAddress2 = 0;
    hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(hi2c) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief I2S1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2S1_Init(void)
{

    /* USER CODE BEGIN I2S1_Init 0 */

    /* USER CODE END I2S1_Init 0 */

    /* USER CODE BEGIN I2S1_Init 1 */

    /* USER CODE END I2S1_Init 1 */
    hi2s1.Instance = SPI1;
    hi2s1.Init.Mode = I2S_MODE_SLAVE_TX;
    hi2s1.Init.Standard = I2S_STANDARD_PHILIPS;
    hi2s1.Init.DataFormat = I2S_DATAFORMAT_32B;
    hi2s1.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
    hi2s1.Init.AudioFreq = I2S_AUDIOFREQ_48K;
    hi2s1.Init.CPOL = I2S_CPOL_LOW;
    hi2s1.Init.ClockSource = I2S_CLOCK_PLL;
    hi2s1.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
    if (HAL_I2S_Init(&hi2s1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2S1_Init 2 */

    /* USER CODE END I2S1_Init 2 */
}

/**
 * @brief I2S3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2S3_Init(void)
{

    /* USER CODE BEGIN I2S3_Init 0 */

    /* USER CODE END I2S3_Init 0 */

    /* USER CODE BEGIN I2S3_Init 1 */

    /* USER CODE END I2S3_Init 1 */
    hi2s3.Instance = SPI3;
    hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
    hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
    hi2s3.Init.DataFormat = I2S_DATAFORMAT_32B;
    hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
    hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
    hi2s3.Init.CPOL = I2S_CPOL_LOW;
    hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
    hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
    if (HAL_I2S_Init(&hi2s3) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2S3_Init 2 */

    /* USER CODE END I2S3_Init 2 */
}

/**
 * @brief SPI4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI4_Init(void)
{

    /* USER CODE BEGIN SPI4_Init 0 */

    /* USER CODE END SPI4_Init 0 */

    /* USER CODE BEGIN SPI4_Init 1 */

    /* USER CODE END SPI4_Init 1 */
    /* SPI4 parameter configuration*/
    hspi4.Instance = SPI4;
    hspi4.Init.Mode = SPI_MODE_MASTER;
    hspi4.Init.Direction = SPI_DIRECTION_1LINE;
    hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi4.Init.NSS = SPI_NSS_SOFT;
    hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi4.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi4) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI4_Init 2 */

    /* USER CODE END SPI4_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

    /* USER CODE BEGIN TIM3_Init 0 */

    // Driven by I2S WS (word select) output, as an input on PD2 as ETR (external trigger)
    // Update event each 32 times

    /* USER CODE END TIM3_Init 0 */

    LL_TIM_InitTypeDef TIM_InitStruct = {0};

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
    /**TIM3 GPIO Configuration
    PD2   ------> TIM3_ETR
    */
    GPIO_InitStruct.Pin = TIM3_ETR_AUDIO_SYNC_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(TIM3_ETR_AUDIO_SYNC_GPIO_Port, &GPIO_InitStruct);

    /* TIM3 interrupt Init */
    NVIC_SetPriority(TIM3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(TIM3_IRQn);

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    TIM_InitStruct.Prescaler = 0;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 31;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    LL_TIM_Init(TIM3, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(TIM3);
    LL_TIM_SetTriggerInput(TIM3, LL_TIM_TS_ETRF);
    LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_EXT_MODE1);
    LL_TIM_DisableExternalClock(TIM3);
    LL_TIM_ConfigETR(TIM3, LL_TIM_ETR_POLARITY_NONINVERTED, LL_TIM_ETR_PRESCALER_DIV1, LL_TIM_ETR_FILTER_FDIV1);
    LL_TIM_DisableIT_TRIG(TIM3);
    LL_TIM_DisableDMAReq_TRIG(TIM3);
    LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
    LL_TIM_DisableMasterSlaveMode(TIM3);
    /* USER CODE BEGIN TIM3_Init 2 */

    /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief TIM4 Initialization Function for encoder
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

    /* USER CODE BEGIN TIM4_Init 0 */

    /* USER CODE END TIM4_Init 0 */

    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM4_Init 1 */
    // Rotary_Encoder_Switch
    /* USER CODE END TIM4_Init 1 */
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 1;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 255;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 3;
    sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 3;
    if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK) // LL_TIM_ENCODER_Init
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM4_Init 2 */

    /* USER CODE END TIM4_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */
}

/**
 * @brief USB_OTG_HS Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_OTG_HS_PCD_Init(void)
{

    /* USER CODE BEGIN USB_OTG_HS_Init 0 */

    /* USER CODE END USB_OTG_HS_Init 0 */

    /* USER CODE BEGIN USB_OTG_HS_Init 1 */

    /* USER CODE END USB_OTG_HS_Init 1 */
    hpcd_USB_OTG_HS.Instance = USB_OTG_HS;
    hpcd_USB_OTG_HS.Init.dev_endpoints = 8;
    hpcd_USB_OTG_HS.Init.speed = PCD_SPEED_HIGH;
    hpcd_USB_OTG_HS.Init.dma_enable = ENABLE;
    hpcd_USB_OTG_HS.Init.phy_itface = USB_OTG_ULPI_PHY;
    hpcd_USB_OTG_HS.Init.Sof_enable = DISABLE;
    hpcd_USB_OTG_HS.Init.low_power_enable = DISABLE;
    hpcd_USB_OTG_HS.Init.lpm_enable = ENABLE;
    hpcd_USB_OTG_HS.Init.vbus_sensing_enable = DISABLE;
    hpcd_USB_OTG_HS.Init.use_dedicated_ep1 = DISABLE;
    hpcd_USB_OTG_HS.Init.use_external_vbus = DISABLE;
    if (HAL_PCD_Init(&hpcd_USB_OTG_HS) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USB_OTG_HS_Init 2 */
    /* Total fido should be <=4096, so <=1024 words, rx + tx ? */
    HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_HS, 0x200);
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_HS, 0, 0x40);
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_HS, 1, 0x174); /* streaming feedback */
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_HS, 2, 0x20);  /* interrupt status, 6B */
                                                     /* USER CODE END USB_OTG_HS_Init 2 */
}

/**
 * Enable DMA controller clock
 * Configure DMA for memory to memory transfers
 *   hdma_memtomem_dma2_stream0
 *   hdma_memtomem_dma2_stream1
 */
static void MX_DMA_Init(void)
{

    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* Configure DMA request hdma_memtomem_dma2_stream0 on DMA2_Stream0 */
    hdma_memtomem_dma2_stream0.Instance = DMA2_Stream0;
    hdma_memtomem_dma2_stream0.Init.Channel = DMA_CHANNEL_0;
    hdma_memtomem_dma2_stream0.Init.Direction = DMA_MEMORY_TO_MEMORY;
    hdma_memtomem_dma2_stream0.Init.PeriphInc = DMA_PINC_ENABLE;
    hdma_memtomem_dma2_stream0.Init.MemInc = DMA_MINC_ENABLE;
    hdma_memtomem_dma2_stream0.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_memtomem_dma2_stream0.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_memtomem_dma2_stream0.Init.Mode = DMA_NORMAL;
    hdma_memtomem_dma2_stream0.Init.Priority = DMA_PRIORITY_LOW;
    hdma_memtomem_dma2_stream0.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_memtomem_dma2_stream0.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_memtomem_dma2_stream0.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_memtomem_dma2_stream0.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_memtomem_dma2_stream0) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure DMA request hdma_memtomem_dma2_stream1 on DMA2_Stream1 */
    hdma_memtomem_dma2_stream1.Instance = DMA2_Stream1;
    hdma_memtomem_dma2_stream1.Init.Channel = DMA_CHANNEL_0;
    hdma_memtomem_dma2_stream1.Init.Direction = DMA_MEMORY_TO_MEMORY;
    hdma_memtomem_dma2_stream1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_memtomem_dma2_stream1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_memtomem_dma2_stream1.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_memtomem_dma2_stream1.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_memtomem_dma2_stream1.Init.Mode = DMA_NORMAL;
    hdma_memtomem_dma2_stream1.Init.Priority = DMA_PRIORITY_LOW;
    hdma_memtomem_dma2_stream1.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_memtomem_dma2_stream1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_memtomem_dma2_stream1.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_memtomem_dma2_stream1.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_memtomem_dma2_stream1) != HAL_OK)
    {
        Error_Handler();
    }

    /* DMA interrupt init */
    /* DMA1_Stream5_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    /* DMA2_Stream0_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    /* DMA2_Stream3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);

    /**/
    LL_GPIO_ResetOutputPin(GPIOE, ANALOG_ON_Pin | Light_fire_R_Pin | Light_fire_L_Pin | Led_G_Pin | Led_R_Pin | PGA_M_Pin | SPI4_CS_Pin);

    /**/
    LL_GPIO_ResetOutputPin(GPIOC, RELAY_ON_Pin | On_L_Pin | On_R_Pin | SEL_SPDIF_Pin | DSDOE_Pin);

    /**/
    LL_GPIO_ResetOutputPin(GPIOB, PDN_Pin | LED1_SPDIF_Pin | BT_PWR_Pin);

    /**/
    LL_GPIO_ResetOutputPin(GPIOD, LED2_BT_Pin | LED3_LINE_Pin | LED4_USB_Pin | BT_RST_Pin);

    /**/
    LL_GPIO_ResetOutputPin(GPIOA, MUX_EN_Pin | MUX_SEL_Pin);

    /**/
    GPIO_InitStruct.Pin = ANALOG_ON_Pin | Light_fire_R_Pin | Light_fire_L_Pin | PGA_M_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = RELAY_ON_Pin | On_L_Pin | On_R_Pin | SEL_SPDIF_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = SPI4_CS_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(SPI4_CS_GPIO_Port, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = PDN_Pin | LED1_SPDIF_Pin | BT_PWR_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = LED2_BT_Pin | LED3_LINE_Pin | LED4_USB_Pin | BT_RST_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = MUX_EN_Pin | MUX_SEL_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = DSDOE_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(DSDOE_GPIO_Port, &GPIO_InitStruct);

    /**/
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTD, LL_SYSCFG_EXTI_LINE11);

    /**/
    EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_11;
    EXTI_InitStruct.LineCommand = ENABLE;
    EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
    EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
    LL_EXTI_Init(&EXTI_InitStruct);

    /**/
    LL_GPIO_SetPinPull(EXT_INT_ENCODER_GPIO_Port, EXT_INT_ENCODER_Pin, LL_GPIO_PULL_NO);

    /**/
    LL_GPIO_SetPinMode(EXT_INT_ENCODER_GPIO_Port, EXT_INT_ENCODER_Pin, LL_GPIO_MODE_INPUT);

    /* EXTI interrupt init*/
    NVIC_SetPriority(EXTI15_10_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(EXTI15_10_IRQn);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
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

/* USER CODE END 4 */

/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
void Events_Thread(void const *argument)
{
    /* USER CODE BEGIN 5 */

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

    /* Infinite loop */
    for (;;)
    {
        ES9038Q2M_ProcessEvents();
    }
    /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartVolume */
/**
 * @brief Function implementing the Volume thread.
 * @param argument: Not used
 * @retval None
 */
void StartVolume(void const *argument)
{
    /* Infinite loop */
    for (;;)
    {
        if (EtatAmp == AMP_ON)
        {
            int8_t counter = __HAL_TIM_GET_COUNTER(&htim4);
            if (counter != last_encoder_counter)
            {
                int8_t delta = (int8_t)(counter - last_encoder_counter); // handle overflow with signed int
                last_encoder_counter = counter;
                LOG_DBG("encoder %d, delta %d", counter, delta);
                ES9038Q2M_DAC_Volume_change(delta);
            }
        }
        osDelay(100);
    }
}

/**
 * @brief Function implementing the Leds thread.
 * @param argument: Not used
 * @retval None
 */
void Leds_Thread(void const *argument)
{
    /* Infinite loop */
    for (;;)
    {
        osDelay(20);
    }
}

/* --- Audio source selection ------------------------------------------------
 * Cycle order: USB -> SPDIF -> BT -> LINE -> USB ...
 * -------------------------------------------------------------------------- */

static bool Source_IsAvailable(AudioSource_t src)
{
    switch (src)
    {
    case SOURCE_USB:
        /* "connected" = enumerated and configured by the host */
        return (hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED);

    case SOURCE_SPDIF:
    case SOURCE_BT:
    case SOURCE_LINE:
    default:
        return true;   /* detection not implemented yet */
    }
}


/* Try to find a source in one loop starting by the given one */
static AudioSource_t Source_Search(AudioSource_t start_source)
{
    for (uint8_t i = 0; i < SOURCE_COUNT; i++)
    {
        AudioSource_t candidate = (AudioSource_t)((start_source + i) % SOURCE_COUNT);
        LL_GPIO_SetOutputPin(sources[candidate].GPIOx, sources[candidate].PinMask);
        if (Source_IsAvailable(candidate))
            return candidate;
        osDelay(500);
        LL_GPIO_ResetOutputPin(sources[candidate].GPIOx, sources[candidate].PinMask);
    }
    return start_source;   /* none else available, keep current */
}

static void Source_Switch(AudioSource_t start_source)
{
    source_leds_off();

    AudioSource_t new_source = Source_Search(start_source);

    if (new_source == current_source &&  EtatAmp == AMP_ON)
    {
        LOG_INFO("source == %s, no other source available", sources[current_source].name);
        return;
    }

    /* Source is changing: mute everything (digital + analog), let it settle,
     * then bring up only the new path below. */
    ES9038Q2M_DAC_SetMute_Force(true);                       /* mute DAC path  */
    PGA2311_Mute(true);                                      /* mute PGA (PGA_M) */
    LL_GPIO_ResetOutputPin(RELAY_ON_GPIO_Port, RELAY_ON_Pin);/* open LINE relay */
    LL_GPIO_ResetOutputPin(MUX_SEL_GPIO_Port, MUX_SEL_Pin);  /* mux -> I2S3_SD (USB) path */
    LL_GPIO_ResetOutputPin(SEL_SPDIF_GPIO_Port, SEL_SPDIF_Pin);/* disable SPDIF input HW */
    ES9038Q2M_DAC_SetInput(ES9038Q2M_INPUT_I2S);             /* default DAC input = I2S */
    /* MUX_EN is active-low and left enabled (low) at all times */
    osDelay(50);

    switch (new_source)
    {
        case SOURCE_USB:
            /* mux already on I2S3_SD (USB) path, DAC on I2S */
            ES9038Q2M_DAC_SetMute_Force(false);              /* un-mute DAC */
            break;

        case SOURCE_SPDIF:
            /* enable SPDIF input HW, switch DAC to its SPDIF input (DAC GPIO1) */
            LL_GPIO_SetOutputPin(SEL_SPDIF_GPIO_Port, SEL_SPDIF_Pin);
            ES9038Q2M_DAC_SetInput(ES9038Q2M_INPUT_SPDIF);
            ES9038Q2M_DAC_SetMute_Force(false);              /* un-mute DAC */
            break;

        case SOURCE_BT:
            /* BT path: switch analog/I2S mux from I2S3_SD to BT, DAC stays on I2S */
            LL_GPIO_SetOutputPin(MUX_SEL_GPIO_Port, MUX_SEL_Pin);
            ES9038Q2M_DAC_SetMute_Force(false);              /* un-mute DAC */
            break;

        case SOURCE_LINE:
            /* analog LINE path: close input relay, settle, then unmute PGA.
             * PGA2311 gain is already kept in sync by ES9038Q2M_ProcessEvents. */
            LL_GPIO_SetOutputPin(RELAY_ON_GPIO_Port, RELAY_ON_Pin);
            osDelay(50);
            PGA2311_Mute(false);                             /* un-mute PGA (PGA_M) */
            break;

        default:
            LOG_ERR("invalid source");
            return;
    }

    current_source = new_source;
    LOG_INFO("source -> %s", sources[new_source].name);
    LL_GPIO_SetOutputPin(sources[new_source].GPIOx, sources[new_source].PinMask);
}

/**
 * @brief Function implementing the Source thread.
 * @param argument: Not used
 * @retval None
 */
void Source_Thread(void const *argument)
{
    for (;;)
    {
        osDelay(50);

        PowerButton_Process();

        if (EtatAmp != AMP_ON)
            continue;

        /* Short press while ON: cycle to next available source */
        if (short_press_pending && EtatAmp == AMP_ON)
        {
            short_press_pending = false;
            Source_Switch(current_source + 1);
        }
    }
}

/**
 * @brief Function implementing the OnOff thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartOnOff */
void AmpOnOff_Thread(void const *argument)
{
    /* Infinite loop */
    for (;;)
    {
        osDelay(100);

        /* Long press -> power OFF (from any state) */
        if (long_press_pending)
        {
            long_press_pending = false;
            CommandeAmp = false;
        }

        /* Short press while OFF -> power ON */
        if (short_press_pending && EtatAmp == AMP_OFF)
        {
            short_press_pending = false;
            CommandeAmp = true;
        }

        if (CommandeAmp && EtatAmp == AMP_OFF)
        {
            LOG_WARN("amp power START sequence");
            EtatAmp = AMP_POWERING;

            /* select default source (USB if connected, else next available) */
            Source_Switch(current_source);

            __HAL_TIM_SET_COUNTER(&htim4, 0);
            last_encoder_counter = 0;
            HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

            LL_GPIO_SetOutputPin(Light_fire_L_GPIO_Port, Light_fire_L_Pin);
            LL_GPIO_SetOutputPin(Light_fire_R_GPIO_Port, Light_fire_R_Pin);
            osDelay(100);
            LL_GPIO_SetOutputPin(On_L_GPIO_Port, On_L_Pin);
            LL_GPIO_SetOutputPin(On_R_GPIO_Port, On_R_Pin);
            LOG_INFO("amp power START, wait for power to stabilize");
            osDelay(5000);
            LL_GPIO_ResetOutputPin(Light_fire_L_GPIO_Port, Light_fire_L_Pin);
            osDelay(100);
            LL_GPIO_ResetOutputPin(Light_fire_R_GPIO_Port, Light_fire_R_Pin);
            
            EtatAmp = AMP_ON;

            ES9038Q2M_DAC_SetMute_Force(false);
            LOG_INFO("amp power ON, unmuted, let's rock...");
        }
        if (!CommandeAmp && EtatAmp != AMP_OFF)
        {
            LOG_WARN("amp power OFF start sequence");
            EtatAmp = AMP_OFF;
            ES9038Q2M_DAC_SetMute_Force(true);
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
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM6)
    {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler_str(const char *where)
{
    LOG_ERR("Error_Handler from %s", where ? where : "?");
    LL_GPIO_SetOutputPin(LED3_LINE_GPIO_Port, LED3_LINE_Pin);

    /* Give RTT a moment to drain before any further action.
     * In normal flow Error_Handler is fatal-ish; if you ever
     * add a reset/halt here, the delay ensures the log gets out. */
    for (volatile uint32_t i = 0; i < 100000; i++)
    {
        __NOP();
    }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

