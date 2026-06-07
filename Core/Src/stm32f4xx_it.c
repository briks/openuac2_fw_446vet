/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_def.h"
#define LOG_LEVEL LOG_LEVEL_INFO
#include "log.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
extern void USBD_AUDIO_Sync(USBD_HandleTypeDef *pdev);
extern void USBD_AUDIO_UpdateFB(USBD_HandleTypeDef *pdev, uint32_t txCnt);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_memtomem_dma2_stream0;
extern DMA_HandleTypeDef hdma_memtomem_dma2_stream1;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi3_tx;
extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN EV */
extern USBD_HandleTypeDef hUsbDeviceHS;
/* USER CODE END EV */

void fault_drain_and_halt(void)
{
    /* Wait for logs to be sent before halting */

    while (1) {
        /* Spin forever */
    }   
}

    /******************************************************************************/
    /*           Cortex-M4 Processor Interruption and Exception Handlers          */
    /******************************************************************************/
    /**
     * @brief This function handles Non maskable interrupt.
     */
    void
    NMI_Handler(void)
{
    LOG_ERR("NMI");
    fault_drain_and_halt();
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
    /* Capture key registers before they're lost */
    uint32_t cfsr  = SCB->CFSR;
    uint32_t hfsr  = SCB->HFSR;
    uint32_t mmfar = SCB->MMFAR;
    uint32_t bfar  = SCB->BFAR;
    LOG_ERR("HARDFAULT cfsr=0x%08lX hfsr=0x%08lX mmfar=0x%08lX bfar=0x%08lX",
            (unsigned long)cfsr, (unsigned long)hfsr,
            (unsigned long)mmfar, (unsigned long)bfar);
    fault_drain_and_halt();
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
    LOG_ERR("MemManage cfsr=0x%08lX mmfar=0x%08lX",
            (unsigned long)SCB->CFSR, (unsigned long)SCB->MMFAR);
    fault_drain_and_halt();
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
    LOG_ERR("BusFault cfsr=0x%08lX bfar=0x%08lX",
            (unsigned long)SCB->CFSR, (unsigned long)SCB->BFAR);
    fault_drain_and_halt();
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
    LOG_ERR("UsageFault cfsr=0x%08lX", (unsigned long)SCB->CFSR);
    fault_drain_and_halt();
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi3_tx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	LL_TIM_ClearFlag_UPDATE(TIM3);
  /* USER CODE END TIM3_IRQn 0 */
  /* USER CODE BEGIN TIM3_IRQn 1 */
	USBD_AUDIO_Sync(&hUsbDeviceHS);
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles I2C1 event interrupt.
  */
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */

  /* USER CODE END I2C1_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&DAC_I2C_Handle);
  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
}


/**
 * @brief This function handles EXTI line[15:10] interrupts.
 */
void EXTI15_10_IRQHandler(void)
{
    static uint32_t pressStartTime = 0;

    if (LL_GPIO_IsInputPinSet(EXT_INT_ENCODER_GPIO_Port, EXT_INT_ENCODER_Pin) == 1)
    {
        pressStartTime = HAL_GetTick();
        LOG_DBG("pwr button DOWN @ %lu ms", (unsigned long)pressStartTime);
    }
    if (LL_GPIO_IsInputPinSet(EXT_INT_ENCODER_GPIO_Port, EXT_INT_ENCODER_Pin) == 0)
    {
        uint32_t pressDuration = HAL_GetTick() - pressStartTime;
        LOG_DBG("pwr button UP, duration=%lu ms", (unsigned long)pressDuration);
        if ((pressDuration < POWER_BUTTON_PRESS_MAX_TIME) &&
            (pressDuration > POWER_BUTTON_PRESS_MIN_TIME))
        {
            CommandeAmp = !CommandeAmp;
        }
        else
        {
            LOG_INFO("power button press ignored (%lu ms)", (unsigned long)pressDuration);
        }
    }

    /* USER CODE END EXTI15_10_IRQn 0 */
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_11) != RESET)
    {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_11);
        /* USER CODE BEGIN LL_EXTI_LINE_11 */

        /* USER CODE END LL_EXTI_LINE_11 */
    }
    /* USER CODE BEGIN EXTI15_10_IRQn 1 */

    /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt and DAC1, DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_memtomem_dma2_stream0);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream3 global interrupt.
  */
void DMA2_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream3_IRQn 0 */

  /* USER CODE END DMA2_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  /* USER CODE BEGIN DMA2_Stream3_IRQn 1 */

  /* USER CODE END DMA2_Stream3_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go HS End Point 1 Out global interrupt.
  */
void OTG_HS_EP1_OUT_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_HS_EP1_OUT_IRQn 0 */

  /* USER CODE END OTG_HS_EP1_OUT_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS);
  /* USER CODE BEGIN OTG_HS_EP1_OUT_IRQn 1 */

  /* USER CODE END OTG_HS_EP1_OUT_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go HS End Point 1 In global interrupt.
  */
void OTG_HS_EP1_IN_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_HS_EP1_IN_IRQn 0 */

  /* USER CODE END OTG_HS_EP1_IN_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS);
  /* USER CODE BEGIN OTG_HS_EP1_IN_IRQn 1 */

  /* USER CODE END OTG_HS_EP1_IN_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go HS global interrupt.
  */
void OTG_HS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_HS_IRQn 0 */

  /* USER CODE END OTG_HS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS);
  /* USER CODE BEGIN OTG_HS_IRQn 1 */

  /* USER CODE END OTG_HS_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
