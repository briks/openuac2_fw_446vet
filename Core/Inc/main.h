/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum
{
    BLOCKING_ERROR,
    ERROR_I2C,
    ERROR_MAX_NBR
} errorNbr;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define POWER_BUTTON_PRESS_MIN_TIME  100 // in ms, min time to detect power button action
#define POWER_BUTTON_PRESS_MAX_TIME 1000 // in ms, max time to detect power button action

extern bool CommandeAmp; // variable globale commande amplis on/off

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void Error_cancel_nonBlocking(errorNbr errorBit_nBr);
void Error_Handler_nonBlocking(char *errorStr, errorNbr errorBit_nBr);
void MX_I2C1_Init(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ANALOG_ON_Pin LL_GPIO_PIN_2
#define ANALOG_ON_GPIO_Port GPIOE
#define Light_fire_R_Pin LL_GPIO_PIN_3
#define Light_fire_R_GPIO_Port GPIOE
#define Light_fire_L_Pin LL_GPIO_PIN_4
#define Light_fire_L_GPIO_Port GPIOE
#define Led_G_Pin LL_GPIO_PIN_5
#define Led_G_GPIO_Port GPIOE
#define Led_R_Pin LL_GPIO_PIN_6
#define Led_R_GPIO_Port GPIOE
#define RELAY_ON_Pin LL_GPIO_PIN_13
#define RELAY_ON_GPIO_Port GPIOC
#define On_L_Pin LL_GPIO_PIN_14
#define On_L_GPIO_Port GPIOC
#define On_R_Pin LL_GPIO_PIN_15
#define On_R_GPIO_Port GPIOC
#define SEL_SPDIF_Pin LL_GPIO_PIN_4
#define SEL_SPDIF_GPIO_Port GPIOC
#define PGA_M_Pin LL_GPIO_PIN_10
#define PGA_M_GPIO_Port GPIOE
#define SPI4_CS_Pin LL_GPIO_PIN_11
#define SPI4_CS_GPIO_Port GPIOE
#define PDN_Pin LL_GPIO_PIN_14
#define PDN_GPIO_Port GPIOB
#define LED1_SPDIF_Pin LL_GPIO_PIN_15
#define LED1_SPDIF_GPIO_Port GPIOB
#define LED2_BT_Pin LL_GPIO_PIN_8
#define LED2_BT_GPIO_Port GPIOD
#define LED3_LINE_Pin LL_GPIO_PIN_9
#define LED3_LINE_GPIO_Port GPIOD
#define LED4_USB_Pin LL_GPIO_PIN_10
#define LED4_USB_GPIO_Port GPIOD
#define EXT_INT_ENCODER_Pin LL_GPIO_PIN_11
#define EXT_INT_ENCODER_GPIO_Port GPIOD
#define EXT_INT_ENCODER_EXTI_IRQn EXTI15_10_IRQn
#define TIM4_CH1_ENCODER_Pin LL_GPIO_PIN_12
#define TIM4_CH1_ENCODER_GPIO_Port GPIOD
#define TIM4_CH2_ENCODER_Pin LL_GPIO_PIN_13
#define TIM4_CH2_ENCODER_GPIO_Port GPIOD
#define MUX_EN_Pin LL_GPIO_PIN_11
#define MUX_EN_GPIO_Port GPIOA
#define MUX_SEL_Pin LL_GPIO_PIN_12
#define MUX_SEL_GPIO_Port GPIOA
#define DSDOE_Pin LL_GPIO_PIN_11
#define DSDOE_GPIO_Port GPIOC
#define TIM3_ETR_AUDIO_SYNC_Pin LL_GPIO_PIN_2
#define TIM3_ETR_AUDIO_SYNC_GPIO_Port GPIOD
#define BT_RST_Pin LL_GPIO_PIN_7
#define BT_RST_GPIO_Port GPIOD
#define BT_PWR_Pin LL_GPIO_PIN_4
#define BT_PWR_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
