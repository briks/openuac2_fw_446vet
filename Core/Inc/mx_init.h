#ifndef __MX_INIT_H
#define __MX_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Peripheral handles (defined in mx_init.c) */
extern I2C_HandleTypeDef DAC_I2C_Handle; /* hi2c1 */
extern I2S_HandleTypeDef hi2s1;
extern I2S_HandleTypeDef hi2s3;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi3_tx;
extern SPI_HandleTypeDef hspi4;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart2;
extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
extern DMA_HandleTypeDef hdma_memtomem_dma2_stream0;
extern DMA_HandleTypeDef hdma_memtomem_dma2_stream1;

/* Clock + peripheral init (extracted from CubeMX-generated main.c) */
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_I2C1_Init(I2C_HandleTypeDef *hi2c);
void MX_I2S1_Init(void);
void MX_I2S3_Init(void);
void MX_SPI4_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_USART2_UART_Init(void);
void MX_USB_OTG_HS_PCD_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __MX_INIT_H */