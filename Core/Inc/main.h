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
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TEC_1_SWEN_Pin LL_GPIO_PIN_2
#define TEC_1_SWEN_GPIO_Port GPIOE
#define TEC_1_EN_Pin LL_GPIO_PIN_5
#define TEC_1_EN_GPIO_Port GPIOE
#define EF_TEC_AUX_Pin LL_GPIO_PIN_6
#define EF_TEC_AUX_GPIO_Port GPIOE
#define EF_TEC_EN_Pin LL_GPIO_PIN_13
#define EF_TEC_EN_GPIO_Port GPIOC
#define WD_DONE_Pin LL_GPIO_PIN_1
#define WD_DONE_GPIO_Port GPIOA
#define EXP_URRT_TX_Pin LL_GPIO_PIN_8
#define EXP_URRT_TX_GPIO_Port GPIOD
#define EXP_UART_RX_Pin LL_GPIO_PIN_9
#define EXP_UART_RX_GPIO_Port GPIOD
#define LED_B_Pin LL_GPIO_PIN_14
#define LED_B_GPIO_Port GPIOD
#define LED_G_Pin LL_GPIO_PIN_15
#define LED_G_GPIO_Port GPIOD
#define TEC_SCK_Pin LL_GPIO_PIN_3
#define TEC_SCK_GPIO_Port GPIOB
#define TEC_MISO_Pin LL_GPIO_PIN_4
#define TEC_MISO_GPIO_Port GPIOB
#define TEC_MOSI_Pin LL_GPIO_PIN_5
#define TEC_MOSI_GPIO_Port GPIOB
#define TEC_4_CS_Pin LL_GPIO_PIN_8
#define TEC_4_CS_GPIO_Port GPIOB
#define TEC_3_CS_Pin LL_GPIO_PIN_9
#define TEC_3_CS_GPIO_Port GPIOB
#define TEC_2_CS_Pin LL_GPIO_PIN_0
#define TEC_2_CS_GPIO_Port GPIOE
#define TEC_1_CS_Pin LL_GPIO_PIN_1
#define TEC_1_CS_GPIO_Port GPIOE
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
