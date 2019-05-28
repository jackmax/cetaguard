/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32l4xx_hal.h"

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
#define READ_PIN_PP(PORT, PIN) ((GPIO ## PORT)->IDR & (GPIO_PIN_ ## PIN))
#define READ_PIN_L(LABEL) ((LABEL ## _GPIO_Port)-> IDR & (LABEL ## _Pin))
#define WRITE_PIN_PP(PORT, PIN, STATE) do {(STATE ? (GPIO ## PORT)->BSRR : (GPIO ## PORT)->BRR) = (uint32_t)(GPIO_PIN_ ## PIN);} while (0)
#define WRITE_PIN_L(LABEL, STATE) do {(STATE ? (LABEL ## _GPIO_Port)->BSRR : (LABEL ## _GPIO_Port)->BRR) = (uint32_t)(LABEL ## _Pin);} while (0)

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define Radio_PKT_Pin GPIO_PIN_3
#define Radio_PKT_GPIO_Port GPIOA
#define Radio_PKT_EXTI_IRQn EXTI3_IRQn
#define SPI1_NCS_Pin GPIO_PIN_0
#define SPI1_NCS_GPIO_Port GPIOB
#define Radio_RST_Pin GPIO_PIN_8
#define Radio_RST_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_3
#define LED_GPIO_Port GPIOB
#define Btn1_Pin GPIO_PIN_4
#define Btn1_GPIO_Port GPIOB
#define Btn1_EXTI_IRQn EXTI4_IRQn
#define Btn2_Pin GPIO_PIN_5
#define Btn2_GPIO_Port GPIOB
#define Btn2_EXTI_IRQn EXTI9_5_IRQn
#define Btn3_Pin GPIO_PIN_6
#define Btn3_GPIO_Port GPIOB
#define Btn3_EXTI_IRQn EXTI9_5_IRQn
#define Btn4_Pin GPIO_PIN_7
#define Btn4_GPIO_Port GPIOB
#define Btn4_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
