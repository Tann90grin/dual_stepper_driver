/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define MS1_B_Pin GPIO_PIN_4
#define MS1_B_GPIO_Port GPIOA
#define MS2_B_Pin GPIO_PIN_5
#define MS2_B_GPIO_Port GPIOA
#define STEP_B_Pin GPIO_PIN_6
#define STEP_B_GPIO_Port GPIOA
#define DIR_B_Pin GPIO_PIN_7
#define DIR_B_GPIO_Port GPIOA
#define STDBY_B_Pin GPIO_PIN_0
#define STDBY_B_GPIO_Port GPIOB
#define SPREAD_B_Pin GPIO_PIN_1
#define SPREAD_B_GPIO_Port GPIOB
#define INDEX_B_Pin GPIO_PIN_2
#define INDEX_B_GPIO_Port GPIOB
#define DIAG_B_IN_Pin GPIO_PIN_10
#define DIAG_B_IN_GPIO_Port GPIOB
#define UART_SEL_Pin GPIO_PIN_12
#define UART_SEL_GPIO_Port GPIOB
#define DIAG_A_OUT_Pin GPIO_PIN_13
#define DIAG_A_OUT_GPIO_Port GPIOB
#define DIAG_B_OUT_Pin GPIO_PIN_15
#define DIAG_B_OUT_GPIO_Port GPIOB
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define STEP_A_Pin GPIO_PIN_15
#define STEP_A_GPIO_Port GPIOA
#define MS1_A_Pin GPIO_PIN_3
#define MS1_A_GPIO_Port GPIOB
#define MS2_A_Pin GPIO_PIN_4
#define MS2_A_GPIO_Port GPIOB
#define INDEX_A_Pin GPIO_PIN_5
#define INDEX_A_GPIO_Port GPIOB
#define DIR_A_Pin GPIO_PIN_6
#define DIR_A_GPIO_Port GPIOB
#define STDBY_A_Pin GPIO_PIN_7
#define STDBY_A_GPIO_Port GPIOB
#define SPREAD_A_Pin GPIO_PIN_8
#define SPREAD_A_GPIO_Port GPIOB
#define DIAG_A_IN_Pin GPIO_PIN_9
#define DIAG_A_IN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
