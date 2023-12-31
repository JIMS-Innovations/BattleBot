/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define BW_RIGHT_B_Pin GPIO_PIN_0
#define BW_RIGHT_B_GPIO_Port GPIOB
#define BW_RIGHT_A_Pin GPIO_PIN_1
#define BW_RIGHT_A_GPIO_Port GPIOB
#define FW_RIGHT_B_Pin GPIO_PIN_2
#define FW_RIGHT_B_GPIO_Port GPIOB
#define FW_RIGHT_A_Pin GPIO_PIN_10
#define FW_RIGHT_A_GPIO_Port GPIOB
#define FW_LEFT_A_Pin GPIO_PIN_12
#define FW_LEFT_A_GPIO_Port GPIOB
#define FW_LEFT_B_Pin GPIO_PIN_13
#define FW_LEFT_B_GPIO_Port GPIOB
#define BW_LEFT_A_Pin GPIO_PIN_14
#define BW_LEFT_A_GPIO_Port GPIOB
#define BW_LEFT_B_Pin GPIO_PIN_15
#define BW_LEFT_B_GPIO_Port GPIOB
#define FW_LEFT_EN_Pin GPIO_PIN_4
#define FW_LEFT_EN_GPIO_Port GPIOB
#define FW_RIGHT_EN_Pin GPIO_PIN_5
#define FW_RIGHT_EN_GPIO_Port GPIOB
#define BW_RIGHT_EN_Pin GPIO_PIN_8
#define BW_RIGHT_EN_GPIO_Port GPIOB
#define BW_LEFT_EN_Pin GPIO_PIN_9
#define BW_LEFT_EN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
