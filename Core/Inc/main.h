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
#include "stm32f1xx_hal.h"

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
#define V_SIO_Pin GPIO_PIN_12
#define V_SIO_GPIO_Port GPIOB
#define V_SCK_Pin GPIO_PIN_13
#define V_SCK_GPIO_Port GPIOB
#define H_SIO_Pin GPIO_PIN_14
#define H_SIO_GPIO_Port GPIOB
#define H_SCK_Pin GPIO_PIN_15
#define H_SCK_GPIO_Port GPIOB
#define B_LEFT_Pin GPIO_PIN_6
#define B_LEFT_GPIO_Port GPIOB
#define B_UP_Pin GPIO_PIN_7
#define B_UP_GPIO_Port GPIOB
#define B_DOWN_Pin GPIO_PIN_8
#define B_DOWN_GPIO_Port GPIOB
#define B_RIGHT_Pin GPIO_PIN_9
#define B_RIGHT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
