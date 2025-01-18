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
typedef enum
{
  DIR_STOP = 0u,
  DIR_TOP,
  DIR_TOP_RIGHT,
  DIR_RIGHT,
  DIR_BOTTOM_RIGHT,
  DIR_BOTTOM,
  DIR_BOTTOM_LEFT,
  DIR_LEFT,
  DIR_TOP_LEFT
} BOT_Direction;
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
#define M0_Direction_Pin GPIO_PIN_1
#define M0_Direction_GPIO_Port GPIOA
#define M0_Pulse_Pin GPIO_PIN_2
#define M0_Pulse_GPIO_Port GPIOA
#define M1_Direction_Pin GPIO_PIN_3
#define M1_Direction_GPIO_Port GPIOA
#define M1_Pulse_Pin GPIO_PIN_4
#define M1_Pulse_GPIO_Port GPIOA
#define STOPPER_LEFT_Pin GPIO_PIN_5
#define STOPPER_LEFT_GPIO_Port GPIOA
#define STOPPER_RIGHT_Pin GPIO_PIN_6
#define STOPPER_RIGHT_GPIO_Port GPIOA
#define STOPPER_TOP_Pin GPIO_PIN_7
#define STOPPER_TOP_GPIO_Port GPIOA
#define STOPPER_BOTTOM_Pin GPIO_PIN_8
#define STOPPER_BOTTOM_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
