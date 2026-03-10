/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

#include "stm32h7xx_nucleo.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "encoder.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct MotorStruct {
	int32_t position;
	int32_t prevPosition;
	int32_t targetPosition;
	int32_t error;
	int32_t prevError;
} MotorStruct;

typedef enum {
	MOTOR_FORWARD,
	MOTOR_BACKWARD,
	MOTOR_STOP
} MotorDirection;

typedef enum {
	MOTOR_STATE_FORWARD,
	MOTOR_STATE_BACKWARD,
	MOTOR_STATE_STOP
} MotorState;

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
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define MOTOR_IN1_PORT GPIOB
#define MOTOR_IN1_PIN GPIO_PIN_3

#define MOTOR_IN2_PORT GPIOB
#define MOTOR_IN2_PIN GPIO_PIN_4

#define HOME_PORT GPIOA
#define HOME_PIN GPIO_PIN_1

#define HTIM_MOTOR htim2
#define HTIM_PID htim4
#define TIM_CHANNEL_MOTOR_A TIM_CHANNEL_1
#define TIM_CHANNEL_MOTOR_B TIM_CHANNEL_2
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
