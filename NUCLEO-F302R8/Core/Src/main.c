/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float Kp_displacement = 13.3f;
float Ki_displacement = 1.2f;
float Kd_displacement = 1.5f;
float Kp_velocity = 0.6f;
float Ki_velocity = 5.0f;
uint8_t N = 25;
const float dt = 0.001f;
float countsToRad = (2.0f * M_PI) / 2797.0f;

float vel = 0.0f;
float maxVel = 2000.0f;
float maxAccel = 5000.0f;

volatile int16_t pos = 0;
volatile float delta = 0;
volatile int16_t prevPos = 0;

volatile float error_displacement = 0;
volatile float prevError_displacement = 0;
volatile float error_velocity = 0.0f;
volatile float prevError_velocity = 0.0f;
volatile float prop_displacement = 0.0f;
volatile float integral_displacement = 0.0f;
volatile float deriv_displacement = 0.0f;
volatile float prop_velocity = 0.0f;
volatile float integral_velocity = 0.0f;
volatile float prevDeriv_displacement = 0.0f;
volatile float PD_output = 0.0f;
volatile float output = 0.0f;
volatile float percent = 0.0f;
volatile float angularDisplacement = 0.0f;
volatile float angularVelocity = 0.0f;
int32_t target = 2000;
int32_t subTarget = 0;
float subTarget_f = 0.0f;
//float position_snap_tolerance = 3.0f;
bool enCtrl = true;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void RateLimiter(int32_t target);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void MotorSetSpeedPercentCh1(float percent);
void MotorSetSpeedPercentCh2(float percent);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM15_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&HTIM_ENCODER, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&HTIM_PID);
  HAL_TIM_PWM_Start(&HTIM_MOTOR, TIM_MOTOR_CHANNEL_A);
  HAL_TIM_PWM_Start(&HTIM_MOTOR, TIM_MOTOR_CHANNEL_B);
  MotorSetSpeedPercentCh1(0.0);
  MotorSetSpeedPercentCh2(0.0);

  printf("\033[2J\033[H");
  uint32_t lastPrint = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  uint32_t now = HAL_GetTick();

	  if (now - lastPrint >= 100) { // every 100 ms
		  lastPrint = HAL_GetTick();
		  //printf("Pos=%ld  d=%ld  deriv=%lf  int=%lf  out=%lf  err=%ld  prevErr=%ld\r\n", pos, delta, deriv, integral, output, error, prevError);
		  //if (now < 10000) {
		  __disable_irq();
		  int16_t pos_copy = pos;
		  int32_t sub_copy = subTarget;
		  float prop_copy = prop_displacement;
		  float int_copy = integral_displacement;
		  float deriv_copy = deriv_displacement;
		  float vel_copy = angularVelocity;
		  float out_copy = output;
		  __enable_irq();

		  printf("%lu,%d,%ld,%f,%f,%f,%f,%f\r\n", now, pos_copy, sub_copy,
			(double)prop_copy, (double)int_copy, (double)deriv_copy,
			(double)vel_copy, (double)out_copy);
		  //}
	  }

	  if (now >= 2000) {
		  target = 300;
	  }

	  if (now >= 4000) {
		  target = 1000;
	  }

	  if (now >= 6000) {
		  target = 3000;
	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_TIM15;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim15ClockSelection = RCC_TIM15CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void RateLimiter(int32_t finalTarget) {
	// TODO: Add rate limiter for target position --> increment target based on slew rate

	float remaining = (float)finalTarget - subTarget_f;
	int32_t position_tolerance = 5;
	int8_t direction = 0;

	if (fabsf(remaining) <= position_tolerance && fabsf(vel) <= (maxAccel * dt)) {
		subTarget_f = (float)finalTarget;
		subTarget = finalTarget;
		vel = 0.0f;
		return;
	}

	if (remaining > 0.0f) {
		direction = 1;
	}
	else if (remaining < 0.0f) {
		direction = -1;
	}
	else {
		direction = 0;
	}

	float vel_abs = fabsf(vel);
	float nextVel = vel_abs + maxAccel * dt;
	float brakingDistance = (nextVel * nextVel) / (2.0f * maxAccel); // calculating one tick ahead due to sampling time 

	if (vel > 0.0f && direction < 0) {
		vel -= maxAccel * dt;
		if (vel < 0.0f) vel = 0.0f;
	}
	else if (vel < 0.0f && direction > 0) {
		vel += maxAccel * dt;
		if (vel > 0.0f) vel = 0.0f;
	}
	else {
		if (fabsf(remaining) <= brakingDistance) {
			if (vel_abs > 0.0f) {
				if (vel > 0.0f) {
					vel -= maxAccel * dt;
					if (vel < maxAccel * dt) {
						subTarget_f = (float)finalTarget;
						subTarget = finalTarget;
						vel = 0.0f;
						return;
					}
				}
				else if (vel < 0.0f) {
					vel += maxAccel * dt;
					if (vel > -(maxAccel * dt)) {
						subTarget_f = (float)finalTarget;
						subTarget = finalTarget;
						vel = 0.0f;
						return;
					}
				}
			}
		}
		else {
			if (fabsf(remaining) > position_tolerance) {
				vel += maxAccel * dt * (float)direction;
				if (vel > maxVel) vel = maxVel;
				if (vel < -maxVel) vel = -maxVel;
			}
		}
	}

	subTarget_f += vel * dt;

	if (direction > 0 && subTarget_f > (float)finalTarget) {
		subTarget_f = (float)finalTarget;
		vel = 0.0f;
	}
	else if (direction < 0 && subTarget_f < (float)finalTarget) {
		subTarget_f = (float)finalTarget;
		vel = 0.0f;
	}

	subTarget = (int32_t)subTarget_f;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &HTIM_PID) {
		pos = (int16_t)__HAL_TIM_GET_COUNTER(&HTIM_ENCODER);
		delta = (float)(pos - prevPos);
		prevPos = pos;

		if (enCtrl) {
			// PID controller method: Backwards Euler
			// sampling frequency f = 1kHz --> sampling period dt = 0.001 s
			// Integral term = i = dt * (z / (z - 1)) --> step response u[n] at each dt
			// Derivative term = d = N / (1 + N*dt*(z / (z-1)))

			// input the rate limiter output into the PID controller
			RateLimiter(target);

			error_displacement = (subTarget - pos)*countsToRad;
			prop_displacement = Kp_displacement * error_displacement;

			deriv_displacement = (prevDeriv_displacement + Kd_displacement * N * (error_displacement - prevError_displacement)) / (1.0f + N * dt);

			PD_output = prop_displacement + deriv_displacement;

			prevError_displacement = error_displacement;
			prevDeriv_displacement = deriv_displacement;

			// Integral anti-windup
			/*
			if (integral > INT_MAX) {
				integral = INT_MAX;
			}
			else if (integral < -INT_MAX) {
				integral = -INT_MAX;
			}
			*/

			angularVelocity = (float)(delta/dt)*countsToRad;
			error_velocity = PD_output - angularVelocity;

			prop_velocity = Kp_velocity * error_velocity;
			integral_velocity += Ki_velocity * error_velocity * dt;
			// Anti-windup
			if (integral_velocity > 100.0f) {
				integral_velocity = 100.0f;
			}
			else if (integral_velocity < -100.0f) {
				integral_velocity = -100.0f;
			}

			output = prop_velocity + integral_velocity;

			// Clamp output
			if (output > 100.0f) {
				output = 100.0f;
			}
			else if (output < -100.0f) {
				output = -100.0f;
			}

			// Implement dead-zone (tolerance) to reset derivative and integral terms to 0

			if (labs(target - pos) < 30) {
				prevError_displacement = 0.0f;
				//integral = 0.0f;
				//deriv = 0.0f;
				prevDeriv_displacement = 0.0f;
				MotorSetSpeedPercentCh1(0);
				MotorSetSpeedPercentCh2(0);
				//if (abs(target-pos) < 10) {
				prop_displacement = 0.0f;
				integral_velocity = 0.0f;
				output = 0.0f;
				//}
			} else {
				// Switch direction depending on output sign
				if (output > 0.0f) {
					MotorSetSpeedPercentCh1(output);
					MotorSetSpeedPercentCh2(0);
				}
				else if (output < 0.0f) {
					MotorSetSpeedPercentCh1(0);
					MotorSetSpeedPercentCh2(output * -1.0f);
				}
				else {
					MotorSetSpeedPercentCh1(0);
					MotorSetSpeedPercentCh2(0);
				}
			}
		}
	}
}

void MotorSetSpeedPercentCh1(float percent) {
    if(percent > 100) percent = 100;
	if(percent < 0 )  percent = 0;
    uint32_t speed = (percent / 100.0) * 63999;
    __HAL_TIM_SET_COMPARE(&HTIM_MOTOR, TIM_CHANNEL_1, speed);
}

void MotorSetSpeedPercentCh2(float percent) {
	if(percent > 100) percent = 100;
	if(percent < 0 )  percent = 0;
	uint32_t speed = (percent / 100.0) * 63999;
	__HAL_TIM_SET_COMPARE(&HTIM_MOTOR, TIM_CHANNEL_2, speed);
}

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
