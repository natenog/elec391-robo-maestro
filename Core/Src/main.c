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

COM_InitTypeDef BspCOMInit;
__IO uint32_t BspButtonState = BUTTON_RELEASED;

/* USER CODE BEGIN PV */
float Kp_displacement = 13.3f;
float Ki_displacement = 3.015f;
//float Kd = 1.2f;
float Kd_displacement = 1.5f;
float Kp_velocity = 0.6f;
float Ki_velocity = 5.0f;
uint8_t N = 25;
float dt = 0.001f;

float vel = 0.0f;
float maxVel = 25000.0f;
float maxAccel = 30000.0f;

MotorStruct Motor = {0};
volatile int32_t pos = 0;
volatile int32_t delta = 0;
volatile int32_t prevPos = 0;
volatile int32_t error_displacement = 0;
volatile int32_t prevError_displacement = 0;
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
bool solenoidOn = false;

volatile uint32_t lastTimestamp = 0;
volatile uint32_t currentTimestamp = 0;

const Note notes[] = {
	{"C2", },
	{"D2", },
	{"E2", },
	{"F2", },
	{"G2", },
	{"A2", },
	{"B2", },
	{"C3", },
	{"D3", },
	{"E3", },
	{"F3", },
	{"G3", },
	{"A3", },
	{"B3", },
	{"C4", },
	{"D4", },
	{"E4", },
	{"F4", },
	{"G4", },
	{"A4", },
	{"B4", },
	{"C5", },
	{"D5", },
	{"E5", },
	{"F5", },
	{"G5", },
	{"A5", },
	{"B5", },
	{"C6", },
	{"D6", },
	{"E6", },
	{"F6", }
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
bool Home(void);
void RateLimiter(int32_t target);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void MotorForward(void);
void MotorBackward(void);
void MotorStop(void);
void MotorSetSpeedPercent(float percent);
void SolenoidPress(int pressTime_ms);
//void MotorControl(MotorDirection direction);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM23_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&HTIM_MOTOR, TIM_CHANNEL_MOTOR_A);
  HAL_TIM_PWM_Start(&HTIM_MOTOR, TIM_CHANNEL_MOTOR_B);
  HAL_TIM_PWM_Start(&htim23, TIM_CHANNEL_1);
  HAL_TIM_Base_Start(&HTIM_MOTOR);
  HAL_TIM_Encoder_Start(&HTIM_ENCODER, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&HTIM_PID);
  MotorSetSpeedPercent(0);
  //__HAL_TIM_MOE_ENABLE(&htim1);
  //__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 999);

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN BSP */

  /* -- Sample board code to send message over COM1 port ---- */
  printf("Welcome to STM32 world !\n\r");

  /* -- Sample board code to switch on leds ---- */
  BSP_LED_On(LED_GREEN);
  BSP_LED_On(LED_YELLOW);
  BSP_LED_On(LED_RED);

  /* USER CODE END BSP */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t lastPrint = 0;
  MotorForward();
  printf("\033[2J\033[H");
  while (1)
  {
	uint32_t now = HAL_GetTick();
	if (now - lastPrint >= 100)   // every 100 ms
		{
			lastPrint = HAL_GetTick();
			//printf("Pos=%ld  d=%ld  deriv=%lf  int=%lf  out=%lf  err=%ld  prevErr=%ld\r\n", pos, delta, deriv, integral, output, error, prevError);
			if (now < 10000) {
				printf("%ld,%ld,%ld,%lf,%lf,%lf,%lf,%lf\r\n", now, pos, subTarget, prop_displacement, integral_displacement, deriv_displacement, angularVelocity, output);
			}
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


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

bool Home(void) {
	enCtrl = false; // disable PID control loop
	MotorSetSpeedPercent(0);
	HAL_Delay(1000);

	while (1) {
		MotorBackward();
		MotorSetSpeedPercent(50);

		if (HAL_GPIO_ReadPin(HOME_PORT, HOME_PIN)) {
			MotorStop();
			MotorSetSpeedPercent(0);
			HAL_Delay(500);
			pos = 0; // reset encoder value to 0
			break;
		}
	}

	enCtrl = true;
	HAL_Delay(1000);

	return true;
}

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
	float brakingDistance = (vel_abs * vel_abs) / (2.0f * maxAccel);

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
					if (vel < 0.0f) vel = 0.0f;
				}
				else if (vel < 0.0f) {
					vel += maxAccel * dt;
					if (vel > 0.0f) vel = 0.0f;
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
	//float INT_MAX = 100.0f / Ki;
	if (htim == &HTIM_PID) {
		pos = __HAL_TIM_GET_COUNTER(&HTIM_ENCODER);
		delta = (int16_t)(pos - prevPos);
		prevPos = pos;

		if (enCtrl) {
			// PID controller method: Backwards Euler
			// sampling frequency f = 1kHz --> sampling period dt = 0.001 s
			// Integral term = i = dt * (z / (z - 1)) --> step response u[n] at each dt
			// Derivative term = d = N / (1 + N*dt*(z / (z-1)))

			// input the rate limiter output into the PID controller
			RateLimiter(target);

			error_displacement = subTarget - pos;
			prop_displacement = Kp_displacement * error_displacement;
			angularVelocity = (float)delta/dt;

			/*
			if (abs(target - pos) >= 30) {
				integral += Ki * error * dt;
			}
			*/

			deriv_displacement = (prevDeriv_displacement + Kd_displacement * N * (error_displacement - prevError_displacement)) / (1.0f + N * dt);

			// Integral anti-windup
			/*
			if (integral > INT_MAX) {
				integral = INT_MAX;
			}
			else if (integral < -INT_MAX) {
				integral = -INT_MAX;
			}
			*/

			PD_output = prop_displacement + deriv_displacement;
			error_velocity = PD_output - angularVelocity;

			prop_velocity = Kp_velocity * error_velocity;
			integral_velocity += Ki_velocity * error_velocity * dt;

			output = prop_velocity + integral_velocity;

			// Clamp output
			if (output > 100.0f) {
				output = 100.0f;
			}
			else if (output < -100.0f) {
				output = -100.0f;
			}

			// Switch direction depending on output sign
			if (output > 0.0f) {
				MotorForward();
			}
			else if (output < 0.0f) {
				MotorBackward();
				output *= -1.0f;
			}
			else {
				MotorStop();
			}

			prevError_displacement = error_displacement;
			prevDeriv_displacement = deriv_displacement;

			// Implement dead-zone (tolerance) to reset derivative and integral terms to 0
			if (abs(subTarget - pos) < 30) {
				prevError_displacement = 0.0f;
				//integral = 0.0f;
				//deriv = 0.0f;
				prevDeriv_displacement = 0.0f;
				MotorSetSpeedPercent(0);
				//if (abs(target-pos) < 10) {
				prop_displacement = 0.0f;
				integral_velocity = 0.0f;
				output = 0.0f;
				//}
			} else {
				MotorSetSpeedPercent(output);
			}
		}

	}
}

void MotorForward(void) {
	HAL_GPIO_WritePin(MOTOR_IN1_PORT, MOTOR_IN1_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_IN2_PORT, MOTOR_IN2_PIN, GPIO_PIN_RESET);
}

void MotorBackward(void) {
	HAL_GPIO_WritePin(MOTOR_IN1_PORT, MOTOR_IN1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_IN2_PORT, MOTOR_IN2_PIN, GPIO_PIN_SET);
}

void MotorStop(void) {
	HAL_GPIO_WritePin(MOTOR_IN1_PORT, MOTOR_IN1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_IN2_PORT, MOTOR_IN2_PIN, GPIO_PIN_RESET);
}

void MotorSetSpeedPercent(float percent) {
    if(percent > 100) percent = 100;
    uint32_t speed = (percent / 100.0) * 63999;
    __HAL_TIM_SET_COMPARE(&HTIM_MOTOR, TIM_CHANNEL_1, speed);
}

void SolenoidPress(int pressTime_ms) {
	uint32_t startTime = HAL_GetTick();
	solenoidOn = true;

	HAL_GPIO_WritePin(SOLENOID_1_PORT, SOLENOID_1_PIN, GPIO_PIN_SET);

	if (HAL_GetTick() - startTime > pressTime_ms) {
		HAL_GPIO_WritePin(SOLENOID_1_PORT, SOLENOID_1_PIN, GPIO_PIN_RESET);
	}
}

/*
void MotorControl(MotorDirection direction) {
	switch (direction) {
	case MOTOR_FORWARD:
		HAL_GPIO_WritePin(MOTOR_IN1_PORT, MOTOR_IN1_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MOTOR_IN2_PORT, MOTOR_IN2_PIN, GPIO_PIN_RESET);
		break;
	case MOTOR_BACKWARD:
		HAL_GPIO_WritePin(MOTOR_IN1_PORT, MOTOR_IN1_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_IN2_PORT, MOTOR_IN2_PIN, GPIO_PIN_SET);
		break;
	case MOTOR_STOP:
		HAL_GPIO_WritePin(MOTOR_IN1_PORT, MOTOR_IN1_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_IN2_PORT, MOTOR_IN2_PIN, GPIO_PIN_RESET);
		break;
	default:
		printf("ERROR: Invalid motor control setting. Please specify either MOTOR_FORWARD, MOTOR_BACKWARD, or MOTOR_STOP");
		exit(EXIT_FAILURE);
	}
}
*/

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief BSP Push Button callback
  * @param Button Specifies the pressed button
  * @retval None
  */
void BSP_PB_Callback(Button_TypeDef Button)
{
  if (Button == BUTTON_USER)
  {
    BspButtonState = BUTTON_PRESSED;
  }
}

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
