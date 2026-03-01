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
float Kp = 5.22505;
float Kd = 0.101565;
float Ki = 45.201;
MotorStruct Motor = {0};
volatile int32_t pos = 0;
volatile int32_t delta = 0;
volatile int32_t prevPos = 0;
volatile int32_t error = 0;
volatile int32_t prevError = 0;
volatile float d = 0.0;
volatile float i = 0.0;
volatile float output;
volatile float percent;
int32_t target = 2000;
bool enCtrl = false;

volatile uint32_t lastTimestamp = 0;
volatile uint32_t currentTimestamp = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
bool Home(void);
void Step(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void MotorForward(void);
void MotorBackward(void);
void MotorStop(void);
void MotorSetSpeedPercent(float percent);
void Move1WhiteKey(MotorDirection direction, float percent);
//void MotorControl(MotorDirection direction);
//void MotorSetSpeed(uint16_t speed);
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
  //MotorState motorState = MOTOR_STATE_FORWARD;
  //uint32_t motorTimer = 0;
  //Move1WhiteKey(MOTOR_FORWARD, 80);
  Move1WhiteKey(MOTOR_BACKWARD, 80);
  //Move1WhiteKey(MOTOR_BACKWARD, 80);
  MotorForward();
  printf("\033[2J\033[H");
  while (1)
  {
	uint32_t now = HAL_GetTick();
	if (now - lastPrint >= 100)   // every 100 ms
		{
			lastPrint = HAL_GetTick();
			printf("Pos=%ld  d=%ld  deriv=%lf  int=%lf  out=%lf  err=%ld  prevErr=%ld\r\n", pos, delta, d, i, output, error, prevError);
		}


	if (now >= 2000) {
		target = 300;
	}

	/*
	switch (motorState) {
		case MOTOR_STATE_FORWARD:
			MotorForward();
			MotorSetSpeedPercent(50);

			if (now - motorTimer >= 1000) {
				motorState = MOTOR_STATE_BACKWARD;
				motorTimer = now;
				printf("---Motor backwards---\r\n");
			}
			break;
		case MOTOR_STATE_BACKWARD:
			MotorBackward();
			MotorSetSpeedPercent(80);

			if (now - motorTimer >= 500) {
				motorState = MOTOR_STATE_STOP;
				motorTimer = now;
				printf("---Motor stop---\r\n");
			}
			break;
		case MOTOR_STATE_STOP:
			MotorStop();

			if (now - motorTimer >= 2000) {
				motorState = MOTOR_STATE_FORWARD;
				motorTimer = now;
				printf("---Motor forwards---\r\n");
			}
			break;
	}
	*/



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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	float dt = 0.001;
	float i_Max = 100 / Ki;
	if (htim == &HTIM_PID) {
		pos = __HAL_TIM_GET_COUNTER(&HTIM_ENCODER);
		delta = (int16_t)(pos - prevPos);
		prevPos = pos;

		if (enCtrl) {
			error = target - pos;
			d = (error - prevError) * dt;
			i += error * dt;
			output = Kp * error + Kd * d + Ki * i;
			//percent = fabs(output / 63999);


			if (i > i_Max) {
				i = i_Max;
			}
			else if (i < -i_Max) {
				i = -i_Max;
			}

			if (output > 100) {
				output = 100;
			}
			else if (output < -100) {
				output = -100;
			}

			if (output > 0) {
				MotorForward();
			}
			else if (output < 0) {
				MotorBackward();
				output *= -1;
			}
			else {
				MotorStop();
			}

			prevError = error;

			//currentTimestamp = HAL_GetTick();

			//printf("Timestamp: %lu\r\n", currentTimestamp - lastTimestamp);

			//lastTimestamp = currentTimestamp;

			MotorSetSpeedPercent(output);
		}

	}
}



void Move1WhiteKey(MotorDirection direction, float percent) {
	// TODO: Fix overshoot of roughly 100 encoder counts (probably fixed with PID?)
	int32_t source = pos;
	printf("Source = %lu, pos = %lu\r\n", source, pos);
	if (direction == MOTOR_FORWARD) {
		MotorForward();
		MotorSetSpeedPercent(percent);
		while (llabs(source-pos) < 811) {
			printf("Source = %ld, pos = %ld\r\n", source, pos);
			continue;
		}
		MotorBackward();
		MotorSetSpeedPercent(80);
		HAL_Delay(50);
		MotorStop();
	}
	else if (direction == MOTOR_BACKWARD) {
		MotorSetSpeedPercent(0);
		MotorBackward();
		HAL_Delay(1);
		MotorSetSpeedPercent(percent);
		while (llabs(source-pos) < 811) {
			printf("Source = %ld, pos = %ld\r\n", source, pos);
			continue;
		}
		MotorForward();
		MotorSetSpeedPercent(80);
		HAL_Delay(50);
		MotorStop();
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

/*
void MotorSlowDown(uint16_t speed) {
	// call condition: position is 1 key away
	// slowDown
	// S-curve function: speed = maxSpeed * (3*___^2 -2*___^3)

	uint16_t currentSpeed = speed;

	while (currentSpeed > 0) {
		currentSpeed /= 2;
		__HAL_TIM_SET_COMPARE(&HTIM_MOTOR, TIM_CHANNEL_1, currentSpeed);
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
