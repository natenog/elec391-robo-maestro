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
#include <math.h>
#include "fsm.h"
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
float Kp_displacement = 20.0f;
float Ki_displacement = 1.2f;
float Kd_displacement = 0.3f;
float Kp_velocity = 0.6f;
float Ki_velocity = 5.0f;
float N = 6.5;
const float dt_inner = 0.0005f;   // 2kHz
const float dt_outer = 0.001f;    // 1kHz
float countsToRad = (2.0f * M_PI) / 2797.0f;

float vel = 0.0f;
float maxVel = 40000.0f;
float maxAccel = 75000.0f;
float alpha = 0.05;

volatile int16_t pos = 0;
volatile float delta = 0;
volatile int16_t prevPos = 0;
volatile int32_t lastTargetFSM = 0;

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
volatile float rawVelocity = 0.0f;

int32_t subTarget = 0;
float subTarget_f = 0.0f;
bool done_move = false;
volatile uint8_t outerLoopCounter = 0;
static uint8_t startupCount = 0;

bool homed = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void RateLimiter(int32_t finalTarget);
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
  StopAllSolenoids();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    FSM();
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

/* ============================================================================
 * Stall-based homing.
 *
 * Drives the carriage toward the home stop and watches the encoder.
 * When the encoder stops changing for several consecutive checks, we know
 * the carriage has hit the hard stop. PA1 limit switch is used as a backup
 * if it happens to be wired correctly.
 *
 * This is called repeatedly from the FSM HOME state. It is non-blocking.
 * ============================================================================ */
void Home(void) {
    static int16_t  home_prev_enc   = 0;
    static uint16_t home_stall_cnt  = 0;
    static uint8_t  home_init_done  = 0;
    static uint32_t home_start_tick = 0;

    if (homed) return;

    // First call: snapshot encoder, start moving
    if (!home_init_done) {
        home_prev_enc   = (int16_t)__HAL_TIM_GET_COUNTER(&HTIM_ENCODER);
        home_stall_cnt  = 0;
        home_start_tick = HAL_GetTick();
        home_init_done  = 1;
    }

    // Drive toward home stop. Channel 2 = original homing direction.
    MotorSetSpeedPercentCh1(0);
    MotorSetSpeedPercentCh2(15.0f);

    // Grace period: give the motor time to actually start before we
    // start checking for stall. Without this we'd "stall" instantly.
    if (HAL_GetTick() - home_start_tick < 400) {
        return;
    }

    int16_t cur_enc = (int16_t)__HAL_TIM_GET_COUNTER(&HTIM_ENCODER);

    // If the encoder barely moved since last check, count it as a stall tick.
    if (abs((int)(cur_enc - home_prev_enc)) < 2) {
        home_stall_cnt++;
    } else {
        home_stall_cnt = 0;
    }
    home_prev_enc = cur_enc;

    // Backup: physical limit switch on PA1.
    // Original code used GPIO_PIN_SET (active-high) — keep that meaning here.
    bool switch_hit = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET);

    // Stall threshold: ~50 consecutive FSM ticks of no movement
    bool stalled = (home_stall_cnt >= 50);

    if (stalled || switch_hit) {
        // Stop the motor
        MotorSetSpeedPercentCh1(0);
        MotorSetSpeedPercentCh2(0);
        HAL_Delay(100); // let it settle

        // Zero the encoder at home
        __HAL_TIM_SET_COUNTER(&HTIM_ENCODER, 0);

        // Reset all PID/control state so we don't immediately chase a stale target
        prevError_displacement = 0.0f;
        prevDeriv_displacement = 0.0f;
        integral_velocity      = 0.0f;
        prop_displacement      = 0.0f;
        deriv_displacement     = 0.0f;
        output                 = 0.0f;
        subTarget              = 0;
        subTarget_f            = 0.0f;
        vel                    = 0.0f;
        target_FSM             = 0;
        lastTargetFSM          = 0;
        pos                    = 0;
        prevPos                = 0;

        // Mark homed and reset our static state for next time
        homed          = true;
        home_init_done = 0;
        home_stall_cnt = 0;
    }
}

void RateLimiter(int32_t finalTarget) {
	// Rate limiter for target position --> increment target based on slew rate

	float remaining = (float)finalTarget - subTarget_f;
	int32_t position_tolerance = 5;
	int8_t direction = 0;

	if (fabsf(remaining) <= position_tolerance && fabsf(vel) <= (maxAccel * dt_outer)) {
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
	float nextVel = vel_abs + maxAccel * dt_outer;
	float brakingDistance = (nextVel * nextVel) / (2.0f * maxAccel);

	if (vel > 0.0f && direction < 0) {
		vel -= maxAccel * dt_outer;
		if (vel < 0.0f) vel = 0.0f;
	}
	else if (vel < 0.0f && direction > 0) {
		vel += maxAccel * dt_outer;
		if (vel > 0.0f) vel = 0.0f;
	}
	else {
		if (fabsf(remaining) <= brakingDistance) {
			if (vel_abs > 0.0f) {
				if (vel > 0.0f) {
					vel -= maxAccel * dt_outer;
					if (vel < maxAccel * dt_outer) {
						subTarget_f = (float)finalTarget;
						subTarget = finalTarget;
						vel = 0.0f;
						return;
					}
				}
				else if (vel < 0.0f) {
					vel += maxAccel * dt_outer;
					if (vel > -(maxAccel * dt_outer)) {
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
				vel += maxAccel * dt_outer * (float)direction;
				if (vel > maxVel) vel = maxVel;
				if (vel < -maxVel) vel = -maxVel;
			}
		}
	}

	subTarget_f += vel * dt_outer;

	if (direction > 0 && subTarget_f > (float)finalTarget) {
		subTarget_f = (float)finalTarget;
		vel = 0.0f;
	}
	else if (direction < 0 && subTarget_f < (float)finalTarget) {
		subTarget_f = (float)finalTarget;
		vel = 0.0f;
	}

	subTarget = (int16_t)subTarget_f;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &HTIM_PID) {
		pos = (int16_t)__HAL_TIM_GET_COUNTER(&HTIM_ENCODER);
		delta = (float)(pos - prevPos);
		prevPos = pos;

		if (enCtrl) {
			outerLoopCounter++;

			// Outer loop: position PD at 1kHz (every 2nd tick)
			if (outerLoopCounter >= 2) {
				outerLoopCounter = 0;

				RateLimiter(target_FSM);

				error_displacement = (subTarget - pos) * countsToRad;
				prop_displacement = Kp_displacement * error_displacement;

				deriv_displacement = (prevDeriv_displacement + Kd_displacement * N * (error_displacement - prevError_displacement)) / (1.0f + N * dt_outer);

				PD_output = prop_displacement + deriv_displacement;

				prevError_displacement = error_displacement;
				prevDeriv_displacement = deriv_displacement;
			}

			// Inner loop: velocity PI at 2kHz
			rawVelocity = (float)(delta / dt_inner) * countsToRad;

			if (startupCount < 10) {
				angularVelocity = 0.0f;
				startupCount++;
			} else {
				angularVelocity = alpha * rawVelocity + (1.0f - alpha) * angularVelocity;
			}
			error_velocity = PD_output - angularVelocity;

			prop_velocity = Kp_velocity * error_velocity;

			float raw_output = prop_velocity + integral_velocity;

			// Clamp output
			if (raw_output > 100.0f) {
				output = 100.0f;
			} else if (raw_output < -100.0f) {
				output = -100.0f;
			} else {
				output = raw_output;
			}

			// Anti-windup
			if (fabsf(raw_output) < 100.0f || (error_velocity * integral_velocity) < 0.0f) {
				integral_velocity += Ki_velocity * error_velocity * dt_inner;
			}

			// Dead-zone
			if (labs(target_FSM - pos) < 30 && fabsf(angularVelocity) < 1.0f) {
				prevError_displacement = 0.0f;
				prevDeriv_displacement = 0.0f;
				MotorSetSpeedPercentCh1(0);
				MotorSetSpeedPercentCh2(0);
				prop_displacement = 0.0f;
				integral_velocity = 0.0f;
				output = 0.0f;
				if (target_FSM != lastTargetFSM) {
					lastTargetFSM = target_FSM;
					done_move = true;
				}
			} else {
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

/* ============================================================================
 * Motor speed control — FIXED PWM SCALING
 *
 * Original bug: code wrote compare values up to 63999, but TIM15 period is 99.
 * Any nonzero percent saturated to 100% duty. Now scales against actual ARR.
 * ============================================================================ */
void MotorSetSpeedPercentCh1(float percent) {
    if (percent > 100.0f) percent = 100.0f;
    if (percent < 0.0f)   percent = 0.0f;
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&HTIM_MOTOR);
    uint32_t speed = (uint32_t)((percent / 100.0f) * (float)arr);
    __HAL_TIM_SET_COMPARE(&HTIM_MOTOR, TIM_CHANNEL_1, speed);
}

void MotorSetSpeedPercentCh2(float percent) {
    if (percent > 100.0f) percent = 100.0f;
    if (percent < 0.0f)   percent = 0.0f;
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&HTIM_MOTOR);
    uint32_t speed = (uint32_t)((percent / 100.0f) * (float)arr);
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
