/**
 ******************************************************************************
 * @file		: encoder.c
 * @brief		: Code for encoder
 * @author		: Nate Noguera
 */

#include "encoder.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"

#include "stm32h7xx_nucleo.h"

uint8_t ReadEncoder() {
	uint8_t pinA = HAL_GPIO_ReadPin(ENC_A_PORT, ENC_A_PIN);
	uint8_t pinB = HAL_GPIO_ReadPin(ENC_B_PORT, ENC_B_PIN);
	return (pinA << 1) | pinB; // in binary notation: AB
}
/*
void EncoderISR() {

}
*/
