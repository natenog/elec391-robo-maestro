#ifndef __ENCODER_H
#define __ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"

#include "stm32h7xx_nucleo.h"

#include "tim.h"

// Variables
// TODO: Add pins for encoder connections
#define ENC_A_PORT GPIOC
#define ENC_A_PIN GPIO_PIN_6
#define ENC_B_PORT GPIOC
#define ENC_B_PIN GPIO_PIN_7

#define HTIM_ENCODER htim3

// Functions
uint32_t ReadEncoder();
void EncoderISR();

#ifdef __cplusplus
}
#endif

#endif
