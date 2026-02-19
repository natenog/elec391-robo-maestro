#ifndef __ENCODER_H
#define __ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

// Variables
// TODO: Add pins for encoder connections
#define ENC_A_PORT GPIOE
#define ENC_A_PIN GPIO_PIN_14
#define ENC_B_PORT GPIOG
#define ENC_B_PIN GPIO_PIN_14

// Functions
uint8_t ReadEncoder();
void EncoderISR();

#ifdef __cplusplus
}
#endif

#endif
