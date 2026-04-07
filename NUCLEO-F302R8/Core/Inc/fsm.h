#ifndef __FSM_H
#define __FSM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f3xx_hal.h"

/* Exported types ------------------------------------------------------------*/
typedef enum {
	HOME,
	WAIT,
	MOVE,
	PLAY,
	DONE
} FSM_State;

typedef struct {
	uint8_t note;
	uint16_t start_ms;
	uint16_t duration_ms;
} SongNote;

typedef struct {
	uint8_t solenoid1;
	uint8_t solenoid2;
	uint8_t solenoid3;
	uint8_t solenoid4;
	uint8_t solenoid5;
} Solenoids;

extern FSM_State state;
extern int32_t target_FSM;
extern float keyPosition_mm;
extern bool enCtrl;

#define SOLENOID_PORT GPIOB
#define SOLENOID_1_PIN GPIO_PIN_5
#define SOLENOID_2_PIN GPIO_PIN_6
#define SOLENOID_3_PIN GPIO_PIN_7
#define SOLENOID_4_PIN GPIO_PIN_8
#define SOLENOID_5_PIN GPIO_PIN_9

/* Exported functions prototypes ---------------------------------------------*/
void FSM(void);
void FireSolenoid(uint8_t solenoid);
void SolenoidUpdate(void);
void StopAllSolenoids(void);

#ifdef __cplusplus
}
#endif

#endif
