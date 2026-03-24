/**
  ******************************************************************************
  * @file           : fsm.c
  * @brief          : Finite state machine for movement and solenoid pressing logic
  * @author			: Nate Noguera
  ******************************************************************************
  */

#include "fsm.h"

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

int16_t state = HOME;

void FSM(void);
void CheckNextNote(void);
void PlayNote(uint8_t note, uint8_t playStart, uint8_t playStop);

void FSM(void) {
	uint32_t now = HAL_GetTick();

	switch (state) {
	case HOME:
		// after the limit switch is pressed, wait 2 seconds before starting the song
		if (limit_switch && (now - _____ >= 2000)) {
			state = MOVE;
		}
		break;
	case MOVE:
		// done_move is sent from PID controller when motor has stopped moving inside the deadzone

		target = ___; // get next note from timeline

		if (done_move) {
			state = PLAY;
		}
		break;
	case PLAY:
		// done_play is asserted when the next note in the song timeline is not currently covered by the solenoids
		if (done_play) {
			if (_____) state = MOVE;
			else state = DONE; // transition to DONE state if there are no more notes in the timeline
		}
		break;
	case DONE:
		if (reset) state = HOME;
		break;
	default:
		exit(EXIT_FAILURE);
	}
}

void ParseTimeline(void) {
	// use MinJSON?
}

void CheckNextNote(void) {
	if (note == solenoid_white) play = true;
	else if (note == solenoid_black) play = true;
	else done_play = true;
}

void PlayNote(uint8_t note, uint8_t playStart, uint8_t playStop) {
	uint32_t now = HAL_GetTick();

	switch (solenoid) {
	case 1:
		if (now - playStart < playStop) HAL_GPIO_WritePin(SOLENOID_1_PORT, SOLENOID_1_PIN, GPIO_PIN_SET);
		else HAL_GPIO_WritePin(SOLENOID_1_PORT, SOLENOID_1_PIN, GPIO_PIN_RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(SOLENOID_2_PORT, SOLENOID_2_PIN, GPIO_PIN_SET);
		break;
	case 3:
		HAL_GPIO_WritePin(SOLENOID_3_PORT, SOLENOID_3_PIN, GPIO_PIN_SET);
		break;
	case 4:
		HAL_GPIO_WritePin(SOLENOID_4_PORT, SOLENOID_4_PIN, GPIO_PIN_SET);
		break;
	case 5:
		HAL_GPIO_WritePin(SOLENOID_5_PORT, SOLENOID_5_PIN, GPIO_PIN_SET);
		break;
	default:
		exit(EXIT_FAILURE);
	}
	return done_play;
}
