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
#include <stdbool.h>

static FSM_State state = HOME;
uint8_t song_index = 0;
uint16_t limit_switch_pressed = 0;
uint16_t target_FSM = 0;
uint8_t solenoid = 0;
bool limit_switch = false;
bool done_move = false;
bool done_play = false;
bool reset = false;

uint8_t notes[] = {46, 48, 50, 51, 53, 55, 47};
float times[] = {0, 250, 500, 750, 1250, 1500, 2000};
uint8_t numNotes = sizeof(notes) / sizeof(notes[0]);

static SongNote song[] = {
		{46, 0, 100},
		{48, 250, 100},
		{50, 500, 100},
		{51, 750, 100},
		{53, 1250, 100},
		{55, 1500, 100},
		{47, 2000, 100}
};

void FSM(void);
void CheckNextNote(uint8_t note);
bool NoteAlreadyCovered(uint8_t note);
void UpdateSolenoidPosition(void);
void PlayNote(uint8_t note, uint8_t playStart, uint8_t playStop);
void StopAllSolenoids(void);

void FSM(void) {
	uint32_t now = HAL_GetTick();

	switch (state) {
	case HOME:
		// after the limit switch is pressed, wait 2 seconds before starting the song
		StopAllSolenoids();
		if (limit_switch) {
			limit_switch_pressed = now;
			state = WAIT;
		}
		break;
	case WAIT:
		if (now - limit_switch_pressed >= 2000)  {
			song_index = 0;
			state = MOVE;
		}
		break;
	case MOVE:
		// done_move is sent from PID controller when motor has stopped moving inside the deadzone
		done_play = false;

		target_FSM = notes[song_index] * 811; // get next note from timeline

		if (done_move) {
			done_move = false;
			state = PLAY;
		}
		break;
	case PLAY:
		// done_play is asserted when the next note in the song timeline is not currently covered by the solenoids
		PlayNote(notes[song_index], times[song_index], times[song_index]+100);
		CheckNextNote(notes[song_index]); // checks to see if the next note is already covered by a solenoid

		if (song_index < numNotes) {
			if (NoteAlreadyCovered(song[song_index].note)) {
				state = PLAY;
			}
			else {
				state = MOVE;
			}
		}

		if (done_play) {
			if (song_index != (numNotes - 1)) state = MOVE; // if the next note is NOT already covered, move to the correct position
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

void CheckNextNote(uint8_t note) {
	//if (note == solenoid_white) play = true;
	//else if (note == solenoid_black) play = true;
	//else done_play = true;
}

bool NoteAlreadyCovered(uint8_t note) {
	return false;
}

void UpdateSolenoidPosition(void) {
}

void PlayNote(uint8_t note, uint8_t playStart, uint8_t playStop) {
	uint32_t now = HAL_GetTick();

	switch (solenoid) {
	case 1:
		if (now - playStart < playStop) HAL_GPIO_WritePin(SOLENOID_PORT, SOLENOID_1_PIN, GPIO_PIN_SET);
		else HAL_GPIO_WritePin(SOLENOID_PORT, SOLENOID_1_PIN, GPIO_PIN_RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(SOLENOID_PORT, SOLENOID_2_PIN, GPIO_PIN_SET);
		break;
	case 3:
		HAL_GPIO_WritePin(SOLENOID_PORT, SOLENOID_3_PIN, GPIO_PIN_SET);
		break;
	case 4:
		HAL_GPIO_WritePin(SOLENOID_PORT, SOLENOID_4_PIN, GPIO_PIN_SET);
		break;
	case 5:
		HAL_GPIO_WritePin(SOLENOID_PORT, SOLENOID_5_PIN, GPIO_PIN_SET);
		break;
	default:
		exit(EXIT_FAILURE);
	}
	//return done_play;
}

void StopAllSolenoids(void) {
	HAL_GPIO_WritePin(SOLENOID_PORT, SOLENOID_1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SOLENOID_PORT, SOLENOID_2_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SOLENOID_PORT, SOLENOID_3_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SOLENOID_PORT, SOLENOID_4_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SOLENOID_PORT, SOLENOID_5_PIN, GPIO_PIN_RESET);
}
