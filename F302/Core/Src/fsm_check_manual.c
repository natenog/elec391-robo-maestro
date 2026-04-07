/**
  ******************************************************************************
  * @file           : fsm_check.c
  * @brief          : Finite state machine for movement and solenoid pressing logic
  * @author			: Yousef Mohamed
  ******************************************************************************
  */

#include "main.h"
#include "fsm.h"
#include "tim.h"

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <limits.h>

// ============ SOLENOID IDS ============
#define SOL_W1   1
#define SOL_WREF 2
#define SOL_W2   3
#define SOL_B1   4
#define SOL_B2   5

// ============ PIANO KEY POSITIONS (in encoder counts) ============
// Each value is the encoder count where W_ref sits when centered on that key.
// Calibrate by manually jogging the carriage to each key and reading the encoder.
#define KEY_E2   0
#define KEY_A2   -1629 // correct
//#define KEY_C3   0
#define KEY_C3	814 // correct
#define KEY_D3   1628 // correct
#define KEY_F3  2444
#define KEY_G3  3258
#define KEY_GS3 3869
#define KEY_A3  4072 // correct
#define KEY_B3  4886
#define KEY_C4  5700 // correct
#define KEY_D4  6514
#define KEY_E4  7328 // correct
#define KEY_F4  8142
#define KEY_FS4 8549

// ============ SOLENOID OFFSETS (in encoder counts) ============
// How far each solenoid sits from W_ref. Positive = to the right.
// Calibrate by parking W_ref on a known key, then jogging until the
// solenoid being calibrated is centered on that same key. The difference
// in encoder counts is the offset.
#define OFFSET_W2   -1705
#define OFFSET_WREF     0
#define OFFSET_W1    1705
#define OFFSET_B1    -407
#define OFFSET_B2     407

#define SOLENOID_PULSE_MS 100

// ============ TYPES ============
typedef struct {
    int32_t motorTarget;  // where to move W_ref (encoder counts)
    uint8_t solenoid;     // which solenoid to fire
} NoteMapping;

typedef struct {
    int32_t key_counts;   // absolute encoder count for this key
    uint16_t time_ms;
    uint8_t solenoid;
} SongEntry;

// ============ STATE VARIABLES ============
FSM_State state = HOME;
uint8_t song_index = 0;
uint16_t limit_switch_pressed = 0;
int32_t target_FSM = 0;
bool limit_switch = false;
//bool done_move = false;
bool reset = false;
bool enCtrl = false;

volatile bool solenoidActive = false;
uint32_t solenoidOffTime = 0;
uint32_t song_start_time = 0;
NoteMapping currentNote;

// ============ SONG DATA ============
// list of piano keys and when to play ms from song start
// ============ HAPPY BIRTHDAY =================

/*
static SongEntry song[] = {
    {KEY_C4,   0,     SOL_W1},     // Step 1:  C4  at P0
    {KEY_D4,   1500,  SOL_WREF},   // Step 2:  E4  at P0
    {KEY_G4,   2500,  SOL_WREF},   // Step 3:  G4  at P1
    {KEY_AS4,  3500,  SOL_B2},     // Step 4:  Bb4 at P2
    {KEY_C5,   4500,  SOL_W2},     // Step 5:  C5  at P2
    {KEY_AS4,  5500,  SOL_B2},     // Step 6:  Bb4 at P2
    {KEY_G4,   6500,  SOL_WREF},   // Step 7:  G4  at P1
    {KEY_FS4,  7500,  SOL_B1},     // Step 8:  F#4 at P1
    {KEY_E4,   8500,  SOL_WREF},   // Step 9:  E4  at P0
    {KEY_DS4,  9500,  SOL_B1},     // Step 10: D#4 at P0
    {KEY_C4,   10500, SOL_W1},     // Step 11: C4  at P0
};
*/
// ============ HOUSE OF THE RISING SUN =========================

static SongEntry song[] = {
	{KEY_A2, 0, SOL_W2},
	{KEY_A3, 250*2.5f, SOL_W2},
	{KEY_C4, 500*2.5f, SOL_WREF},
	{KEY_E4, 750*2.5f, SOL_W1},
	{KEY_C4, 1000*2.5f, SOL_WREF},
	{KEY_A3, 1250*2.5f, SOL_W2},
	{KEY_C3, 1500*2.5f, SOL_W2},
	{KEY_G3, 1750*2.5f, SOL_WREF},
	{KEY_C4, 2000*2.5f, SOL_W1},
	{KEY_E4, 2250*2.5f, SOL_W1},
	{KEY_C4, 2500*2.5f, SOL_WREF},
	{KEY_G3, 2750*2.5f, SOL_W2},

	{KEY_D3, 3000*2.5f, SOL_W2},
	{KEY_A3, 3250*2.5f, SOL_WREF},
	{KEY_D4, 3500*2.5f, SOL_W1},
	{KEY_FS4, 3750*2.5f, SOL_B2},
	{KEY_D4, 4000*2.5f, SOL_WREF},
	{KEY_A3, 4250*2.5f, SOL_W2},

	{KEY_F3, 4500*2.5f, SOL_W2},
	{KEY_A3, 4750*2.5f, SOL_WREF},
	{KEY_C4, 5000*2.5f, SOL_W1},
	{KEY_F4, 5250*2.5f, SOL_W1},
	{KEY_C4, 5500*2.5f, SOL_WREF},
	{KEY_A3, 5750*2.5f, SOL_W2},

	{KEY_A2, 6000*2.5f, SOL_W2},
	{KEY_A3, 6250*2.5f, SOL_W2},
	{KEY_C4, 6500*2.5f, SOL_WREF},
	{KEY_E4, 6750*2.5f, SOL_W1},
	{KEY_C4, 7000*2.5f, SOL_WREF},
	{KEY_A3, 7250*2.5f, SOL_W2},

	//{KEY_E3, 7500*2.5f, SOL_W2},
	{KEY_GS3, 7750*2.5f, SOL_B1},
	{KEY_B3, 8000*2.5f, SOL_W1},
	{KEY_E4, 8250*2.5f, SOL_W1},
	{KEY_B3, 8500*2.5f, SOL_WREF},
	{KEY_GS3, 8750*2.5f, SOL_B2},

	{KEY_A2, 9000*2.5f, SOL_W2},
	{KEY_A3, 9250*2.5f, SOL_W2},
	{KEY_C4, 9500*2.5f, SOL_WREF},
	{KEY_E4, 9750*2.5f, SOL_W1},
	{KEY_C4, 10000*2.5f, SOL_WREF},
	{KEY_A3, 10250*2.5f, SOL_W2},

	//{KEY_E3, 10500*2.5f, SOL_W2},
	{KEY_GS3, 10750*2.5f, SOL_B1},
	{KEY_B3, 11000*2.5f, SOL_W1},
	{KEY_E4, 11250*2.5f, SOL_W1},
};
/*
static SongEntry song[] = {
	{KEY_A2, 0, SOL_W2},
	{KEY_A3, 250*2, SOL_W1},
	{KEY_C4, 500*2, SOL_WREF},
	{KEY_E4, 750*2, SOL_W2},
	{KEY_C4, 1000*2, SOL_WREF},
	{KEY_A3, 1250*2, SOL_W1},
	{KEY_C3, 1500, SOL_W1},
	{KEY_G3, 1750, SOL_WREF},
	{KEY_C4, 2000, SOL_W2},
	{KEY_E4, 2250, SOL_W2},
	{KEY_C4, 2500, SOL_WREF},
	{KEY_G3, 2750, SOL_W1},

	{KEY_D3, 3000, SOL_W1},
	{KEY_A3, 3250, SOL_WREF},
	{KEY_D4, 3500, SOL_W2},
	{KEY_FS4, 3750, SOL_B2},
	{KEY_D4, 4000, SOL_WREF},
	{KEY_A3, 4250, SOL_W1},

	{KEY_F3, 4500, SOL_W1},
	{KEY_A3, 4750, SOL_WREF},
	{KEY_C4, 5000, SOL_W2},
	{KEY_F4, 5250, SOL_W2},
	{KEY_C4, 5500, SOL_WREF},
	{KEY_A3, 5750, SOL_W1},

	{KEY_A2, 6000, SOL_W1},
	{KEY_A3, 6250, SOL_W1},
	{KEY_C4, 6500, SOL_WREF},
	{KEY_E4, 6750, SOL_W2},
	{KEY_C4, 7000, SOL_WREF},
	{KEY_A3, 7250, SOL_W1},

	//{KEY_E3, 7500, SOL_W1},
	{KEY_GS3, 7750, SOL_B2},
	{KEY_B3, 8000, SOL_W2},
	{KEY_E4, 8250, SOL_W2},
	{KEY_B3, 8500, SOL_WREF},
	{KEY_GS3, 8750, SOL_B1},

	{KEY_A2, 9000, SOL_W1},
	{KEY_A3, 9250, SOL_W1},
	{KEY_C4, 9500, SOL_WREF},
	{KEY_E4, 9750, SOL_W2},
	{KEY_C4, 10000, SOL_WREF},
	{KEY_A3, 10250, SOL_W1},

	//{KEY_E3, 10500, SOL_W1},
	{KEY_GS3, 10750, SOL_B2},
	{KEY_B3, 11000, SOL_W2},
	{KEY_E4, 11250, SOL_W2},
};
*/

// ============ THOMAS THE TANK ENGINE THEME SONG ===============

/*static SongEntry song[] = {
	// CHORUS
	{KEY_G4, 0, SOL_WREF},
	{KEY_A4, 250, SOL_WREF},
	{KEY_B4, 500, SOL_WREF},
	{KEY_C5, 750, SOL_WREF},
	{KEY_D5, 1250, SOL_WREF},
	{KEY_E5, 1500, SOL_WREF},
	{KEY_GS4, 2000, SOL_B1},

	{KEY_A4, 4000, SOL_W2},
	{KEY_F4, 4250, SOL_WREF},
	{KEY_A4, 4500, SOL_W2},
	{KEY_G4, 4750, SOL_WREF},

	{KEY_GS4, 5875, SOL_B1},
	{KEY_A4, 6000, SOL_WREF},
	{KEY_F4, 6250, SOL_W1},
	{KEY_F4, 6500, SOL_W1},
	{KEY_A4, 6750, SOL_WREF},
	{KEY_G4, 6875, SOL_WREF},

	{KEY_FS4, 7375, SOL_B1},
	{KEY_G4, 7500, SOL_WREF},
	{KEY_FS4, 7625, SOL_B1},
	{KEY_G4, 7750, SOL_WREF},
	{KEY_FS4, 7875, SOL_B1},
	{KEY_G4, 8000, SOL_WREF},
	{KEY_G4, 8500, SOL_WREF},

	{KEY_FS4, 9375, SOL_B1},
	{KEY_G4, 9500, SOL_WREF},
	{KEY_FS4, 9625, SOL_B1},
	{KEY_G4, 9750, SOL_WREF},
	{KEY_GS4, 10000, SOL_B2},
	{KEY_GS4, 10500, SOL_B2},

	{KEY_DS4, 11125, SOL_B1},
	{KEY_F4, 11500, SOL_WREF},
	{KEY_FS4, 11750, SOL_B2},
	{KEY_G4, 12000, SOL_WREF},
	{KEY_AS4, 12500, SOL_B2},
	{KEY_F4, 13000, SOL_W1},
	{KEY_G4, 13500, SOL_W1},
	{KEY_GS4, 14000, SOL_B1},

	// VERSE
	{KEY_GS3, 15250, SOL_B2},
	{KEY_G3, 15500, SOL_WREF},
	{KEY_FS3, 15750, SOL_B2},
	{KEY_F3, 16000, SOL_WREF},
	{KEY_F3, 16250, SOL_WREF},
	{KEY_AS3, 16500, SOL_B2},
	{KEY_AS3, 16750, SOL_B2},
	{KEY_CS4, 17000, SOL_B2},
	{KEY_CS4, 17125, SOL_B2},
	{KEY_F4, 17375, SOL_W2},

	{KEY_DS3, 18063, SOL_B1},
	{KEY_DS3, 18313, SOL_B1},
	{KEY_GS3, 18438, SOL_B2},
	{KEY_GS3, 18688, SOL_B2},
	{KEY_C4, 18938, SOL_WREF},
	{KEY_
	}
};
*/

uint8_t numNotes = sizeof(song) / sizeof(song[0]);

// ============ FUNCTION PROTOTYPES ============
//void FSM(void);
NoteMapping MapNote(int32_t keyPosition_counts);
//void FireSolenoid(uint8_t solenoid);
//void SolenoidUpdate(void);
//void StopAllSolenoids(void);

// ============ NOTE MAPPING ============
// Given a key position in counts, finds the best solenoid and motor position
// Picks the solenoid that requires the least carriage movement
NoteMapping MapNote(int32_t keyPosition_counts) {
    NoteMapping result;

    int32_t offsets[5] = {OFFSET_W1, OFFSET_WREF, OFFSET_W2, OFFSET_B1, OFFSET_B2};
    uint8_t solIDs[5] = {SOL_W1, SOL_WREF, SOL_W2, SOL_B1, SOL_B2};

    int32_t bestDist = INT32_MAX;
    int best = 0;

    for (int i = 0; i < 5; i++) {
        int32_t wrefPos = keyPosition_counts - offsets[i];

        int32_t dist = labs(wrefPos - target_FSM);
        if (dist < bestDist) {
            bestDist = dist;
            best = i;
        }

    }

    result.motorTarget = keyPosition_counts - offsets[best];
    result.solenoid = solIDs[best];
    return result;
}

// ============ SOLENOID CONTROL ============
void FireSolenoid(uint8_t solenoid) {
    switch (solenoid) {
    case SOL_W1:
        HAL_GPIO_WritePin(SOLENOID_PORT, SOLENOID_5_PIN, GPIO_PIN_SET);
        break;
    case SOL_B1:
        HAL_GPIO_WritePin(SOLENOID_PORT, SOLENOID_4_PIN, GPIO_PIN_SET);
        break;
    case SOL_WREF:
        HAL_GPIO_WritePin(SOLENOID_PORT, SOLENOID_3_PIN, GPIO_PIN_SET);
        break;
    case SOL_B2:
        HAL_GPIO_WritePin(SOLENOID_PORT, SOLENOID_2_PIN, GPIO_PIN_SET);
        break;
    case SOL_W2:
        HAL_GPIO_WritePin(SOLENOID_PORT, SOLENOID_1_PIN, GPIO_PIN_SET);
        break;
    }
    solenoidActive = true;
    solenoidOffTime = HAL_GetTick() + SOLENOID_PULSE_MS;
}

// Call this every loop iteration to turn off solenoid after pulse duration
void SolenoidUpdate(void) {
    if (solenoidActive && HAL_GetTick() >= solenoidOffTime) {
        StopAllSolenoids();
        solenoidActive = false;
    }
}

void StopAllSolenoids(void) {
    HAL_GPIO_WritePin(SOLENOID_PORT, SOLENOID_1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SOLENOID_PORT, SOLENOID_2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SOLENOID_PORT, SOLENOID_3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SOLENOID_PORT, SOLENOID_4_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SOLENOID_PORT, SOLENOID_5_PIN, GPIO_PIN_RESET);
}

// ============ FINITE STATE MACHINE ============
void FSM(void) {
    uint32_t now = HAL_GetTick();
    SolenoidUpdate(); // always check if solenoid needs turning off

    switch (state) {
    case HOME:
        // Wait for limit switch to be pressed to start homing
        StopAllSolenoids();
        Home();
        if (homed) {
            limit_switch_pressed = now;
            state = WAIT;
        }
        break;

    case WAIT:
        // After the limit switch is pressed, wait 2 seconds before starting the song
    	__HAL_TIM_SET_COUNTER(&HTIM_ENCODER, 0);
        if (now - limit_switch_pressed >= 2000) {
            song_index = 0;
            song_start_time = now;
            enCtrl = true;
            state = MOVE;
        }
        break;

    case MOVE:
    {
        int32_t offsets[6] = {0, OFFSET_W1, OFFSET_WREF, OFFSET_W2, OFFSET_B1, OFFSET_B2};
        int32_t keyPos = song[song_index].key_counts;
        target_FSM = keyPos - offsets[song[song_index].solenoid];

        if (done_move) {
            done_move = false;
            FireSolenoid(song[song_index].solenoid);
            song_index++;
            state = PLAY;
        }
        break;
    }

    case PLAY:
        if (solenoidActive) {
            break;
        }
        if (now - song_start_time >= song[song_index].time_ms) {
            //FireSolenoid(song[song_index].solenoid);
        	//FireSolenoid(SOL_W1);
        	//song_index++;

            if (song_index >= numNotes) {
                state = DONE;
            }
            else {
                // Same position if next note's motor target matches current
                int32_t offsets[6] = {0, OFFSET_W1, OFFSET_WREF, OFFSET_W2, OFFSET_B1, OFFSET_B2};
                int32_t nextKey = song[song_index].key_counts;
                int32_t nextTarget = nextKey - offsets[song[song_index].solenoid];

                if (nextTarget == target_FSM) {
                    state = PLAY;
                }
                else {
                	state = MOVE;
                }
            }
        }
        break;

    case DONE:
        // Song finished, wait for reset
        if (reset) state = HOME;
        break;

    default:
        break;
    }
}
