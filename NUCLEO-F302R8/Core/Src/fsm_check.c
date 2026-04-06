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

// ============ SOLENOID IDS ============
#define SOL_W1   1
#define SOL_WREF 2
#define SOL_W2   3
#define SOL_B1   4
#define SOL_B2   5

// ============ PHYSICAL MAPPING ============
// Calibrate this to your pulley: counts_per_rev / (diameter_mm * PI)
#define MM_TO_COUNTS (2797.0f / (2.0f * 12.5f * M_PI))

// Piano key positions in mm from C4 (home)
#define KEY_C4   0.0f
#define KEY_CS4  14.0f
#define KEY_D4   21.5f
#define KEY_DS4  37.5f
#define KEY_E4   43.0f
#define KEY_F4   64.5f
#define KEY_FS4  84.0f
#define KEY_G4   86.0f
#define KEY_GS4  107.5f
#define KEY_A4   107.5f
#define KEY_AS4  131.0f
#define KEY_B4   129.0f
#define KEY_C5   150.5f

// Solenoid offsets from W_ref in mm
#define OFFSET_W1    -45.0f
#define OFFSET_WREF   0.0f
#define OFFSET_W2    45.0f
#define OFFSET_B1   -10.75f
#define OFFSET_B2    10.75f

#define SOLENOID_PULSE_MS 100

// ============ TYPES ============
typedef struct {
    int32_t motorTarget;  // where to move W_ref (encoder counts)
    uint8_t solenoid;     // which solenoid to fire
} NoteMapping;

typedef struct {
    float key_mm;         // which key (use KEY_C4, KEY_FS4, etc.)
    uint16_t time_ms;     // when to play from song start
} SongEntry;

// ============ STATE VARIABLES ============
FSM_State state = HOME;
uint8_t song_index = 0;
uint16_t limit_switch_pressed = 0;
uint16_t target_FSM = 0;
//bool done_move = false;
bool reset = false;
bool enCtrl = false;

volatile bool solenoidActive = false;
uint32_t solenoidOffTime = 0;
uint32_t song_start_time = 0;
NoteMapping currentNote;

// ============ SONG DATA ============
// list of piano keys and when to play ms from song start
static SongEntry song[] = {
    //{KEY_C4,   0},
		/*
    {KEY_E4,   1000},
    {KEY_G4,   2000},
    {KEY_AS4,  3000},
    {KEY_C5,   4000},
    {KEY_AS4,  5000},
    {KEY_G4,   6000},
    {KEY_FS4,  7000},
    {KEY_E4,   8000},
    {KEY_DS4,  9000},
    {KEY_C4,   10000},
    */
		{KEY_F4, 1000}
};
uint8_t numNotes = sizeof(song) / sizeof(song[0]);

// ============ FUNCTION PROTOTYPES ============
//void FSM(void);
NoteMapping MapNote(float keyPosition_mm);
//void FireSolenoid(uint8_t solenoid);
//void SolenoidUpdate(void);
//void StopAllSolenoids(void);

// ============ NOTE MAPPING ============
// Given a key position in mm, finds the best solenoid and motor position
// Picks the solenoid that requires the least carriage movement
NoteMapping MapNote(float keyPosition_mm) {
    NoteMapping result;

    float offsets[5] = {OFFSET_W1, OFFSET_WREF, OFFSET_W2, OFFSET_B1, OFFSET_B2};
    uint8_t solIDs[5] = {SOL_W1, SOL_WREF, SOL_W2, SOL_B1, SOL_B2};

    float bestDist = 999999.0f;
    int best = 0;

    for (int i = 0; i < 5; i++) {
        float wrefPos = keyPosition_mm - offsets[i];

        float dist = fabsf(wrefPos * MM_TO_COUNTS - (float)target_FSM);
        if (dist < bestDist) {
            bestDist = dist;
            best = i;
        }

    }

    float wrefPos = keyPosition_mm - offsets[best];
    result.motorTarget = (int32_t)(wrefPos * MM_TO_COUNTS);
    result.solenoid = solIDs[best];
    return result;
}

// ============ SOLENOID CONTROL ============
void FireSolenoid(uint8_t solenoid) {
    switch (solenoid) {
    case SOL_W1:
        HAL_GPIO_WritePin(SOLENOID_PORT, SOLENOID_1_PIN, GPIO_PIN_SET);
        break;
    case SOL_WREF:
        HAL_GPIO_WritePin(SOLENOID_PORT, SOLENOID_2_PIN, GPIO_PIN_SET);
        break;
    case SOL_W2:
        HAL_GPIO_WritePin(SOLENOID_PORT, SOLENOID_3_PIN, GPIO_PIN_SET);
        break;
    case SOL_B1:
        HAL_GPIO_WritePin(SOLENOID_PORT, SOLENOID_4_PIN, GPIO_PIN_SET);
        break;
    case SOL_B2:
        HAL_GPIO_WritePin(SOLENOID_PORT, SOLENOID_5_PIN, GPIO_PIN_SET);
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
        // Map the current note to find motor target and solenoid
        currentNote = MapNote(song[song_index].key_mm);
        target_FSM = currentNote.motorTarget;

        if (done_move) {
            done_move = false;
            state = PLAY;
        }
        break;

    case PLAY:
        if (solenoidActive) {
            break;
        }
        if (now - song_start_time >= song[song_index].time_ms) {
            FireSolenoid(currentNote.solenoid);
            song_index++;

            if (song_index >= numNotes) {
                state = DONE;
            }
            else {
                NoteMapping nextNote = MapNote(song[song_index].key_mm);
                if (nextNote.motorTarget == currentNote.motorTarget) {
                    currentNote = nextNote;
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
