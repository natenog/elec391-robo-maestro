/**
  ******************************************************************************
  * @file           : fsm.c
  * @brief          : Finite state machine for movement and solenoid pressing logic
  * @author         : Yousef Mohamed
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
#define MM_TO_COUNTS 33.65f
#define HOME_TO_C4 6655.0f

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
#define KEY_C6	 280.0f

// Solenoid offsets from W_ref in mm
#define OFFSET_W1    -45.0f
#define OFFSET_WREF   0.0f
#define OFFSET_W2    45.0f
#define OFFSET_B1   -10.75f
#define OFFSET_B2    10.75f

#define SOLENOID_PULSE_MS 100

// Homing safety: if homing takes longer than this, give up and retry
#define HOME_TIMEOUT_MS   15000

// ============ TYPES ============
typedef struct {
    int32_t motorTarget;
    uint8_t solenoid;
} NoteMapping;

typedef struct {
    float key_mm;
    uint16_t time_ms;
    uint8_t solenoid;
} SongEntry;

// ============ STATE VARIABLES ============
FSM_State state = HOME;
uint8_t song_index = 0;
uint16_t limit_switch_pressed = 0;
int32_t target_FSM = 0;
bool limit_switch = false;
bool reset = false;
bool enCtrl = false;

volatile bool solenoidActive = false;
uint32_t solenoidOffTime = 0;
uint32_t song_start_time = 0;
NoteMapping currentNote;

// ============ SONG DATA ============
static SongEntry song[] = {
    {KEY_C4,   0,     SOL_W1},
    {KEY_D4,   1500,  SOL_WREF},
    {KEY_G4,   2500,  SOL_WREF},
    {KEY_AS4,  3500,  SOL_B2},
    {KEY_C5,   4500,  SOL_W2},
    {KEY_AS4,  5500,  SOL_B2},
    {KEY_G4,   6500,  SOL_WREF},
    {KEY_FS4,  7500,  SOL_B1},
    {KEY_E4,   8500,  SOL_WREF},
    {KEY_DS4,  9500,  SOL_B1},
    {KEY_C4,   10500, SOL_W1},
};

uint8_t numNotes = sizeof(song) / sizeof(song[0]);

// ============ FUNCTION PROTOTYPES ============
NoteMapping MapNote(float keyPosition_mm);

// ============ NOTE MAPPING ============
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
    case SOL_B1:
        HAL_GPIO_WritePin(SOLENOID_PORT, SOLENOID_2_PIN, GPIO_PIN_SET);
        break;
    case SOL_WREF:
        HAL_GPIO_WritePin(SOLENOID_PORT, SOLENOID_3_PIN, GPIO_PIN_SET);
        break;
    case SOL_W2:
        HAL_GPIO_WritePin(SOLENOID_PORT, SOLENOID_4_PIN, GPIO_PIN_SET);
        break;
    case SOL_B2:
        HAL_GPIO_WritePin(SOLENOID_PORT, SOLENOID_5_PIN, GPIO_PIN_SET);
        break;
    }
    solenoidActive = true;
    solenoidOffTime = HAL_GetTick() + SOLENOID_PULSE_MS;
}

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
    static uint32_t home_state_entry = 0;

    uint32_t now = HAL_GetTick();
    SolenoidUpdate();

    switch (state) {
    case HOME:
        // Track time spent in HOME for the timeout safety
        if (home_state_entry == 0) {
            home_state_entry = now;
        }

        StopAllSolenoids();
        Home();

        if (homed) {
            limit_switch_pressed = now;
            home_state_entry = 0;
            state = WAIT;
        }
        else if (now - home_state_entry > HOME_TIMEOUT_MS) {
            // Homing has been running too long. Stop the motor and retry.
            // Without UART we can't print the failure, but at least we
            // won't run the motor against a wall forever.
            MotorSetSpeedPercentCh1(0);
            MotorSetSpeedPercentCh2(0);
            home_state_entry = 0;
            // Stay in HOME — Home() will be called again next FSM tick
        }
        break;

    case WAIT:
        // Encoder is already zeroed in Home() now, so we don't need to do it here
        if (now - limit_switch_pressed >= 2000) {
            song_index = 0;
            song_start_time = now;
            enCtrl = true;
            state = MOVE;
        }
        break;

    case MOVE:
    {
        float offsets[6] = {0, OFFSET_W1, OFFSET_WREF, OFFSET_W2, OFFSET_B1, OFFSET_B2};
        float keyPos = song[song_index].key_mm;
        float wrefPos = keyPos - offsets[song[song_index].solenoid];
        target_FSM = (int32_t)((wrefPos * MM_TO_COUNTS) + HOME_TO_C4);

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
            if (song_index >= numNotes) {
                state = DONE;
            }
            else {
                float offsets[6] = {0, OFFSET_W1, OFFSET_WREF, OFFSET_W2, OFFSET_B1, OFFSET_B2};
                float nextWref = song[song_index].key_mm - offsets[song[song_index].solenoid];
                int32_t nextTarget = (int32_t)((nextWref * MM_TO_COUNTS) + HOME_TO_C4);

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
        if (reset) state = HOME;
        break;

    default:
        break;
    }
}
