#ifndef FSM_H
#define FSM_H

#include <Arduino.h>

// --- STATE DEFINITIONS ---
enum SystemState {
    STATE_DISARMED,
    STATE_ARMED,
    STATE_ALARM_MOTION,
    STATE_ALARM_TAMPER
};

// Global State Variable (extern allows main.cpp to see it if needed)
extern SystemState currentState;

// Function Declarations
void FSM_Init();
void FSM_Run();     // The heart of the state machine
void FSM_HandleSerialCommands(); // Handles user input (A, D, R)

#endif