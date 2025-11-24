#include <Arduino.h>
#include "sensors.h"
#include "actuators.h"
#include "fsm.h"

void setup() {
    Serial.begin(115200);
    Serial.println("\n[System Boot] Initializing Access Control...");

    // 1. Init Hardware
    Sensors_Init();
    Actuators_Init();

    // 2. Init Logic
    FSM_Init();

    Serial.println("[System Boot] Ready. Waiting for command...");
}

void loop() {
    // The main loop simply ticks the state machine
    FSM_Run();
    
    // Small delay to prevent CPU hogging
    delay(50);
}