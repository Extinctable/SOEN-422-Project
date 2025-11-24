#include "fsm.h"
#include "sensors.h"
#include "actuators.h"

// Initialize the global state
SystemState currentState = STATE_DISARMED;

// Variables specific to logic
int initialDoorPosition = 0;
const int TAMPER_THRESHOLD = 100;

void FSM_Init() {
    // Any logic-specific setup goes here
    currentState = STATE_DISARMED;
}

void FSM_HandleSerialCommands() {
    if (Serial.available()) {
        char cmd = Serial.read();
        
        // DISARM (D) or RESET (R)
        if ((cmd == 'D' || cmd == 'R') && currentState != STATE_DISARMED) {
            currentState = STATE_DISARMED;
            Actuators_SetLock(false); 
            Serial.println("\n>> CMD: SYSTEM DISARMED.");
        } 
        // ARM (A)
        else if (cmd == 'A' && currentState == STATE_DISARMED) {
            currentState = STATE_ARMED;
            initialDoorPosition = Sensors_ReadDoorPosition(); // Calibrate
            Actuators_SetLock(true);
            Serial.println("\n>> CMD: SYSTEM ARMED. Monitoring...");
        }
    }
}

void FSM_Run() {
    // 1. Always check for user commands first
    FSM_HandleSerialCommands();

    // 2. Execute State Logic
    switch (currentState) {
        
        case STATE_DISARMED:
            // Idle. Do nothing.
            break;

        case STATE_ARMED: {
            // Check Sensors
            int motion = Sensors_CheckMotion();
            int currentPos = Sensors_ReadDoorPosition();
            
            // Logic: Motion
            if (motion == HIGH) {
                currentState = STATE_ALARM_MOTION;
                Serial.println("\n!!! ALARM: MOTION DETECTED !!!");
            }
            
            // Logic: Tamper
            if (abs(currentPos - initialDoorPosition) > TAMPER_THRESHOLD) {
                currentState = STATE_ALARM_TAMPER;
                Serial.println("\n!!! ALARM: TAMPER DETECTED !!!");
            }
            break;
        }

        case STATE_ALARM_MOTION:
        case STATE_ALARM_TAMPER:
            // Alarm State behavior
            Actuators_TriggerAlarm();
            // We stay here until user types 'D' or 'R' (handled in HandleSerialCommands)
            break;
    }
}
