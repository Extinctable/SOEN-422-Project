#include "fsm.h"
#include "sensors.h"
#include "actuators.h"
#include "lora_comms.h"
#include "sd_card.h"

// Initialize the global state
SystemState currentState = STATE_DISARMED;

// Variables specific to logic
int initialDoorPosition = 0;
const int TAMPER_THRESHOLD = 100;

void FSM_Init() {
    // Any logic-specific setup goes here
    currentState = STATE_DISARMED;
    Actuators_UpdateLEDs(); // Set initial LED state
}

void FSM_HandleSerialCommands() {
    if (Serial.available()) {
        char cmd = Serial.read();
        
        // DISARM (D) or RESET (R)
        if ((cmd == 'D' || cmd == 'R') && currentState != STATE_DISARMED) {
            currentState = STATE_DISARMED;
            Actuators_SetLock(false); 
            Serial.println("\n>> CMD: SYSTEM DISARMED.");
            SD_Card_LogEvent("DISARM", "System disarmed via serial.");
        } 
        // ARM (A)
        else if (cmd == 'A' && currentState == STATE_DISARMED) {
            currentState = STATE_ARMED;
            initialDoorPosition = Sensors_ReadDoorPosition(); // Calibrate
            Actuators_SetLock(true);
            Serial.println("\n>> CMD: SYSTEM ARMED. Monitoring...");
            SD_Card_LogEvent("ARM", "System armed via serial.");
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
                SD_Card_LogEvent("ALARM_MOTION", "Motion detected by IR sensor.");
                
                // TRIGGER LORA TRANSMISSION
                Lora_SendAlarm("MOTION", motion); 
            }
            
            // Logic: Tamper
            if (abs(currentPos - initialDoorPosition) > TAMPER_THRESHOLD) {
                currentState = STATE_ALARM_TAMPER;
                Serial.println("\n!!! ALARM: TAMPER DETECTED !!!");
                SD_Card_LogEvent("ALARM_TAMPER", "Tamper detected on door sensor.");

                // TRIGGER LORA TRANSMISSION
                Lora_SendAlarm("TAMPER", currentPos);
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
