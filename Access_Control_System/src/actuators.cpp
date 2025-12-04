#include "actuators.h"
#include "fsm.h" // Include FSM header for state access

Servo lockServo;

void Actuators_Init() {
    lockServo.attach(PIN_SERVO);
    Actuators_SetLock(true); // Default to locked on boot

    // Initialize LED pins
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(RED_LED_PIN, OUTPUT);
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, LOW);
}

void Actuators_SetLock(bool lockState) {
    if (lockState) {
        lockServo.write(ANGLE_LOCKED);
        Serial.println(">> LOCK: ENGAGED");
    } else {
        lockServo.write(ANGLE_UNLOCKED);
        Serial.println(">> LOCK: OPEN");
    }
    Actuators_UpdateLEDs(); // Update LEDs whenever lock state changes
}

void Actuators_TriggerAlarm() {
    // Placeholder for Buzzer logic
    // digitalWrite(PIN_BUZZER, HIGH);
    Serial.println("!!!! ALARM TRIGGERED !!!!");
    Actuators_UpdateLEDs(); // Update LEDs to reflect alarm state
}

void Actuators_UpdateLEDs() {
    switch (currentState) {
        case STATE_DISARMED:
            digitalWrite(GREEN_LED_PIN, HIGH);
            digitalWrite(RED_LED_PIN, LOW);
            break;
        case STATE_ARMED:
            digitalWrite(GREEN_LED_PIN, LOW);
            digitalWrite(RED_LED_PIN, LOW); // Or a dim "breathing" red LED
            break;
        case STATE_ALARM_MOTION:
        case STATE_ALARM_TAMPER:
            digitalWrite(GREEN_LED_PIN, LOW);
            digitalWrite(RED_LED_PIN, HIGH); // Blinking might be better here
            break;
    }
}
