#include "actuators.h"

Servo lockServo;

void Actuators_Init() {
    lockServo.attach(PIN_SERVO);
    Actuators_SetLock(true); // Default to locked on boot
}

void Actuators_SetLock(bool lockState) {
    if (lockState) {
        lockServo.write(ANGLE_LOCKED);
        Serial.println(">> LOCK: ENGAGED"); // debug
    } else {
        lockServo.write(ANGLE_UNLOCKED);
        Serial.println(">> LOCK: OPEN"); // debug
    }
}

void Actuators_TriggerAlarm() {
    // Placeholder for Buzzer/LED logic
    // digitalWrite(PIN_BUZZER, HIGH);
}
