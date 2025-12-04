#ifndef ACTUATORS_H
#define ACTUATORS_H

#include <Arduino.h>
#include <ESP32Servo.h>

// Pin Definitions
#define PIN_SERVO      13
#define GREEN_LED_PIN  22
#define RED_LED_PIN    23
#define PIN_BUZZER     15 // Optional

// Servo Angles
#define ANGLE_LOCKED   0
#define ANGLE_UNLOCKED 90

// Function Declarations
void Actuators_Init();
void Actuators_SetLock(bool lockState);
void Actuators_TriggerAlarm();
void Actuators_UpdateLEDs();

#endif
