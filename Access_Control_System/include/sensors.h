#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

// Pin Definitions
#define PIN_POTENTIOMETER 34
#define PIN_IR_SENSOR     25

// Function Declarations
void Sensors_Init();
int Sensors_ReadDoorPosition();
int Sensors_CheckMotion();

#endif
