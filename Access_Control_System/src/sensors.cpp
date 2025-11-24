#include "sensors.h"

void Sensors_Init() {
    pinMode(PIN_IR_SENSOR, INPUT);
    pinMode(PIN_POTENTIOMETER, INPUT);
}

int Sensors_ReadDoorPosition() {
    return analogRead(PIN_POTENTIOMETER);
}

int Sensors_CheckMotion() {
    return digitalRead(PIN_IR_SENSOR);
}
