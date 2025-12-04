#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <Arduino.h>
#include "BluetoothSerial.h"

// Function Declarations
void Bluetooth_Init();
void Bluetooth_HandleConnection(); // Manages the challenge-response flow

#endif
