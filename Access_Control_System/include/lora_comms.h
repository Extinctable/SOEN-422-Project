#ifndef LORA_COMMS_H
#define LORA_COMMS_H

#include <Arduino.h>

// TTGO LoRa32 V1 Pin Definitions (Standard)
#define LORA_SS      18
#define LORA_RST     14
#define LORA_DIO0    26
#define LORA_BAND    915E6  // 915MHz for North America, use 868E6 for Europe

// Function Declarations
void Lora_Init();
void Lora_SendAlarm(String alarmType, int sensorValue);

#endif