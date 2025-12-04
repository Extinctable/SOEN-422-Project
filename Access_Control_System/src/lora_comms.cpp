#include "lora_comms.h"
#include <SPI.h>
#include <LoRa.h>

void Lora_Init() {
    Serial.println("[LoRa] Initializing Radio...");
    
    // Configure pins for the TTGO board
    SPI.begin(5, 19, 27, 18); // SCK, MISO, MOSI, SS
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

    // Attempt to start LoRa engine
    if (!LoRa.begin(LORA_BAND)) {
        Serial.println("[LoRa] Initialization FAILED! Check wiring/power.");
        while (1); // Halt if radio fails (vital component)
    }
    
    // Optional: Set sync word to isolate network (0-0xFF)
    LoRa.setSyncWord(0xF3); 
    Serial.println("[LoRa] Radio Initialized OK!");
}

void Lora_SendAlarm(String alarmType, int sensorValue) {
    Serial.print("[LoRa] Transmitting Packet: ");
    
    // Create a simple payload: "ALARM_TYPE:VALUE"
    String payload = alarmType + ":" + String(sensorValue);
    Serial.println(payload);

    // Send Packet
    LoRa.beginPacket();
    LoRa.print(payload);
    LoRa.endPacket();
    
    Serial.println("[LoRa] Transmission Complete.");
}
