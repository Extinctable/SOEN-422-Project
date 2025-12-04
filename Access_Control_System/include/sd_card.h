#ifndef SD_CARD_H
#define SD_CARD_H

#include <Arduino.h>

// --- Pin Definition ---
// NOTE: This CS pin is a common choice, but may need to be adjusted
// based on your specific TTGO LoRa board wiring for the SD module.
#define SD_CS_PIN 4

// --- Function Declarations ---

// Initializes the MicroSD card module.
void SD_Card_Init();

// Logs a formatted security event to a file on the SD card.
// Example: SD_Card_LogEvent("ALARM_MOTION", "Motion Detected");
void SD_Card_LogEvent(String eventType, String eventDetails);

#endif
