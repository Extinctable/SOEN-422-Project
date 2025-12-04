#include "sd_card.h"
#include <SPI.h>
#include <SD.h>

// Define the log file name
const char* logFileName = "/security_log.txt";

void SD_Card_Init() {
    Serial.print("[SD Card] Initializing... ");
    
    // The SD library uses the standard SPI pins for your board.
    // You only need to specify the Chip Select (CS) pin.
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("Initialization FAILED!");
        Serial.println("[SD Card] - Check wiring, power, and card format.");
        // Depending on requirements, you might halt here or continue without logging.
        return;
    }
    
    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("OK!\n[SD Card] Size: %lluMB\n", cardSize);
}

void SD_Card_LogEvent(String eventType, String eventDetails) {
    // Open the log file in append mode
    File logFile = SD.open(logFileName, FILE_APPEND);
    
    if (logFile) {
        // Construct the log entry with a simple timestamp
        String logEntry = String(millis()) + " | " + eventType + " | " + eventDetails;
        
        logFile.println(logEntry);
        logFile.close(); // Close the file to save the data
        
        Serial.println("[SD Card] Event logged: " + logEntry);
    } else {
        Serial.println("[SD Card] Error opening log file!");
    }
}
