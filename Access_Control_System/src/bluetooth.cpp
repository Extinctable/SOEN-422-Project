#include "bluetooth.h"
#include "fsm.h"
#include "actuators.h"
#include "sd_card.h"

BluetoothSerial SerialBT;

// --- Challenge-Response Configuration ---
const String SECRET_KEY = "1234"; // Replace with a secure key management strategy
String challenge = "";

void generateChallenge() {
    challenge = String(random(1000, 9999));
}

void Bluetooth_Init() {
    if (!SerialBT.begin("AC_SYS_REMOTE")) {
        Serial.println("!!! Bluetooth: An error occurred initializing Bluetooth");
    } else {
        Serial.println("[Bluetooth] Ready. Pair to disarm.");
    }
}

void Bluetooth_HandleConnection() {
    if (SerialBT.hasClient()) {
        Serial.println("[Bluetooth] Client Connected.");
        generateChallenge();
        SerialBT.println("--- Access Control System ---");
        SerialBT.println("STATUS: " + String(currentState == STATE_ARMED ? "ARMED" : "ALARM"));
        SerialBT.println("Enter Response for Challenge: " + challenge);
    }

    if (SerialBT.available()) {
        String response = SerialBT.readStringUntil('\n');
        response.trim();
        Serial.println("[Bluetooth] Received Response: " + response);

        // --- VERY Simple Validation (for demo purposes) ---
        // A real system would use HMAC or a more secure method.
        String expectedResponse = SECRET_KEY + challenge; 

        if (response == expectedResponse) {
            SerialBT.println("ACCESS GRANTED. Disarming system.");
            Serial.println("[Bluetooth] Correct Response. Disarming.");
            SD_Card_LogEvent("DISARM_BT", "System disarmed via Bluetooth.");
            currentState = STATE_DISARMED;
            Actuators_SetLock(false);
            Actuators_UpdateLEDs();
            SerialBT.disconnect();
        } else {
            SerialBT.println("ACCESS DENIED. Incorrect response.");
            Serial.println("[Bluetooth] Incorrect Response.");
            SerialBT.disconnect();
        }
    }
}
