#include <SPI.h>
#include <LoRa.h>
#include "mbedtls/aes.h"

// ==========================================
// 1. PIN DEFINITIONS (Receiver Board)
// ==========================================
#define SCK_PIN     5
#define MISO_PIN    19
#define MOSI_PIN    27
#define SS_PIN      18
#define RST_PIN     14 // Use 14 for V1.3
#define DI0_PIN     26

#define BUZZER_PIN  21 
#define BIOMETRIC_PIN 13 // Touch Pin (T4 on ESP32)

// ==========================================
// 2. CONFIGURATION
// ==========================================
const char* adminUser = "admin";
const char* adminPass = "password";
const int TOUCH_THRESHOLD = 50;

// Security State
int loginAttempts = 0;
unsigned long lockoutStartTime = 0;
const unsigned long LOCKOUT_DURATION = 60000; // 60 seconds
bool isLockedOut = false;

// Console Interface State
enum ConsolePage {
  PAGE_LOCKED,
  PAGE_LOGIN_USER,
  PAGE_LOGIN_PASS,
  PAGE_BIO_CHECK,
  PAGE_ADMIN_MENU,
  PAGE_LIVE_FEED
};
ConsolePage currentPage = PAGE_LOCKED;
String tempUser = "";
String tempPass = "";
String inputBuffer = ""; 

unsigned char aesKey[16] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F};

// ==========================================
// 3. HELPER FUNCTIONS
// ==========================================

void triggerAlarmSound() {
  Serial.println(">>> ACTIVATING BUZZER <<<");
  // 3 Short Beeps - Manual Generation to avoid LEDC/tone() errors
  for(int k=0; k<3; k++) {
    // Generate 2kHz tone manually for 150ms
    unsigned long start = millis();
    while (millis() - start < 150) {
      digitalWrite(BUZZER_PIN, HIGH);
      delayMicroseconds(250); // 250us HIGH
      digitalWrite(BUZZER_PIN, LOW);
      delayMicroseconds(250); // 250us LOW = 500us period = 2kHz
    }
    // Silence for 150ms
    digitalWrite(BUZZER_PIN, LOW);
    delay(150);
  }
}

// AES Decrypt Incoming Message
String decryptMessage(String hexString) {
  mbedtls_aes_context aes;
  mbedtls_aes_init(&aes);
  mbedtls_aes_setkey_dec(&aes, aesKey, 128); 

  int inputLen = hexString.length() / 2;
  unsigned char input[inputLen];
  unsigned char output[inputLen];

  // Convert hex string to byte array
  for (int i = 0; i < inputLen; i++) {
    char byteChars[3] = {hexString[2*i], hexString[2*i+1], '\0'};
    input[i] = (unsigned char) strtol(byteChars, NULL, 16);
  }
  // Decrypt in 16-byte blocks
  for (int i = 0; i < inputLen; i += 16) {
    mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_DECRYPT, input + i, output + i);
  }
  mbedtls_aes_free(&aes);

  String plainText = "";
  for (int i = 0; i < inputLen; i++) {
    plainText += (char)output[i];
  }
  return plainText; 
}

// Simulate an incoming packet event
void runSimulation() {
  Serial.println("\n[DEMO] Simulating Incoming LoRa Packet...");
  delay(1000);
  
  // Fake Encrypted Payload (Pre-calculated hex for "FORCED_ENTRY")
  String fakeEncrypted = "A1B2C3D4E5F600112233445566778899"; 
  Serial.print("[RX] RSSI: -45 dBm | Payload: ");
  Serial.println(fakeEncrypted);
  delay(500);
  
  Serial.println("[SECURE] Decrypting payload with AES-128...");
  delay(500);
  
  Serial.println(">> ALERT TYPE: FORCED_ENTRY");
  Serial.println(">> STATUS: DANGER - TRIGGERING ALARM");
  
  triggerAlarmSound();
  
  Serial.println("[DEMO] Simulation Complete.\n");
}

// ==========================================
// 4. CONSOLE INTERFACE LOGIC
// ==========================================

void showPage() {
  Serial.println("\n--------------------------------");
  switch (currentPage) {
    case PAGE_LOCKED:
      Serial.println("       SYSTEM LOCKED");
      Serial.println("--------------------------------");
      if (isLockedOut) {
         Serial.print("TOO MANY ATTEMPTS. WAIT ");
         Serial.print((LOCKOUT_DURATION - (millis() - lockoutStartTime))/1000);
         Serial.println("s");
      } else {
         Serial.println("Type 'login' to authenticate.");
      }
      break;
      
    case PAGE_LOGIN_USER:
      Serial.println("ENTER USERNAME:");
      break;
      
    case PAGE_LOGIN_PASS:
      Serial.println("ENTER PASSWORD:");
      break;
      
    case PAGE_BIO_CHECK:
      Serial.println("BIOMETRIC VERIFICATION REQUIRED");
      Serial.println("Please hold the Touch Sensor (Wire)...");
      Serial.println("...and press ENTER to verify.");
      break;
      
    case PAGE_ADMIN_MENU:
      Serial.println("       ADMIN DASHBOARD");
      Serial.println("--------------------------------");
      Serial.println("1. View Live Decrypted Feed");
      Serial.println("2. Logout");
      Serial.println("3. Simulate Alarm (Demo Mode)"); 
      Serial.println("\nEnter choice (1-3):");
      break;
      
    case PAGE_LIVE_FEED:
      Serial.println(">> LIVE FEED ACTIVE <<");
      Serial.println("Listening for LoRa packets...");
      Serial.println("[Type 'b' to go back]");
      break;
  }
}
// Handle User Input Based on Current Page
void handleInput(String input) {
  input.trim(); 
  
  if (isLockedOut) {
    if (millis() - lockoutStartTime > LOCKOUT_DURATION) {
      isLockedOut = false;
      loginAttempts = 0;
      Serial.println("[Lockout Expired]");
      currentPage = PAGE_LOCKED;
      showPage();
    } else {
      return; 
    }
  }

  switch (currentPage) {
    case PAGE_LOCKED:
      if (input.equalsIgnoreCase("login")) {
        currentPage = PAGE_LOGIN_USER;
        showPage();
      } else {
        Serial.println("Invalid command. Type 'login'.");
      }
      break;

    case PAGE_LOGIN_USER:
      tempUser = input;
      currentPage = PAGE_LOGIN_PASS;
      showPage();
      break;

    case PAGE_LOGIN_PASS:
      tempPass = input;
      currentPage = PAGE_BIO_CHECK;
      showPage();
      break;

    case PAGE_BIO_CHECK:
      {
        int bioVal = touchRead(BIOMETRIC_PIN);
        Serial.print("Reading Sensor... Value: "); Serial.println(bioVal);
        
        if (tempUser == adminUser && tempPass == adminPass && bioVal < TOUCH_THRESHOLD) {
          Serial.println(">>> ACCESS GRANTED <<<");
          loginAttempts = 0;
          currentPage = PAGE_ADMIN_MENU;
        } else {
          loginAttempts++;
          Serial.println(">>> ACCESS DENIED (Bad Creds or No Touch) <<<");
          Serial.print("Attempts: "); Serial.print(loginAttempts); Serial.println("/3");
          
          if (loginAttempts >= 3) {
            isLockedOut = true;
            lockoutStartTime = millis();
            currentPage = PAGE_LOCKED;
          } else {
            currentPage = PAGE_LOCKED; 
          }
        }
        showPage();
      }
      break;

    case PAGE_ADMIN_MENU:
      if (input == "1") {
        currentPage = PAGE_LIVE_FEED;
        showPage();
      } else if (input == "2") {
        Serial.println("Logging out...");
        currentPage = PAGE_LOCKED;
        showPage();
      } else if (input == "3") {
        runSimulation();
        showPage(); // Return to menu after sim
      } else {
        Serial.println("Invalid Option.");
        showPage();
      }
      break;

    case PAGE_LIVE_FEED:
      if (input.equalsIgnoreCase("b")) {
        currentPage = PAGE_ADMIN_MENU;
        showPage();
      }
      break;
  }
}

// ==========================================
// 5. SETUP & LOOP
// ==========================================
void setup() {
  delay(5000); 

  Serial.begin(115200);
  while (!Serial);

  Serial.println("=== LoRa Security Gateway ===");
  
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW); 

  // Setup LoRa (Receiver Mode)
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  LoRa.setPins(SS_PIN, RST_PIN, DI0_PIN);
  if (!LoRa.begin(915E6)) { 
    Serial.println("Error: LoRa Init Failed!");
    // We don't halt here anymore so you can still use the Admin Console Demo
  } else {
    Serial.println("LoRa Radio Initialized.");
  }
  
  Serial.println("Gateway Listening...");
  
  // Show Initial Page
  showPage();
}

void loop() {
  // 1. Handle Serial Input (Interactive Menu with Echo)
  while (Serial.available()) {
    char c = (char)Serial.read();
    
    if (c == '\b' || c == 127) {
      if (inputBuffer.length() > 0) {
        inputBuffer.remove(inputBuffer.length() - 1);
        Serial.print("\b \b");
      }
      continue;
    }

    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        Serial.println(); 
        handleInput(inputBuffer);
        inputBuffer = ""; 
      }
    } 
    else if (c >= 32 && c <= 126) {
      if (currentPage == PAGE_LOGIN_PASS) {
        Serial.print('*');
      } else {
        Serial.print(c); 
      }
      inputBuffer += c;
    }
  }

  // 2. Handle LoRa Packets (If Antenna Existed)
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String encryptedHex = "";
    while (LoRa.available()) {
      encryptedHex += (char)LoRa.read();
    }

    if (currentPage == PAGE_LIVE_FEED) {
      String decrypted = decryptMessage(encryptedHex);
      Serial.print("[ADMIN ALERT] "); Serial.println(decrypted);
      
      if (decrypted.indexOf("ALARM") >= 0 || 
          decrypted.indexOf("TRESPASSING") >= 0 || 
          decrypted.indexOf("FORCED") >= 0) {
         triggerAlarmSound();
      }
    }
  }
  
  // Refresh Lockout Timer check
  if (isLockedOut && (millis() - lockoutStartTime > LOCKOUT_DURATION)) {
      isLockedOut = false;
      loginAttempts = 0;
      Serial.println("\n[LOCKOUT EXPIRED - PRESS ENTER]");
      currentPage = PAGE_LOCKED;
      showPage();
  }
}