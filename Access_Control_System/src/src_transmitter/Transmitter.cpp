#include <SPI.h>
#include <LoRa.h>
#include <ESP32Servo.h> 
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "mbedtls/aes.h"

// ==========================================
// 1. PIN DEFINITIONS & CONSTANTS
// ==========================================
#define GREEN_LED_PIN      22
#define RED_LED_PIN        23  
#define SERVO_PIN          13 
#define POTENTIOMETER_PIN  34 

// TTGO LoRa32 T3 v1.6.1 Specific Pins
#define SCK_PIN            5
#define MISO_PIN           19
#define MOSI_PIN           27
#define SS_PIN             18
#define RST_PIN            23   
#define DI0_PIN            26

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define TAMPER_THRESHOLD   70    
#define ALARM_INTERVAL     3000  
#define POT_DEADBAND       30 
#define MAX_AUTH_ATTEMPTS  3 

// ==========================================
// 2. GLOBAL VARIABLES
// ==========================================
Servo lockServo;
BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
std::string rxValue = ""; 

// Security & Crypto
unsigned char aesKey[16] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F};
String currentChallenge = "";
int initialDoorPos = 0;
bool loraInitialized = false;

// Input Flags
bool bleReqDisarm = false;
bool bleReqArm = false;
bool bleReqOpen = false;
bool bleReqClose = false; 
bool bleReqDialOn = false; 
int  bleAuthStatus = 0; 

// State Variables
int failedAttempts = 0;       
bool isManuallyOpened = false;

// Timers & Servo State
unsigned long lastAlarmTx = 0;
unsigned long lastLedBlink = 0;
bool ledState = false;
int currentServoAngle = 0;
int targetServoAngle = 0;
unsigned long lastServoMove = 0;
bool potFollowMode = true; 
int lastStablePotVal = -1;

// State Machine States
enum SystemState {
  STATE_LOCKING,
  STATE_ARMED,
  STATE_ALARM,
  STATE_WAITING_2FA,
  STATE_DISARMED
} currentState; 

// ==========================================
// 3. HELPER FUNCTIONS
// ==========================================

// Read Potentiometer with Averaging
int readPotAvg() {
  long sum = 0;
  for(int i=0; i<10; i++) {
    sum += analogRead(POTENTIOMETER_PIN);
    delay(1); 
  }
  return (int)(sum / 10);
}

// Smoothly update servo position towards target
void updateServoPosition() {
  // Move servo every 15ms for smoothness
  if (millis() - lastServoMove > 15) { 
    // Move towards target
    if (currentServoAngle < targetServoAngle) {
      currentServoAngle++;
      lockServo.write(currentServoAngle);
      lastServoMove = millis();
    } 
    // Added '=' to ensure it stops exactly at target
    else if (currentServoAngle > targetServoAngle) {
      currentServoAngle--;
      lockServo.write(currentServoAngle);
      lastServoMove = millis();
    }
  }
}

// Set servo target angle
void setServoTarget(bool open) {
  if (open) targetServoAngle = 180;
  else targetServoAngle = 0;
}

// AES Encrypt Payload
String encryptPayload(String plainText) {
  mbedtls_aes_context aes;
  mbedtls_aes_init(&aes);
  mbedtls_aes_setkey_enc(&aes, aesKey, 128);
  
  int inputLen = plainText.length();
  int paddedLen = ((inputLen / 16) + 1) * 16;
  unsigned char input[paddedLen];
  unsigned char output[paddedLen];
  
  for(int i=0; i<paddedLen; i++) input[i] = (i<inputLen) ? (unsigned char)plainText[i] : ' ';
  for(int i=0; i<paddedLen; i+=16) mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, input+i, output+i);
  mbedtls_aes_free(&aes);

  String hexString = "";
  for(int i=0; i<paddedLen; i++) {
    if(output[i] < 16) hexString += "0";
    hexString += String(output[i], HEX);
  }
  return hexString;
}

// Send LoRa Alert Message
void sendLoRaAlert(String message) {
  // DEMO MODIFICATION: Actual radio TX is commented out to prevent hanging.
  // The system will only print to Serial.
  /*
  if (!loraInitialized) {
    if (message != "ALARM_ACTIVE") Serial.println("[LoRa Error] Radio skipped TX: " + message);
    return; 
  }
  
  String encrypted = encryptPayload(message);
  LoRa.beginPacket();
  LoRa.print(encrypted);
  LoRa.endPacket(); 
  */
  
  Serial.println("[LoRa TX SIMULATED] " + message);
}

// BLE Server Callbacks
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { 
      deviceConnected = true; 
      Serial.println("BLE Device Connected");
    };
    void onDisconnect(BLEServer* pServer) { 
      deviceConnected = false; 
      Serial.println("BLE Device Disconnected");
      pServer->getAdvertising()->start();
    }
};

// BLE Characteristic Callbacks
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxVal = pCharacteristic->getValue();
      if (rxVal.length() > 0) rxValue = rxVal;
    }
};

// Send Bluetooth Message
void sendBluetoothMessage(String msg) {
  if (deviceConnected) {
    pTxCharacteristic->setValue((uint8_t*)msg.c_str(), msg.length());
    pTxCharacteristic->notify();
    Serial.println("[BLE TX] " + msg);
  }
}

// Check for Tamper Condition
bool checkTamper() {
  int currentPos = readPotAvg();
  // Using abs() with explicit cast to handle potential overflow issues
  if (abs((long)currentPos - (long)initialDoorPos) > TAMPER_THRESHOLD) {
    return true;
  }
  return false;
}

// ==========================================
// 4. TICK FUNCTION (State Machine)
// ==========================================
void TickFct_Security() {
  
  // --- A. TRANSITIONS SWITCH ---
  switch (currentState) {
    
    case STATE_LOCKING:
      if (currentServoAngle == 0) {
        delay(1000); 
        initialDoorPos = readPotAvg(); 
        Serial.println("Servo Locked. Sensors Calibrated. Entering ARMED Mode.");
        Serial.println("Reference Pot Value: " + String(initialDoorPos));
        
        failedAttempts = 0; 
        currentState = STATE_ARMED;
        sendBluetoothMessage("STATUS: ARMED");
      }
      break;

    case STATE_ARMED:
      // Check for Tamper
      if (checkTamper()) {
        Serial.println("Tamper Detected: Pot moved significantly");
        sendLoRaAlert("FORCED_ENTRY");
        currentState = STATE_ALARM;
      }
      // Check for Disarm Request
      else if (bleReqDisarm) {
        int challenge = random(1000, 9999);
        currentChallenge = String(challenge);
        sendBluetoothMessage("CHALLENGE:" + currentChallenge);
        
        bleReqDisarm = false; 
        failedAttempts = 0; 
        currentState = STATE_WAITING_2FA;
      }
      break;

    case STATE_ALARM:
      // Check for Disarm Request
      if (bleReqDisarm) {
        int challenge = random(1000, 9999);
        currentChallenge = String(challenge);
        sendBluetoothMessage("CHALLENGE:" + currentChallenge);
        
        bleReqDisarm = false;
        failedAttempts = 0;
        currentState = STATE_WAITING_2FA;
      }
      break;

    case STATE_WAITING_2FA:
      if (bleAuthStatus == 1) { // Auth Success
        sendBluetoothMessage("AUTH_SUCCESS");
        bleAuthStatus = 0; 
        failedAttempts = 0; 
        potFollowMode = true; 
        currentState = STATE_DISARMED;
      }
      else if (bleAuthStatus == 2) { // Auth Fail
        failedAttempts++;
        bleAuthStatus = 0; 
        
        if (failedAttempts >= MAX_AUTH_ATTEMPTS) {
          Serial.println("Max Auth Attempts Exceeded!");
          sendBluetoothMessage("ALARM: TOO MANY ATTEMPTS");
          sendLoRaAlert("UNAUTHORIZED_ACCESS_ATTEMPT");
          currentState = STATE_ALARM;
        } else {
          Serial.print("Auth Failed. Attempts: "); 
          Serial.println(failedAttempts);
          sendBluetoothMessage("AUTH_FAILED (" + String(failedAttempts) + "/3)");
        }
      }
      break;
    
    case STATE_DISARMED:
      if (bleReqArm) {
        Serial.println("Locking System...");
        bleReqArm = false; 
        currentState = STATE_LOCKING; 
      }
      else if (bleReqOpen) {
        potFollowMode = false; 
        targetServoAngle = 180; 
        bleReqOpen = false;
        Serial.println("Manual OPEN. Pot Control DISABLED.");
      }
      else if (bleReqClose) {
        potFollowMode = false; 
        targetServoAngle = 0; 
        bleReqClose = false;
        Serial.println("Manual CLOSE. Pot Control DISABLED.");
      }
      else if (bleReqDialOn) {
        potFollowMode = true; 
        lastStablePotVal = readPotAvg(); 
        bleReqDialOn = false;
        Serial.println("DIAL ON. Pot Control ENABLED.");
      }
      break;
      
    default:
      currentState = STATE_ARMED;
      break;
  }

  // --- B. STATE ACTIONS SWITCH ---
  switch (currentState) {
    
    case STATE_LOCKING:
      setServoTarget(false); 
      digitalWrite(GREEN_LED_PIN, HIGH); 
      digitalWrite(RED_LED_PIN, LOW);
      break;

    case STATE_ARMED:
      digitalWrite(GREEN_LED_PIN, HIGH);
      digitalWrite(RED_LED_PIN, LOW);
      break;

    case STATE_ALARM:
      digitalWrite(GREEN_LED_PIN, LOW); 
      
      // Blink Red LED
      if (millis() - lastLedBlink > 250) {
        ledState = !ledState;
        digitalWrite(RED_LED_PIN, ledState);
        lastLedBlink = millis();
      }
      if (millis() - lastAlarmTx > ALARM_INTERVAL) {
        sendLoRaAlert("ALARM_ACTIVE");
        lastAlarmTx = millis();
      }
      break;

    case STATE_WAITING_2FA:
      digitalWrite(GREEN_LED_PIN, LOW); 
      if (millis() - lastLedBlink > 250) {
        ledState = !ledState;
        digitalWrite(RED_LED_PIN, ledState);
        lastLedBlink = millis();
      }
      break;

    case STATE_DISARMED:
      if (potFollowMode) {
        int currentPotVal = readPotAvg(); 
        if (abs(currentPotVal - lastStablePotVal) > POT_DEADBAND) {
           lastStablePotVal = currentPotVal; 
           targetServoAngle = map(currentPotVal, 0, 4095, 0, 180);
        }
      }
      digitalWrite(GREEN_LED_PIN, LOW);
      digitalWrite(RED_LED_PIN, LOW);
      break;
      
    default:
      break;
  }
}

void handleBluetoothCommand(String cmd) {
  cmd.trim(); 
  if (cmd.length() == 0) return;
  Serial.print("Processing CMD: ["); Serial.print(cmd); Serial.println("]");

  if (cmd == "DISARM") {
    bleReqDisarm = true;
  }
  else if (cmd == "ARM") {
    bleReqArm = true;
  }
  else if (cmd == "OPEN") {
    bleReqOpen = true;
  }
  else if (cmd == "CLOSE") { 
    bleReqClose = true;
  }
  else if (cmd == "DIAL_ON") { 
    bleReqDialOn = true;
  }
  else {
    if (currentState == STATE_WAITING_2FA) {
      String response = "";
      if (cmd.startsWith("RESPONSE:")) response = cmd.substring(9);
      else response = cmd;

      if (response == currentChallenge) bleAuthStatus = 1; 
      else bleAuthStatus = 2; 
    }
  }
}

// ==========================================
// 6. SETUP & LOOP
// ==========================================
void setup() {
  delay(5000); 

  Serial.begin(115200);
  Serial.println("System Initializing...");

  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(POTENTIOMETER_PIN, INPUT);
  
  lockServo.setPeriodHertz(50); 
  lockServo.attach(SERVO_PIN, 500, 2400); 
  lockServo.write(0); 
  currentServoAngle = 0;
  targetServoAngle = 0;

  BLEDevice::init("Secure_Asset"); 
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("BLE Initialized.");

  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, LOW);
  delay(20);
  digitalWrite(RST_PIN, HIGH);
  delay(20);

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  LoRa.setPins(SS_PIN, RST_PIN, DI0_PIN);
  
  if (!LoRa.begin(915E6)) {
    Serial.println("ERROR: LoRa Init Failed!");
    loraInitialized = false; 
  } else {
    Serial.println("LoRa Initialized Successfully.");
    LoRa.setTxPower(10); 
    loraInitialized = true;
  }

  // START IN DISARMED MODE WITH POT CONTROL ENABLED
  currentState = STATE_DISARMED;
  potFollowMode = true;
  initialDoorPos = readPotAvg();
  lastStablePotVal = initialDoorPos; 
}

void loop() {
  updateServoPosition();

  if (!deviceConnected && oldDeviceConnected) {
      delay(500); 
      pServer->startAdvertising(); 
      Serial.println("Restarting BLE Advertising...");
      oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) { oldDeviceConnected = deviceConnected; }

  if (rxValue.length() > 0) {
    String cmd = String(rxValue.c_str());
    rxValue = ""; 
    handleBluetoothCommand(cmd);
  }

  TickFct_Security();
  
  delay(10); 
  yield(); 
}
