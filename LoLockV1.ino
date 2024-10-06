// LoLock Code

#include <Arduino.h>
#include <RadioLib.h> //use 6.6.0
#include <ESP32Servo.h>  // Include the ESP32Servo library
#include "mbedtls/base64.h"

// Pin Definitions
#define LORA_SS 8          // GPIO8 - SPI Chip Select
#define LORA_DIO1 14       // GPIO14 - DIO1
#define LORA_RST 12        // GPIO12 - LoRa Reset
#define LORA_BUSY 13       // GPIO13 - LoRa Busy

#define SERVO_PIN 18   
// Define the LoRa radio
Module myModule(LORA_SS, LORA_DIO1, LORA_RST, LORA_BUSY);
SX1262 radio(&myModule); 
// Radio Config
#define FREQUENCY           868.3       // Frequency for Europe
#define BANDWIDTH           125.0
#define SPREADING_FACTOR    7
#define TRANSMIT_POWER      14

// Servo positions
const int NUM_POSITIONS = 5;
int servoPositions[NUM_POSITIONS] = { 68, 90, 118, 135, 162 }; // Adjust angles as needed - DJI FPV bracket

int currentPositionIndex = 0; // Start at position 0 (fully retracted)
bool isArmed = false;         // Track whether the system is armed

Servo myservo;

// Variables for radio communication
#define RX_BUFFER_SIZE 256
uint8_t rxdata[RX_BUFFER_SIZE];
volatile bool rxFlag = false;
unsigned long last_tx = 0;
unsigned long tx_time;
unsigned long minimum_pause = 0;
unsigned long lastHeartbeat = 0;

// Variables for pairing and encryption
#define KEY_LENGTH 16
uint8_t encryptionKey[KEY_LENGTH];
enum DeviceState {
  PAIRING,
  IDLE
};
DeviceState currentState = PAIRING;

// Function prototypes
void moveServoToPosition(int positionIndex);
void handleReceivedPacket(uint8_t* packet, size_t length);
void sendHeartbeat();
void sendResetDone();
void sendServoDone(int buttonNumber, char operationType);
void sendPairingAck();
void rx();
void encryptMessage(const char* message, char* output, size_t outputSize);
void decryptMessage(const char* message, size_t messageLength, char* output, size_t outputSize);
void printEncryptionKey();

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Initializing Servo...");

  // Attach servo
  myservo.setPeriodHertz(50);
  myservo.attach(SERVO_PIN, 500, 2400);

  // Move servo to initial position
  moveServoToPosition(currentPositionIndex);
  Serial.println("Servo initialized");
  Serial.println("Radio init");

  // Initialize the radio
  int radioState = radio.begin();
  if (radioState != RADIOLIB_ERR_NONE) {
    Serial.print("Radio init failed, code ");
    Serial.println(radioState);
    while (true);
  }

  radio.setDio1Action(rx);
  radioState = radio.setFrequency(FREQUENCY);
  if (radioState != RADIOLIB_ERR_NONE) {
    Serial.print("setFrequency failed, code ");
    Serial.println(radioState);
    while (true);
  }

  radioState = radio.setBandwidth(BANDWIDTH);
  if (radioState != RADIOLIB_ERR_NONE) {
    Serial.print("setBandwidth failed, code ");
    Serial.println(radioState);
    while (true);
  }

  radioState = radio.setSpreadingFactor(SPREADING_FACTOR);
  if (radioState != RADIOLIB_ERR_NONE) {
    Serial.print("setSpreadingFactor failed, code ");
    Serial.println(radioState);
    while (true);
  }

  radioState = radio.setOutputPower(TRANSMIT_POWER);
  if (radioState != RADIOLIB_ERR_NONE) {
    Serial.print("setOutputPower failed, code ");
    Serial.println(radioState);
    while (true);
  }

  // Initialize pairing state
  currentState = PAIRING;

  // Start receiving data
  radioState = radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);
  if (radioState != RADIOLIB_ERR_NONE) {
    Serial.print("startReceive failed, code ");
    Serial.println(radioState);
    while (true);
  }

  Serial.println("Starting in PAIRING mode...");
}

void loop() {
  // Send heartbeat every 5 seconds
  if (currentState != PAIRING && millis() - lastHeartbeat > 5000) {
    lastHeartbeat = millis();
    sendHeartbeat();
  }

  // Handle received packets
  if (rxFlag) {
    rxFlag = false;

    // Clear rxdata
    memset(rxdata, 0, RX_BUFFER_SIZE);

    // Read the data
    int16_t state = radio.readData(rxdata, RX_BUFFER_SIZE);

    if (state == RADIOLIB_ERR_NONE) {
      size_t len = radio.getPacketLength();
      if (len > RX_BUFFER_SIZE - 1) {
        len = RX_BUFFER_SIZE - 1;
      }
      rxdata[len] = '\0'; // Null-terminate
      handleReceivedPacket(rxdata, len);
    } else {
      Serial.print("RX Error: ");
      Serial.println(state);
    }

    // Restart receiving
    radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);
  }
}

void handleReceivedPacket(uint8_t* packet, size_t length) {
  packet[length] = '\0'; // Null-terminate
  Serial.print("RX: ");
  Serial.println((char*)packet);
  Serial.print("Current State: ");
  Serial.println(currentState);

  if (currentState == PAIRING && strncmp((char*)packet, "pairing_request:", 16) == 0) {
    Serial.println("Pairing request received.");
    char keyString[KEY_LENGTH * 2 + 1]; // +1 for null terminator
    strncpy(keyString, (char*)packet + 16, KEY_LENGTH * 2);
    keyString[KEY_LENGTH * 2] = '\0';

    if (strlen(keyString) == KEY_LENGTH * 2) {
      for (int i = 0; i < KEY_LENGTH; i++) {
        char byteString[3];
        strncpy(byteString, keyString + i * 2, 2);
        byteString[2] = '\0';
        encryptionKey[i] = (uint8_t)strtol(byteString, NULL, 16);
        // Debug: Print each key byte
       // Serial.print("Key Byte ");
       // Serial.print(i);
       // Serial.print(": ");
       // Serial.println(encryptionKey[i], HEX);
      }
      Serial.println("Encryption key received.");
      printEncryptionKey(); // Print the received encryption key for debugging
      sendPairingAck();
      currentState = IDLE;
    } else {
      Serial.println("Invalid key length.");
    }
  } else if (currentState != PAIRING) {
    // Decrypt the message
    char decryptedMessage[128];
    decryptMessage((char*)packet, length, decryptedMessage, sizeof(decryptedMessage));

    if (strcmp(decryptedMessage, "heartbeat_ack") == 0) {
      Serial.println("LoKey Acknowledged Heartbeat");
    } else if (strcmp(decryptedMessage, "reset") == 0) {
      Serial.println("Reset command received. Moving to initial position.");
      currentPositionIndex = 0;
      isArmed = false;
      moveServoToPosition(currentPositionIndex);
      sendResetDone();
    } else if (strncmp(decryptedMessage, "servo", 5) == 0) {
      int buttonNumber = atoi(&decryptedMessage[5]); // Extract button number (1-4)
      if (buttonNumber >= 1 && buttonNumber <= 4) {
        int targetPositionIndex = -1;
        char operationType;

        if (!isArmed) {
          // Locking/Arming Mode
          operationType = 'L';
          switch (buttonNumber) {
            case 4:
              targetPositionIndex = 1;
              break;
            case 3:
              targetPositionIndex = 2;
              break;
            case 2:
              targetPositionIndex = 3;
              break;
            case 1:
              targetPositionIndex = 4;
              isArmed = true;
              Serial.println("System is now armed.");
              break;
            default:
              Serial.println("Invalid button number.");
              break;
          }
        } else {
          // Unlocking Mode
          operationType = 'U';
          switch (buttonNumber) {
            case 1:
              targetPositionIndex = 3;
              break;
            case 2:
              targetPositionIndex = 2;
              break;
            case 3:
              targetPositionIndex = 1;
              break;
            case 4:
              targetPositionIndex = 0;
              isArmed = false;
              Serial.println("System is now disarmed.");
              break;
            default:
              Serial.println("Invalid button number.");
              break;
          }
        }

        if (targetPositionIndex != -1) {
          currentPositionIndex = targetPositionIndex;
          Serial.print("Moving to position ");
          Serial.print(currentPositionIndex);
          Serial.print(", angle ");
          Serial.println(servoPositions[currentPositionIndex]);
          moveServoToPosition(currentPositionIndex);
          sendServoDone(buttonNumber, operationType);
        } else {
          Serial.println("Invalid target position.");
        }
      } else {
        Serial.println("Invalid button number received.");
      }
    } else {
      Serial.println("Invalid command received.");
    }
  }
}

void moveServoToPosition(int positionIndex) {
  if (positionIndex >= 0 && positionIndex < NUM_POSITIONS) {
    int angle = servoPositions[positionIndex];
    Serial.print("Moving servo to position ");
    Serial.print(positionIndex);
    Serial.print(", angle ");
    Serial.println(angle);
    myservo.write(angle);
    // Wait for servo to reach position
    delay(200);
  } else {
    Serial.println("Invalid position index.");
  }
}

void sendHeartbeat() {
  if (millis() > last_tx + minimum_pause) {
    const char* message = "heartbeat";
    char encryptedMessage[128];
    encryptMessage(message, encryptedMessage, sizeof(encryptedMessage));
    Serial.println("TX Heartbeat: " + String(encryptedMessage));
    radio.clearDio1Action();
    tx_time = millis();
    int radioState = radio.transmit((uint8_t*)encryptedMessage, strlen(encryptedMessage));
    tx_time = millis() - tx_time;
    if (radioState == RADIOLIB_ERR_NONE) {
      Serial.print("TX OK (");
      Serial.print(tx_time);
      Serial.println(" ms)");
    } else {
      Serial.print("TX Error: ");
      Serial.println(radioState);
    }
    minimum_pause = 0;
    last_tx = millis();
    radio.setDio1Action(rx);
    radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);
  }
}

void sendResetDone() {
  const char* message = "reset_done";
  char encryptedMessage[128];
  encryptMessage(message, encryptedMessage, sizeof(encryptedMessage));
  Serial.println("TX Reset Done: " + String(encryptedMessage));
  radio.clearDio1Action();
  tx_time = millis();
  int radioState = radio.transmit((uint8_t*)encryptedMessage, strlen(encryptedMessage));
  tx_time = millis() - tx_time;
  if (radioState == RADIOLIB_ERR_NONE) {
    Serial.print("TX OK (");
    Serial.print(tx_time);
    Serial.println(" ms)");
  } else {
    Serial.print("TX Error: ");
    Serial.println(radioState);
  }
  minimum_pause = tx_time * 100;
  last_tx = millis();
  radio.setDio1Action(rx);
  radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);
}

void sendServoDone(int buttonNumber, char operationType) {
  char servoDoneMsg[32];
  snprintf(servoDoneMsg, sizeof(servoDoneMsg), "servo_done%c%d", operationType, buttonNumber);
  char encryptedMessage[128];
  encryptMessage(servoDoneMsg, encryptedMessage, sizeof(encryptedMessage));
  Serial.println("TX Servo Done: " + String(encryptedMessage));
  radio.clearDio1Action();
  tx_time = millis();
  int radioState = radio.transmit((uint8_t*)encryptedMessage, strlen(encryptedMessage));
  tx_time = millis() - tx_time;
  if (radioState == RADIOLIB_ERR_NONE) {
    Serial.print("TX OK (");
    Serial.print(tx_time);
    Serial.println(" ms)");
  } else {
    Serial.print("TX Error: ");
    Serial.println(radioState);
  }
  minimum_pause = tx_time * 100;
  last_tx = millis();
  radio.setDio1Action(rx);
  radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);
}

void sendPairingAck() {
  Serial.println("TX Pairing Ack");
  radio.clearDio1Action();
  int radioState = radio.transmit((uint8_t*)"pairing_ack", strlen("pairing_ack"));
  if (radioState != RADIOLIB_ERR_NONE) {
    Serial.print("TX Error: ");
    Serial.println(radioState);
  } else {
    Serial.println("Pairing Ack sent successfully.");
    // Move servo across full range after sending pairing_ack
    delay(200); // Small delay to ensure message is transmitted
    Serial.println("Servo Move after Pairing");
    moveServoToPosition(4); // Move to position 4
    delay(400); // Wait for servo to reach position
    Serial.println("Moving servo back to position 0.");
    moveServoToPosition(0); // Move back to position 0
  }
  radio.setDio1Action(rx);
  radio.startReceive(RADIOLIB_SX126X_RX_TIMEOUT_INF);
}

void rx() {
  rxFlag = true;
}

void encryptMessage(const char* message, char* output, size_t outputSize) {
  size_t messageLength = strlen(message);
  uint8_t encryptedBytes[messageLength];
  for (size_t i = 0; i < messageLength; i++) {
    encryptedBytes[i] = message[i] ^ encryptionKey[i % KEY_LENGTH];
  }

  // Debug: Print encrypted bytes
  //Serial.print("Encrypted Bytes: ");
  //for (size_t i = 0; i < messageLength; i++) {
  //  Serial.printf("%02x ", encryptedBytes[i]);
  //}
  //Serial.println();

  // Base64 encode the encrypted data
  size_t encodedLength = ((messageLength + 2) / 3) * 4;
  if (encodedLength + 1 > outputSize) { // +1 for null terminator
    Serial.println("Output buffer too small for encoded message");
    return;
  }

  size_t actualEncodedLength = 0;
  int result = mbedtls_base64_encode((unsigned char*)output, outputSize, &actualEncodedLength, encryptedBytes, messageLength);

  if (result != 0) {
    Serial.println("Base64 encoding failed");
    return;
  }

  output[actualEncodedLength] = '\0'; // Null-terminate

  // Debug: Print Base64 encoded message
  Serial.print("Encrypted Message (Base64): ");
  Serial.println(output);
}

void decryptMessage(const char* message, size_t messageLength, char* output, size_t outputSize) {

  // Add Base64 length check
  if (messageLength % 4 != 0) {
    Serial.println("Invalid Base64 message length");
    return;
  }

  size_t decodedLength = (messageLength * 3) / 4 + 1;
  uint8_t encryptedBytes[decodedLength];
  size_t actualDecodedLength = 0;

  int result = mbedtls_base64_decode(encryptedBytes, decodedLength, &actualDecodedLength, (const unsigned char*)message, messageLength);

  if (result != 0) {
    Serial.println("Base64 decoding failed");
    return;
  }

  // Debug: Print encrypted bytes after Base64 decoding
  Serial.print("Encrypted Bytes After Base64 Decoding: ");
  for (size_t i = 0; i < actualDecodedLength; i++) {
    Serial.printf("%02x ", encryptedBytes[i]);
  }
  Serial.println();

  if (actualDecodedLength + 1 > outputSize) { // +1 for null terminator
    Serial.println("Output buffer too small for decoded message");
    return;
  }

  // Use actualDecodedLength for indexing
  for (size_t i = 0; i < actualDecodedLength; i++) {
    output[i] = encryptedBytes[i] ^ encryptionKey[i % KEY_LENGTH];
  }

  output[actualDecodedLength] = '\0'; // Null-terminate

  // Debug: Print decrypted message
  Serial.print("Decrypted Message: ");
  Serial.println(output);
}

void printEncryptionKey() {
  Serial.print("Encryption Key: ");
  for (int i = 0; i < KEY_LENGTH; i++) {
    Serial.printf("%02x", encryptionKey[i]);
  }
  Serial.println();
}
