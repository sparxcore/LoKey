// LoKey Code

#define HELTEC_POWER_BUTTON
#include <heltec_unofficial.h>
#include "mbedtls/base64.h"

// Radio Config - ensure this is the same on LoLock
#define FREQUENCY           868.3       // for Europe
#define BANDWIDTH           125.0
#define SPREADING_FACTOR    7
#define TRANSMIT_POWER      14

// Pin Definitions
#define LED_1 4                // LED 1 connected to GPIO 4
#define LED_2 5                // LED 2 connected to GPIO 5
#define LED_3 20               // LED 3 connected to GPIO 20
#define LED_4 21               // LED 4 connected to GPIO 21
#define BUTTON_1 16            // Button 1 connected to GPIO 16
#define BUTTON_2 17            // Button 2 connected to GPIO 17
#define BUTTON_3 18            // Button 3 connected to GPIO 18
#define BUTTON_4 19            // Button 4 connected to GPIO 19

// Others
#define FLASH_INTERVAL 200     // Interval for LED flashing in milliseconds
#define RX_BUFFER_SIZE 256     // Buffer size for receiving data

// Flags for managing LED behaviors
bool flashLEDsFlag = false;           
bool pairingCompleted = false;          
unsigned long lastFlashTime = 0;
bool ledsOn = false;

// Enum for states
enum DeviceState {
  PAIRING,
  IDLE,
  WAITING_FOR_RESET,
  READY_FOR_COMMANDS,
  WAITING_FOR_SERVO_LOCK,
  ARMED_MODE,
  WAITING_FOR_SERVO_UNLOCK,
  UNLOADED
};

DeviceState currentState = PAIRING;
int lockPosition = 4;
int unlockPosition = 1;
bool buttonHeld = false;
unsigned long lastButtonPressTime = 0;
unsigned long buttonHoldStartTime = 0;
unsigned long debounceDelay = 200;
bool buttonPressed[4] = {false, false, false, false};
volatile bool rxFlag = false;
uint8_t rxdata[RX_BUFFER_SIZE]; 

const int LED_PINS[4] = {LED_1, LED_2, LED_3, LED_4};
const int BUTTON_PINS[4] = {BUTTON_1, BUTTON_2, BUTTON_3, BUTTON_4};

// Variables for pairing and encryption
#define KEY_LENGTH 16
uint8_t encryptionKey[KEY_LENGTH];
const int pairingSequence[] = {0, 1, 2, 3, 2, 1};
const int pairingSequenceLength = 6;
int pairingSequenceIndex = 0;
unsigned long pairingSequenceInterval = 200;
unsigned long lastPairingSequenceTime = 0;

// Variables for heartbeat tracking
bool heartbeatReceived = false;
unsigned long lastHeartbeatTime = 0;
const unsigned long HEARTBEAT_TIMEOUT = 6000;
unsigned long lastPairingRequestTime = 0; 
const unsigned long PAIRING_REQUEST_INTERVAL = 5000;

// Function prototypes
void sendPairingRequest();
void sendResetCommand();
void sendServoCommand(int command);
void sendHeartbeatAck();
void handlePairingPacket(uint8_t* packet, size_t length);
void handleReceivedPacket(uint8_t* packet, size_t length);
void lightAllLEDs();
void resetAllLEDs();
void checkButtonPresses();
void handleButton1Press();
void resetButtonStates();
void flashLoadingLEDs();
void blinkAllLEDsOnce();
void encryptMessage(const char* message, char* output, size_t outputSize);
void decryptMessage(const char* message, size_t messageLength, char* output, size_t outputSize);
void flashAllLEDs(int times, int delayTime);
void printEncryptionKey();
void setupEncryptionKey();
void rx();

void setup() {
  Serial.begin(115200);
  heltec_setup();
  both.println("Radio init");
  RADIOLIB_OR_HALT(radio.begin());

  RADIOLIB_OR_HALT(radio.setFrequency(FREQUENCY));
  RADIOLIB_OR_HALT(radio.setBandwidth(BANDWIDTH));
  RADIOLIB_OR_HALT(radio.setSpreadingFactor(SPREADING_FACTOR));
  RADIOLIB_OR_HALT(radio.setOutputPower(TRANSMIT_POWER));

  for (int i = 0; i < 4; i++) {
    pinMode(LED_PINS[i], OUTPUT);
    pinMode(BUTTON_PINS[i], INPUT_PULLUP);
  }

  resetAllLEDs();  // Ensure all LEDs are off at startup
  // Initialize pairing state
  currentState = PAIRING;

  // Generate random encryption key
  randomSeed(analogRead(0));  // Initialize random seed
  for (int i = 0; i < KEY_LENGTH; i++) {
    encryptionKey[i] = random(0, 256);
  }
  printEncryptionKey();

  sendPairingRequest();

  lastFlashTime = millis();
  lastPairingSequenceTime = millis();

  Serial.println("Starting in PAIRING mode...");
}


// LED Handling Functions
void handlePairingLEDs() {
  if (millis() - lastPairingSequenceTime >= pairingSequenceInterval) {
    lastPairingSequenceTime = millis();
    resetAllLEDs();
    digitalWrite(LED_PINS[pairingSequence[pairingSequenceIndex]], HIGH);
    pairingSequenceIndex = (pairingSequenceIndex + 1) % pairingSequenceLength;
  }
}

void handleIdleState() {
  if (flashLEDsFlag) {  // Triggered when a heartbeat is received
    if (millis() - lastFlashTime >= FLASH_INTERVAL) {
      lastFlashTime = millis();
      blinkAllLEDsOnce();
      flashLEDsFlag = false;  // Reset the flag after flashing
      //Serial.println("Heartbeat processed.");
    }
  }
}

void handleResetModeLEDs() {
  // Regular LED flashing to indicate arming process
  if (millis() - lastFlashTime >= FLASH_INTERVAL) {
    lastFlashTime = millis();
    ledsOn = !ledsOn;
    for (int i = 0; i < 10; i++) {
      digitalWrite(LED_PINS[i], ledsOn ? HIGH : LOW);
    }
  }
}

void loop() {
  heltec_loop();
  memset(rxdata, 0, RX_BUFFER_SIZE);

  // Receive data
  int state = radio.receive(rxdata, RX_BUFFER_SIZE);

  if (state == RADIOLIB_ERR_NONE) {
    size_t len = radio.getPacketLength();
    if (len > RX_BUFFER_SIZE - 1) {
      len = RX_BUFFER_SIZE - 1;
    }
    rxdata[len] = '\0'; // Null-terminate

    //DEBUG: Raw packets
    //Serial.print("Received raw packet length: ");
    //Serial.println(len);
    //Serial.print("Received raw packet: ");
    //for (size_t i = 0; i < len; i++) {
    //  Serial.printf("%02x ", rxdata[i]);
    //}
    //Serial.println();

    if (currentState == PAIRING) {
      handlePairingPacket(rxdata, len);
    } else {
      handleReceivedPacket(rxdata, len);
    }
  } else if (state != RADIOLIB_ERR_RX_TIMEOUT) {
    // Handle other receive errors
    Serial.print("Receive failed, code ");
    Serial.println(state);
  }

  checkButtonPresses();

  // Handle LEDs based on the current state
  switch (currentState) {
    case PAIRING:
      handlePairingLEDs();
      if (millis() - lastPairingRequestTime >= PAIRING_REQUEST_INTERVAL) {
        sendPairingRequest();
        lastPairingRequestTime = millis();  // Update the time of the last request
      }
      break;

    case IDLE:
      handleIdleState();
      break;

    case WAITING_FOR_RESET:
      handleResetModeLEDs();
      break;

    default:
      break;
  }
}


void handlePairingPacket(uint8_t* packet, size_t length) {
  packet[length] = '\0'; // Null-terminate
  Serial.print("Received Pairing Packet: ");
  Serial.println((char*)packet);

  if (strncmp((char*)packet, "pairing_ack", length) == 0) {
    Serial.println("Pairing successful, transitioning to IDLE...");
    pairingCompleted = true;
    currentState = IDLE; // Transition to IDLE

    // MUST clear pairing-related buffer after successful pairing
    memset(rxdata, 0, RX_BUFFER_SIZE);

    // Encryption key should be already set, print it for debugging
    //printEncryptionKey();
  }
}

void handleReceivedPacket(uint8_t* packet, size_t length) {
  packet[length] = '\0';  // Null-terminate
  Serial.print("Received Packet: ");
  Serial.println((char*)packet);

  if (pairingCompleted) {
    // Decrypt and process the message
    char decryptedMessage[128];
    decryptMessage((char*)packet, length, decryptedMessage, sizeof(decryptedMessage));

    Serial.print("Decrypted Message: ");
    Serial.println(decryptedMessage);

    if (strcmp(decryptedMessage, "heartbeat") == 0) {
      // Handle heartbeat packet
      Serial.println("LoLock Sent Heartbeat");
      lastHeartbeatTime = millis();
      heartbeatReceived = true;
      flashLEDsFlag = true;
      sendHeartbeatAck();
    } else if (strcmp(decryptedMessage, "reset_done") == 0 && currentState == WAITING_FOR_RESET) {
      // Handle reset_done packet
      lockPosition = 4;
      unlockPosition = 1;
      flashAllLEDs(3, 200);
      Serial.println("Ready for sequential commands");
      currentState = READY_FOR_COMMANDS;
      //lastFlashTime = millis();  // Reset the flash timer
    } else if (strncmp(decryptedMessage, "servo_done", 10) == 0) {
      // Handle servo done packet using char positions - jank
      char operationType = decryptedMessage[10]; // 'L' -lock or 'U' -unlock
      int cmd = atoi(&decryptedMessage[11]); // Gate position number (1-4)

      if (cmd >= 1 && cmd <= 4) {
        if (operationType == 'L' && currentState == WAITING_FOR_SERVO_LOCK) {
          // Handle loading acknowledgment
          
          if (cmd == lockPosition) {
            digitalWrite(LED_PINS[cmd - 1], HIGH);  // LED on
            Serial.print("Locked position ");
            Serial.println(cmd);
            lockPosition--;
            if (lockPosition < 1) {
              currentState = ARMED_MODE;
              Serial.println("System armed. Ready for unlock commands.");
            } else {
              currentState = READY_FOR_COMMANDS;
              //lastFlashTime = millis();  // Reset the flash timer
            }
          } else {
            Serial.print("Unexpected servo_done command: ");
            Serial.println(cmd);
            currentState = READY_FOR_COMMANDS;
          }
        } else if (operationType == 'U' && currentState == WAITING_FOR_SERVO_UNLOCK) {
          // Handle unloading acknowledgment
          if (cmd == unlockPosition) {
            digitalWrite(LED_PINS[cmd - 1], LOW);  // Turn off the corresponding LED
            Serial.print("Unlocked position ");
            Serial.println(cmd);
            unlockPosition++;
            if (unlockPosition > 4) {
              currentState = UNLOADED;
              Serial.println("System unloaded. Returning to IDLE state.");
              resetAllLEDs();
              currentState = IDLE; // Transition back to IDLE
            } else {
              currentState = ARMED_MODE;
            }
          } else {
            Serial.print("Unexpected servo_done command: ");
            Serial.println(cmd);
            currentState = ARMED_MODE;
          }
        } else {
          // Received servo_done with mismatched operation type or state could probably use this for something
          Serial.println("Received servo_done with mismatched operation or state.");
          currentState = (currentState == WAITING_FOR_SERVO_LOCK) ? READY_FOR_COMMANDS : ARMED_MODE;
        }
      } else {
        // Invalid servo_done command received, could probably use this for something
        Serial.print("Invalid servo_done command: ");
        Serial.println(cmd);
        currentState = (currentState == WAITING_FOR_SERVO_LOCK) ? READY_FOR_COMMANDS : ARMED_MODE;
      }
    } else {
      // Handle invalid or unknown packet...
      Serial.println("Invalid command received.");
    }
  } else {
    Serial.println("Received data before pairing was complete.");
  }
}

void sendPairingRequest() {
  char pairingMsg[128];
  snprintf(pairingMsg, sizeof(pairingMsg), "pairing_request:");
  for (int i = 0; i < KEY_LENGTH; i++) {
    char hexByte[3];
    snprintf(hexByte, sizeof(hexByte), "%02x", encryptionKey[i]);
    strncat(pairingMsg, hexByte, sizeof(pairingMsg) - strlen(pairingMsg) - 1);
  }

  Serial.println("TX Pairing Request: " + String(pairingMsg));

  int radioState = radio.transmit((uint8_t*)pairingMsg, strlen(pairingMsg));
  if (radioState != RADIOLIB_ERR_NONE) {
    Serial.print("TX Pairing Request Error: ");
    Serial.println(radioState);
  } else {
    Serial.println("Pairing request sent successfully.");
  }
}

void sendResetCommand() {
    const char* resetCmd = "reset";
    char encryptedMessage[128];
    encryptMessage(resetCmd, encryptedMessage, sizeof(encryptedMessage));

    Serial.println("TX Reset Command: " + String(encryptedMessage));

    int radioState = radio.transmit((uint8_t*)encryptedMessage, strlen(encryptedMessage));
    if (radioState != RADIOLIB_ERR_NONE) {
        Serial.print("TX Reset Command Error: ");
        Serial.println(radioState);
    } else {
        Serial.println("Reset Command Transmitted Successfully.");
    }

}

void sendServoCommand(int command) {
  char servoCmd[16];
  snprintf(servoCmd, sizeof(servoCmd), "servo%d", command);
  char encryptedMessage[128];
  encryptMessage(servoCmd, encryptedMessage, sizeof(encryptedMessage));
  Serial.println("TX Servo Command: " + String(encryptedMessage));

  int radioState = radio.transmit((uint8_t*)encryptedMessage, strlen(encryptedMessage));
  if (radioState != RADIOLIB_ERR_NONE) {
    Serial.print("TX Servo Command Error: ");
    Serial.println(radioState);
  }
}

void sendHeartbeatAck() {
  const char* message = "heartbeat_ack";
  char encryptedMessage[128];
  encryptMessage(message, encryptedMessage, sizeof(encryptedMessage));
  Serial.println("TX Heartbeat Ack: " + String(encryptedMessage));

  int radioState = radio.transmit((uint8_t*)encryptedMessage, strlen(encryptedMessage));
  if (radioState != RADIOLIB_ERR_NONE) {
    Serial.print("TX Heartbeat Ack Error: ");
    Serial.println(radioState);
  }
}

void encryptMessage(const char* message, char* output, size_t outputSize) {
  // XOR Encrypt the packet
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

  // Base64 encode the encrypted data to ensure good lora packet
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
  //Serial.print("Encrypted Message (Base64): ");
  //Serial.println(output);
}

void decryptMessage(const char* message, size_t messageLength, char* output, size_t outputSize) {
  size_t decodedLength = (messageLength * 3) / 4 + 1;
  uint8_t encryptedBytes[decodedLength];
  size_t actualDecodedLength = 0;

  int result = mbedtls_base64_decode(encryptedBytes, decodedLength, &actualDecodedLength, (const unsigned char*)message, messageLength);

  if (result != 0) {
    Serial.println("Base64 decoding failed");
    return;
  }

  // Debug: Print encrypted bytes after Base64 decoding
  //Serial.print("Encrypted Bytes After Base64 Decoding: ");
  //for (size_t i = 0; i < actualDecodedLength; i++) {
  //  Serial.printf("%02x ", encryptedBytes[i]);
  //}
  //Serial.println();

  if (actualDecodedLength + 1 > outputSize) { // +1 for null terminator
    Serial.println("Output buffer too small for decoded message");
    return;
  }

  for (size_t i = 0; i < actualDecodedLength; i++) {
    output[i] = encryptedBytes[i] ^ encryptionKey[i % KEY_LENGTH];
  }

  output[actualDecodedLength] = '\0'; // Null-terminate

}

void printEncryptionKey() {
  Serial.print("Encryption Key: ");
  for (int i = 0; i < KEY_LENGTH; i++) {
    Serial.printf("%02x", encryptionKey[i]);
  }
  Serial.println();
}

void lightAllLEDs() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(LED_PINS[i], HIGH);
  }
}

void resetAllLEDs() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(LED_PINS[i], LOW);
  }
}

void flashAllLEDs(int times, int delayTime) {
  //for (int i = 0; i < times; i++) {
    lightAllLEDs();
    delay(delayTime);
    resetAllLEDs();
    delay(delayTime);
  //}
}

void blinkAllLEDsOnce() {
  lightAllLEDs();
  delay(500);
  resetAllLEDs();
}

void checkButtonPresses() {
  if ((millis() - lastButtonPressTime) > debounceDelay) {
    bool buttonStates[4];
    for (int i = 0; i < 4; i++) {
      buttonStates[i] = digitalRead(BUTTON_PINS[i]) == LOW;
    }

    if (buttonStates[0]) {
      handleButton1Press();
    }

    if (currentState == READY_FOR_COMMANDS) {
      int buttonIndex = lockPosition - 1;  // From 3 down to 0
      if (buttonIndex >= 0 && buttonIndex < 4) { // Safety check
        if (buttonStates[buttonIndex] && !buttonPressed[buttonIndex]) {
          buttonPressed[buttonIndex] = true;
          lastButtonPressTime = millis();
          Serial.print("Button ");
          Serial.print(lockPosition);
          Serial.println(" pressed for locking.");
          sendServoCommand(lockPosition);
          currentState = WAITING_FOR_SERVO_LOCK;
        }
      }
    } else if (currentState == ARMED_MODE) {
      int buttonIndex = unlockPosition - 1;  // From 0 up to 3
      if (buttonIndex >= 0 && buttonIndex < 4) { // Safety check
        if (buttonStates[buttonIndex] && !buttonPressed[buttonIndex]) {
          buttonPressed[buttonIndex] = true;
          lastButtonPressTime = millis();
          Serial.print("Button ");
          Serial.print(unlockPosition);
          Serial.println(" pressed for unlocking.");
          sendServoCommand(unlockPosition);
          currentState = WAITING_FOR_SERVO_UNLOCK;
        }
      }
    }

    resetButtonStates();
  }
}

void resetButtonStates() {
  for (int i = 0; i < 4; i++) {
    if (digitalRead(BUTTON_PINS[i]) == HIGH) {
      buttonPressed[i] = false;
    }
  }
}

void handleButton1Press() {
  if (digitalRead(BUTTON_PINS[0]) == LOW) { // Button is pressed
    if (currentState == IDLE || currentState == READY_FOR_COMMANDS || currentState == UNLOADED) {
      if (!buttonHeld) {
        Serial.println("Button 1 pressed, starting hold timer...");
        buttonHoldStartTime = millis();
        buttonHeld = true;
      } else if (millis() - buttonHoldStartTime > 3000) { // 3 seconds hold
        Serial.println("Button 1 held for 3 seconds, triggering reset.");
        buttonPressed[0] = true;
        lastButtonPressTime = millis();
        buttonHeld = false;
        sendResetCommand();  // Send reset command
        currentState = WAITING_FOR_RESET;
        Serial.println("Entering RESET_MODE...");
      }
    }
  } else { // Button released
    if (buttonHeld) {
      Serial.println("Button 1 released before 3 seconds.");
    }
    buttonHeld = false;
  }
}

void rx() {
  rxFlag = true;
}