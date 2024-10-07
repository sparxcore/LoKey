# LoKey and LoLock System

## Overview

This project implements a **LoKey** and **LoLock** system using ESP32 / LoRa hardware. The system facilitates wireless communication between **LoKey** (a handheld control unit) and **LoLock** (a locking mechanism). **LoKey** sends encrypted commands to **LoLock** over LoRa, instructing it to perform actions such as locking, unlocking, or resetting a servo mechanism.

The system is designed with some-what secure communication using XOR-based encryption with a Random key, and Base64 encoding. 
It requires **RadioLib v6.6.0**.

## Features

- **Wireless Communication:** LoKey and LoLock communicate wirelessly using LoRa.
- **Encrypted Communication:** Commands are XOR encrypted with a 16-byte key and Base64 encoded.
- **Servo Control:** LoLock controls a servo mechanism to lock and unlock based on commands received from LoKey.
- **Heartbeat Mechanism:** LoLock sends periodic heartbeat signals to LoKey to maintain communication integrity.
- **Reset Functionality:** LoKey can reset LoLock to return the servo to its initial position.
- **Packet Acknowledgment and feedback:** Both devices acknowledge the successful reception and execution of commands LoKey uses LEDs to show status.

## Hardware Requirements

- **Heltec Wireless Stick (V3)**
- **ESP32Servo Library** (for servo control on LoLock)
- **LoRa SX1262 Module** (used by both LoKey and LoLock)
- **9g Servo** (controlled by LoLock for locking and unlocking actions)
- **4 LED backlit Switches** (to use as input control for LoKey
- **LoKey Controller Model** https://www.printables.com/model/1031026-lokey-controller
- **LoLock Drone Mount** https://www.printables.com/model/1031020-lolock-dji-fpv

## Software Requirements

- **Arduino IDE** (or PlatformIO)
- **RadioLib v6.6.0** (required for LoRa communication)
  - **Note:** Ensure you are using **RadioLib v6.6.0** specifically, newer versions may work, but the latest needs some refactoring.
- **mbedTLS** (for Base64 encoding/decoding)

## Installation

### 1. Install Required Libraries

- **RadioLib v6.6.0:**
  Install the library via Arduino Library Manager, or manually download it from [RadioLib GitHub](https://github.com/jgromes/RadioLib) and install version **6.6.0**.

  ```bash
  arduino-cli lib install "RadioLib"
  ```

- **ESP32Servo Library:**

  ```bash
  arduino-cli lib install "ESP32Servo"
  ```

- **mbedTLS Library:** Included with the ESP32 core libraries by default.

### 2. Flashing the LoKey and LoLock

The project is divided into two parts:
- **LoKey Code:** This code is flashed onto the Heltec WiFi LoRa 32 device that will act as the handheld control unit.
- **LoLock Code:** This code is flashed onto the Heltec WiFi LoRa 32 device that will act as the locking mechanism.

Ensure you upload the correct code to the appropriate device.

### 3. Wiring

- **LoKey Wiring:**
  - Connect 4 buttons to GPIO pins: `BUTTON_1 (16)`, `BUTTON_2 (17)`, `BUTTON_3 (18)`, `BUTTON_4 (19)`.
  - Connect 4 LEDs to GPIO pins: `LED_1 (4)`, `LED_2 (5)`, `LED_3 (20)`, `LED_4 (21)`.

- **LoLock Wiring:**
  - Connect a servo motor to GPIO pin: `SERVO_PIN (18)`.

### 4. Usage Instructions

- **Pairing:**
  Upon startup, **LoKey** sends a pairing request to **LoLock**. Once paired, both devices transition to an **IDLE** state.

- **Reset Command:**
  BUTTON 1 o **LoKey** held for 5 second will send a reset command to **LoLock**, which moves the servo back to the initial position and sends an acknowledgment to **LoKey**.

- **Locking/Unlocking:**
  **LoKey** sends commands to move the servo on **LoLock** to lock or unlock positions, with each action being acknowledged. (Button 4 to 1)

- **Heartbeat:**
  **LoLock** periodically sends a heartbeat message to **LoKey** to ensure the connection is active. If no heartbeat is received and all 4 LEDs are flashed. **LoKey** can trigger a reset.


## Troubleshooting

- **Issue:** Packets are not being received or decoded correctly.
  - **Solution:** Ensure you are using **RadioLib v6.6.0**. This version resolved critical packet-handling issues in the system.

- **Issue:** No acknowledgment received after sending a command.
  - **Solution:** Make sure **LoKey** switches back to **receive** mode after sending a command.

- **Issue:** Servo is not moving to the expected position.
  - **Solution:** Check the wiring of the servo and ensure the correct servoPositions are set for your use.
 
- **PR's** Only accepted if started with "No idiot, you do it like this..."
 
## TO-DO

- There some odd timing in packets transmitting on the LoLock code which rely on the Debug print statments to be in the code.... nothing im looking to fix anytime soon....
- I used code from another project on the LoKey controller, which used the heltec_unoffical library... This needs refactoring for the RadioLib library to put it in line with the LoLock device...
- **sometimes** during locking of position 1, for some reason LoKey controller seems to thing the button is being held for the reset command, which then fully unlocks the LoLock unit. which is stupid.

## Acknowledgments

This project utilizes the **RadioLib** library by [jgromes](https://github.com/jgromes) for LoRa communication.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.
