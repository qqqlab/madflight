# DTS6012M_UART Arduino Library

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Description

This library provides an interface for the DTS6012M single-point dToF (direct Time-of-Flight) distance sensor module using UART communication.

The DTS6012M is a compact sensor capable of measuring distances up to 20 meters with features like dual-target detection and good ambient light resistance. This library handles the UART protocol (frame parsing, CRC checking) required to communicate with the sensor and retrieve measurement data like distance and signal intensity.

This library is based on the DTS6012M User Manual V1.6 (dated 2024-07-26).

## Features

* Initializes UART communication with the sensor.
* Starts and stops the sensor's continuous measurement stream with `enableSensor()` and `disableSensor()`.
* Parses incoming data frames according to the datasheet protocol.
* Performs Modbus CRC-16 checksum validation for data integrity (can be optionally disabled for performance).
* Provides easy-to-use functions to retrieve:
    * Primary Target Distance (mm)
    * Primary Target Intensity
    * Secondary Target Distance (mm)
    * Secondary Target Intensity
    * Sunlight Base Level
    * Correction Values (Primary & Secondary)
* Allows disabling the CRC check via `enableCRC(false)` for potentially faster updates, at the risk of accepting corrupted data.
* Sensor enable/disable control for power management and measurement control.
* Includes example sketch demonstrating usage with enable/disable functionality.

## Hardware Requirements

* **DTS6012M Sensor Module:** The sensor this library is designed for.
* **Arduino Board:** An Arduino board with at least one available **HardwareSerial** port (e.g., `Serial1`, `Serial2`). Examples include Arduino Mega, Arduino Due, ESP32, STM32-based boards, etc.
    * **Note:** The default sensor baud rate (921600 bps) is generally **too high** for SoftwareSerial libraries. Using a HardwareSerial port is strongly recommended.
* **3.3V Power Supply:** The sensor requires a 3.3V supply for both Pin 1 (3V3_LASER) and Pin 2 (3V3). Ensure your Arduino can supply sufficient current or use an external 3.3V regulator.
* **Jumper Wires:** For making connections.
* **(Optional)** Logic Level Shifter (if connecting 5V Arduino TX to 3.3V sensor RX).

## Software Requirements

* **Arduino IDE:** Version 1.8.10 or later recommended.
* **This Library:** `DTS6012M_UART`

## Installation

1.  **Library Manager:**
    * Open the Arduino IDE.
    * Go to `Sketch` -> `Include Library` -> `Manage Libraries...`
    * Search for `DTS6012M_UART`.
    * Click `Install`.
2.  **Manual Installation:**
    * Download the latest release ZIP file from the repository.
    * In the Arduino IDE, go to `Sketch` -> `Include Library` -> `Add .ZIP Library...`
    * Select the downloaded ZIP file.
    * Alternatively, unzip the file and copy the `DTS6012M_UART` folder into your Arduino `libraries` directory (usually found in your Sketchbook location).
    * Restart the Arduino IDE.

## Wiring (UART Mode)

**Crucially, connect the sensor's GPIO pin (Pin 5) to GND *before* powering on the sensor to select UART mode.**

| DTS6012M Pin | Function        | Arduino Connection                                     | Notes                                                                  |
| :----------- | :-------------- | :----------------------------------------------------- | :--------------------------------------------------------------------- |
| Pin 1        | 3V3_LASER       | **Arduino 3.3V** | Sensor Laser Power                                                     |
| Pin 2        | 3V3             | **Arduino 3.3V** | Sensor Logic Power                                                     |
| Pin 3        | UART_TX/I2C_SDA | **Arduino Hardware RX Pin** (e.g., RX1 Pin 19 on Mega) | Sensor transmits data *to* Arduino                                     |
| Pin 4        | UART_RX/I2C_SCL | **Arduino Hardware TX Pin** (e.g., TX1 Pin 18 on Mega) | Sensor receives data *from* Arduino (Logic Level Shift Needed for 5V Arduinos) |
| Pin 5        | GPIO            | **Arduino GND** | **MUST be connected to GND before power-on for UART mode** |
| Pin 6        | GND             | **Arduino GND** | Common Ground                                                          |

**Logic Level Shifting:** If using a 5V Arduino board (like Uno, Nano), you **must** use a logic level shifter between the Arduino's TX pin (5V) and the sensor's RX pin (Pin 4, 3.3V tolerant) to avoid damaging the sensor. The sensor's TX output (3.3V) might be readable by a 5V Arduino RX pin, but check your Arduino board's specifications. Using a level shifter on both TX/RX lines is the safest approach.

## Basic Usage

```cpp
#include <Arduino.h>
#include "DTS6012M_UART.h" // 1. Include the library

// 2. Select the HardwareSerial port connected to the sensor
HardwareSerial &SensorSerial = Serial1; // Use Serial1, Serial2, etc.

// 3. Create an instance of the sensor library
DTS6012M_UART dtsSensor(SensorSerial);

void setup() {
  Serial.begin(115200); // For printing results
  while (!Serial);

  // 4. Initialize the sensor library (starts Serial1 at 921600 default)
  if (!dtsSensor.begin()) {
    Serial.println("Failed to initialize sensor!");
    while (1); // Halt
  }
  Serial.println("Sensor initialized.");

  // --- Optional: Disable CRC Check ---
  // For maximum performance, you can disable the CRC check.
  // This reduces processing overhead but increases the risk of using corrupted data if transmission errors occur.
  // dtsSensor.enableCRC(false); // CRC is ENABLED by default. Uncomment this line to disable it.
  
  // --- Optional: Control sensor enable/disable ---
  // dtsSensor.disableSensor(); // Stop measurements
  // dtsSensor.enableSensor();  // Resume measurements
}

void loop() {
  // 5. Call update() frequently to process incoming data
  bool newData = dtsSensor.update();

  // 6. Check if new data was received
  if (newData) {
    // 7. Get the data
    uint16_t distance = dtsSensor.getDistance(); // in mm
    uint16_t intensity = dtsSensor.getIntensity();

    Serial.print("Distance: ");
    if (distance == 0xFFFF) {
      Serial.print("Invalid/OOR");
    } else {
      Serial.print(distance);
      Serial.print(" mm");
    }
    Serial.print("\t Intensity: ");
    Serial.println(intensity);
  }

  // Do other things...
  delay(10); // Keep loop running reasonably fast
}