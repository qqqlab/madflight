//modified for madflight: MF_Serial

#ifndef DTS6012M_UART_H
#define DTS6012M_UART_H

#include <Arduino.h>
#include "../../hal/MF_Serial.h"

// --- Constants based on datasheet ---
// Protocol constants
const byte DTS_HEADER = 0xA5;
const byte DTS_DEVICE_NO = 0x03;
const byte DTS_DEVICE_TYPE = 0x20; // Example type from datasheet

// Command codes
const byte DTS_CMD_START_STREAM = 0x01;
const byte DTS_CMD_STOP_STREAM = 0x02;
const byte DTS_CMD_GET_VERSION = 0x0A;
const byte DTS_CMD_SET_BAUD = 0x10;  //write 1 byte: 0:9600, 1:14400, 2:19200, 3:38400, 4:43000, 5:57600, 6:76800, 7:115200, 8:12800, 9:230400, 0x0A:256000, 0x0B:460800, 0x0E:921600
const byte DTS_CMD_GET_BAUD = 0x11;
const byte DTS_CMD_SET_I2C_ADDR = 0x12;
const byte DTS_CMD_GET_I2C_ADDR = 0x13;
const byte DTS_CMD_SET_FRAME_RATE = 0x1A; //write 1 byte: 0:50, 1:100, 2:250 fps (default 100 fps)
const byte DTS_CMD_GET_FRAME_RATE = 0x1B;


// Response frame structure constants for start_stream command (0x01)
const int DTS_RESPONSE_FRAME_LENGTH = 23; // Full frame size: Header(1)+DevNo(1)+DevType(1)+CMD(1)+Res(1)+Len(2)+Data(14)+CRC(2)
const int DTS_DATA_LENGTH_EXPECTED = 14;  // Expected data payload size

// Indices within the 14-byte data payload (LSB first ordering)
const int DTS_IDX_SEC_DIST = 0; // Secondary Target Distance (2 bytes)
const int DTS_IDX_SEC_CORR = 2; // Secondary Target Correction (2 bytes)
const int DTS_IDX_SEC_INT = 4;  // Secondary Target Intensity (2 bytes)
const int DTS_IDX_PRI_DIST = 6; // Primary Target Distance (2 bytes)
const int DTS_IDX_PRI_CORR = 8; // Primary Target Correction (2 bytes)
const int DTS_IDX_PRI_INT = 10; // Primary Target Intensity (2 bytes)
const int DTS_IDX_SUN_BASE = 12;// Sunlight Base (2 bytes)


class DTS6012M_UART {
public:
  // --- Public Methods ---

  DTS6012M_UART();

  // Initialization: Starts serial communication at the specified baud rate
  // Sends the 'Start Stream' command automatically.
  // Returns true on success, false if the serial port failed to start.
  bool begin(MF_Serial *serialPort, int baudRate = 921600); // Default baud rate from datasheet
  void startStream();
  bool sendCommandBaud(int baudRate);
  int sendCommandFrameRate(int framerate);
  // Update: Processes incoming serial data. Call this frequently in your loop().
  // Returns true if a new, valid measurement frame was received and parsed, false otherwise.
  bool update();

  // --- Data Getters ---
  // These return the values from the last successfully parsed measurement frame.

  int16_t getDistance();           // Primary target distance (mm). Returns -1 if no target detected or invalid.
  uint16_t getIntensity();          // Primary target intensity/signal strength.
  uint16_t getSunlightBase();       // Ambient sunlight base level.
  uint16_t getCorrection();         // Primary target correction value (meaning may vary).
  int16_t getSecondaryDistance();  // Secondary target distance (mm). Returns -1 if no target detected or invalid.
  uint16_t getSecondaryIntensity(); // Secondary target intensity/signal strength.
  uint16_t getSecondaryCorrection();// Secondary target correction value.

  // --- Advanced Methods ---

  // Send Command: Allows sending other commands defined in the datasheet (e.g., stop stream, set baud).
  // cmd: The command code (e.g., DTS_CMD_STOP_STREAM).
  // dataPayload: Pointer to byte array containing data for the command (if any).
  // payloadLength: Number of bytes in dataPayload.
  void sendCommand(byte cmd, const byte *dataPayload = NULL, uint16_t payloadLength = 0);

  // Enable/Disable CRC Check: Controls whether the CRC checksum is validated on incoming frames.
  // Disabling improves performance but risks accepting corrupted data. Enabled by default.
  void enableCRC(bool enable);

  // Enable Sensor: Starts the measurement stream from the sensor.
  void enableSensor();

  // Disable Sensor: Stops the measurement stream from the sensor.
  void disableSensor();

private:
  // --- Private Members ---
  MF_Serial *_serial; // Reference to the hardware serial port instance

  byte _rxBuffer[DTS_RESPONSE_FRAME_LENGTH]; // Buffer to hold incoming frame bytes
  int _rxBufferIndex;                        // Current position in the rx buffer

  // Variables to store the latest valid measurement data
  int16_t _distancePrimary_mm;
  uint16_t _intensityPrimary;
  uint16_t _correctionPrimary;
  int16_t _distanceSecondary_mm;
  uint16_t _intensitySecondary;
  uint16_t _correctionSecondary;
  uint16_t _sunlightBase;
  bool _crcCheckEnabled;  // Flag to control CRC validation


  // --- Private Helper Methods ---

  // parseFrame: Validates and extracts data from the completed buffer.
  // Returns true if the frame is valid (correct format, length, CRC), false otherwise.
  bool parseFrame();

  // calculateCRC16: Computes the Modbus CRC-16 checksum for given data.
  uint16_t calculateCRC16(const byte *data, int len);
};

#endif // DTS6012M_UART_H
