/*==========================================================================================
MIT License

Copyright (c) 2026 https://madflight.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
===========================================================================================*/

// Board definition for the Raspberry Pi Pico 2 (RP2350) + GY-87 drone firmware.
//
// The pinout is the standard "default_RP2040.h" Raspberry Pi Pico pinout:
//   UART0  GPIO0 = TX, GPIO1 = RX   (RC control input)
//   UART1  GPIO8 = TX, GPIO9 = RX   (telemetry output)
//   I2C0   GPIO20 = SDA, GPIO21 = SCL (GY-87: MPU6050 + BMP280 + HMC5883L)
//   OUT0..OUT3 GPIO2, GPIO3, GPIO4, GPIO5  (motor 1..4)
//   GPIO22 MPU6050 data ready interrupt
//   GPIO25 status LED
//
// The MF_BOARD_NAME and MF_MCU_NAME macros are already defined by the HAL
// (hal/RP2040/hal_RP2040.h), so they are undefined here to avoid a macro
// redefinition warning.

#undef MF_BOARD_NAME
#undef MF_MCU_NAME

#include "default_RP2040.h"
