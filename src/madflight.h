#define MADFLIGHT_VERSION "madflight v1.2.1-DEV"

/*==========================================================================================
madflight - Flight Controller for ESP32 / RP2040 / STM32

MIT License

Copyright (c) 2023-2024 qqqlab - https://github.com/qqqlab

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

#pragma once

//include hardware specific code & default board pinout
#if defined ARDUINO_ARCH_ESP32
  #include <madflight/hw_ESP32/hw_ESP32.h>
#elif defined ARDUINO_ARCH_RP2040
  #include <madflight/hw_RP2040/hw_RP2040.h>
#elif defined ARDUINO_ARCH_STM32
  #include <madflight/hw_STM32/hw_STM32.h>
#else 
  #error "Unknown hardware architecture, expected ESP32 / RP2040 / STM32"
#endif

//Outputs:
float out_command[HW_OUT_COUNT] = {0}; //Mixer outputs (values: 0.0 to 1.0)
PWM out[HW_OUT_COUNT]; //ESC and Servo outputs (values: 0.0 to 1.0)

//for testing individual modules use: #define MF_TEST  MF_TEST_LED | MF_TEST_RCIN
#define MF_TEST_BASE 0x0000
#define MF_TEST_CFG  0x0001
#define MF_TEST_LED  0x0002
#define MF_TEST_AHRS 0x0004
#define MF_TEST_RCIN 0x0008
#define MF_TEST_IMU  0x0010
#define MF_TEST_GPS  0x0020
#define MF_TEST_BARO 0x0040
#define MF_TEST_MAG  0x0080
#define MF_TEST_BAT  0x0100
#define MF_TEST_BB   0x0200
#define MF_TEST_CLI  0x0400


//include all modules. Before including madflight.h define the all module options, for example: #define IMU_USE IMU_USE_SPI_MPU6500
//load config first, so that cfg.xxx can be used by other modules
//load CLI last, so that it can access all other modules without using "extern". 

#if !defined(MF_TEST) || ((MF_TEST) & MF_TEST_CFG)
  #include "madflight/cfg/cfg.h"
#endif
#if !defined(MF_TEST) || ((MF_TEST) & MF_TEST_LED)
  #include "madflight/led/led.h"
#endif
#if !defined(MF_TEST) || ((MF_TEST) & MF_TEST_AHRS)
  #include "madflight/ahrs/ahrs.h"
#endif
#if !defined(MF_TEST) || ((MF_TEST) & MF_TEST_RCIN)
  #include "madflight/rcin/rcin.h"
#endif
#if !defined(MF_TEST) || ((MF_TEST) & MF_TEST_IMU)
  #include "madflight/imu/imu.h"
#endif
#if !defined(MF_TEST) || ((MF_TEST) & MF_TEST_GPS)
  #include "madflight/gps/gps.h"
#endif
#if !defined(MF_TEST) || ((MF_TEST) & MF_TEST_BARO)
  #include "madflight/baro/baro.h"
#endif
#if !defined(MF_TEST) || ((MF_TEST) & MF_TEST_MAG)
  #include "madflight/mag/mag.h"
#endif
#if !defined(MF_TEST) || ((MF_TEST) & MF_TEST_BAT)
  #include "madflight/bat/bat.h"
#endif
#if !defined(MF_TEST) || ((MF_TEST) & MF_TEST_BB)
  #include "madflight/bb/bb.h"
#endif
#if !defined(MF_TEST) || ((MF_TEST) & MF_TEST_CLI)
  #include "madflight/cli/cli.h" 
#endif


