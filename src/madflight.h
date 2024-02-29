#define MADFLIGHT_VERSION "madflight v1.0.4-DEV"

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

//include all modules. First set all USE_xxx and MODULE_xxx defines. For example: USE_MAG_QMC5883L and MAG_I2C_ADR
#include "madflight/cfg/cfg.h" //load config first, so that cfg.xxx can be used by other modules
#include "madflight/led/led.h"
#include "madflight/ahrs/ahrs.h"
#include "madflight/rcin/rcin.h"
#include "madflight/imu/imu.h"
#include "madflight/gps/gps.h"
#include "madflight/baro/baro.h"
#include "madflight/mag/mag.h"
#include "madflight/bat/bat.h"
#include "madflight/bb/bb.h"
#include "madflight/cli/cli.h" //load CLI last, so that it can access all other modules without using "extern". 