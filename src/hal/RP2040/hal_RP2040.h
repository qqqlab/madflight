/*==========================================================================================
MIT License

Copyright (c) 2023-2026 https://madflight.com

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

//serial driver selection
#ifndef MF_SERIAL_DMA
  #define MF_SERIAL_DMA 1
#endif

#ifndef IMU_EXEC
  #define IMU_EXEC IMU_EXEC_FREERTOS_OTHERCORE
  //#define IMU_EXEC IMU_EXEC_FREERTOS
#endif

#ifndef IMU_FREERTOS_TASK_PRIORITY
  #define IMU_FREERTOS_TASK_PRIORITY (configMAX_PRIORITIES - 2) // =6; prio 7 is used for IdleCode0,1 tasks for flash operations; prio4 is used for CORE0,1 (loop,loop1) tasks
#endif

#define HAL_SER_NUM 2
#define HAL_I2C_NUM 2
#define HAL_SPI_NUM 2


#include <Arduino.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include "RP2040_PWM.h"  //Servo and oneshot
#include "Dshot/Dshot.h"
#include "DshotBidir/DshotBidir.h"
#include "pio_registry.h"

#define MF_FREERTOS_DEFAULT_STACK_SIZE 512 //stack size in 32bit words

//Arduino-pico version string
#define HAL_ARDUINO_STR "Arduino-Pico v" ARDUINO_PICO_VERSION_STR

//get processor type
#ifndef MF_MCU_NAME
  #ifdef PICO_RP2350
    #if !PICO_RP2350A
      #define MF_MCU_NAME "RP2350B (48 GPIO)"
    #else
      #define MF_MCU_NAME "RP2350A (30 GPIO)"
    #endif
  #else
    #define MF_MCU_NAME "RP2040"
  #endif
#endif

//check arduino-pico v5 or later
#if ARDUINO_PICO_MAJOR < 5
  #error "Arduino Pico version 5 or later is required"
#endif

//check FreeRTOS enabled
#if !__FREERTOS
  #error "FreeRTOS required - Arduino IDE menu: Tools->Operating System = FreeRTOS SMP - PlatformIO add: build_flags = -DPIO_FRAMEWORK_ARDUINO_ENABLE_FREERTOS"
#endif
