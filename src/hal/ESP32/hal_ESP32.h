#pragma once

#ifndef IMU_EXEC
  #define IMU_EXEC IMU_EXEC_FREERTOS //ESP32 always uses FreeRTOS on core0 (can't used float on core1)
#endif
#ifndef IMU_FREERTOS_TASK_PRIORITY
  #define IMU_FREERTOS_TASK_PRIORITY (configMAX_PRIORITIES - 1) //IMU Interrupt task priority, higher number is higher priority.
#endif

/*--------------------------------------------------------------------------------------------------
  IMPORTANT
  
  ESP32 Wire has a bug in I2C which causes the bus to hang for 1 second after a failed read, which can 
  happen a couple times per minute. This makes Wire I2C for IMU not a real option... 
  See --> https://github.com/espressif/esp-idf/issues/4999

  Uncomment USE_ESP32_SOFTWIRE to use software I2C, but this does not work well with all sensors...
  
  So, until a better I2C solution is available: use an SPI IMU sensor on ESP32!!!!
----------------------------------------------------------------------------------------------------*/  
//#define USE_ESP32_SOFTWIRE //uncomment to use SoftWire instead of Wire

#define HAL_SER_NUM 3
#define HAL_I2C_NUM 2
#define HAL_SPI_NUM 2

#include <Arduino.h>
#include "ESP32_PWM.h"      //Servo and onshot
#include "Dshot/Dshot.h"
#include "DshotBidir/DshotBidir.h"

#define MF_FREERTOS_DEFAULT_STACK_SIZE 2048 //stack size on ESP32 is in bytes, not in 32bit words

//Arduino version string
#ifndef ESP_ARDUINO_VERSION_MAJOR
  #define ESP_ARDUINO_VERSION_MAJOR 0
#endif
#ifndef ESP_ARDUINO_VERSION_MINOR
  #define ESP_ARDUINO_VERSION_MINOR 0
#endif
#ifndef ESP_ARDUINO_VERSION_PATCH
  #define ESP_ARDUINO_VERSION_PATCH 0
#endif
#define mf_df2xstr(s)              #s
#define mf_df2str(s)               mf_df2xstr(s)
#define HAL_ARDUINO_STR "Arduino-ESP32 v" mf_df2str(ESP_ARDUINO_VERSION_MAJOR) "." mf_df2str(ESP_ARDUINO_VERSION_MINOR) "." mf_df2str(ESP_ARDUINO_VERSION_PATCH)

#ifndef MF_MCU_NAME
  #ifdef CONFIG_IDF_TARGET_ESP32S3
    #define MF_MCU_NAME "ESP32-S3"
  #elif defined CONFIG_IDF_TARGET_ESP32
    #define MF_MCU_NAME "ESP32"
  #else
    #define MF_MCU_NAME "ESP32 (unsupported type)"
  #endif
#endif