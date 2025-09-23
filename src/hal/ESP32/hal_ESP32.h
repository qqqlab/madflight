//NOTE: all files in this directory should be .h headers, or use #ifdef ARDUINO_ARCH_ESP32 guards to 
//stop the Arduino builder from compiling these source files when a different target is selected

#pragma once

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
