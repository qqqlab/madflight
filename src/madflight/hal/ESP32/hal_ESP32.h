//NOTE: all files in this directory should be .h headers, or use #ifdef ARDUINO_ARCH_ESP32 guards to 
//stop the Arduino builder from compiling these source files when a different target is selected

#pragma once

#include <Arduino.h>
#include "ESP32_PWM.h"      //Servo and onshot

#define MF_FREERTOS_DEFAULT_STACK_SIZE 2048 //stack size on ESP32 is in bytes, not in 32bit words