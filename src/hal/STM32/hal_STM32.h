#pragma once

//This hack bumps the interrupt priority of the IMU interrupt. Without this hack the IMU interrupt 
//does not fire, it aparently gets masked by FreeRTOS
//see https://github.com/qqqlab/madflight/issues/79 and https://github.com/qqqlab/madflight/pull/81
#ifndef MF_HACK_STM32_INTERRUPT_PRIORITY
  #define MF_HACK_STM32_INTERRUPT_PRIORITY 1
#endif

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include "STM32_PWM.h" //Servo and oneshot
#include "Dshot/Dshot.h"
#include "DshotBidir/DshotBidir.h"

#include <STM32FreeRTOS.h>
#define MF_FREERTOS_DEFAULT_STACK_SIZE 512 //stack size in 32bit words

//Arduino version string
#define df2xstr(s)              #s
#define df2str(s)               df2xstr(s)
#define HAL_ARDUINO_STR "Arduino_Core_STM32 v" df2str(STM32_CORE_VERSION_MAJOR) "." df2str(STM32_CORE_VERSION_MINOR) "." df2str(STM32_CORE_VERSION_PATCH)