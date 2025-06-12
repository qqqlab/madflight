#pragma once

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include "STM32_PWM.h" //Servo and oneshot
#include "Dshot/Dshot.h"
#include "DshotBidir/DshotBidir.h"

#include <STM32FreeRTOS.h>
#define MF_FREERTOS_DEFAULT_STACK_SIZE 512 //stack size in 32bit words