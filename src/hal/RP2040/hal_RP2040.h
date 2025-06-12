#pragma once

#include <Arduino.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include "RP2040_PWM.h"  //Servo and oneshot
#include "Dshot/Dshot.h"
#include "DshotBidir/DshotBidir.h"

#define MF_FREERTOS_DEFAULT_STACK_SIZE 512 //stack size in 32bit words
