#define MADFLIGHT_VERSION "madflight v1.3.3-DEV"

/*==========================================================================================
madflight - Flight Controller for ESP32 / ESP32-S3 / RP2350 / RP2040 / STM32

MIT License

Copyright (c) 2023-2024 https://madflight.com

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

//Before including madflight.h define the all module options.
//for example: #define IMU_USE IMU_USE_SPI_MPU6500

#pragma once

#include "madflight/common/MF_Serial.h"
#include "madflight/common/MF_I2C.h"

MF_Serial *rcin_Serial = nullptr; //serial bus for RCIN
MF_Serial *gps_Serial = nullptr;  //serial bus for GPS
MF_I2C *mf_i2c = nullptr;         //I2C (Wire) bus for sensors

//include hardware specific code & default board pinout
#if defined ARDUINO_ARCH_ESP32
  #include "madflight/hw_ESP32/hw_ESP32.h"
#elif defined ARDUINO_ARCH_RP2040
  #include "madflight/hw_RP2040/hw_RP2040.h"
#elif defined ARDUINO_ARCH_STM32
  #include "madflight/hw_STM32/hw_STM32.h"
#else 
  #error "Unknown hardware architecture, expected ESP32 / RP2040 / STM32"
#endif

//default for ALT (Altitude Estimation)
#ifndef ALT_USE
  #define ALT_USE ALT_USE_BARO
#endif

//defaults for OUT
#ifndef HW_OUT_COUNT
  #define HW_OUT_COUNT 0
  #define HW_PIN_OUT_LIST {}
#endif

//defaults for madflight_setup();
#ifndef IMU_SAMPLE_RATE
  #define IMU_SAMPLE_RATE 1000
#endif
#ifndef BARO_SAMPLE_RATE
  #define BARO_SAMPLE_RATE 100
#endif
#ifndef IMU_GYR_LP_HZ
  #define IMU_GYR_LP_HZ 70
#endif
#ifndef IMU_ACC_LP_HZ
  #define IMU_ACC_LP_HZ 60
#endif
#ifndef IMU_MAG_LP_HZ
  #define IMU_MAG_LP_HZ 1e10
#endif

//include all modules
#include "madflight/ahrs/ahrs.h"
#include "madflight/alt/alt.h"
#include "madflight/baro/baro.h"
#include "madflight/bat/bat.h"
#include "madflight/bb/bb.h"
#include "madflight/cfg/cfg.h"
#include "madflight/cli/cli.h"
#include "madflight/gps/gps.h"
#include "madflight/imu/imu.h"
#include "madflight/led/led.h"
#include "madflight/mag/mag.h"
#include "madflight/out/out.h"
#include "madflight/pid/pid.h"
#include "madflight/rcin/rcin.h"
#include "madflight/veh/veh.h"

#ifndef MF_SETUP_DELAY_S 
  #define MF_SETUP_DELAY_S 6
#endif

//===============================================================================================
// HELPERS
//===============================================================================================

void madflight_warn_or_die(String msg, bool never_return) {
  bool do_print = true;
  do{
    if(do_print) Serial.print(msg + "\n");
    for(int i=0;i<20;i++) {
      led.toggle();
      uint32_t ts = millis();
      while(millis() - ts < 50) {
        if(cli.loop()) do_print = false; //process CLI commands, stop error output after first command
      } 
    }
  } while(never_return);
}
void madflight_die(String msg) { madflight_warn_or_die(msg, true); }
void madflight_warn(String msg) { madflight_warn_or_die(msg, false); }

//===============================================================================================
// madflight_setup()
//===============================================================================================

void madflight_setup() {
  led.setup(); //Set built in LED to turn on to signal startup
  Serial.begin(115200); //start console serial

  #ifndef MF_SETUP_FAST
    //6 second startup delay
    for(int i = 12; i > 0; i--) { 
      Serial.printf(MADFLIGHT_VERSION " starting %d ...\n", i);
      delay(500);
    } 
  #endif
  Serial.printf("Arduino library: " HW_ARDUINO_STR "\n");

  cli.print_boardInfo(); //print board info and pinout

  //hardware specific setup for busses: serial, spi and i2c (see hw_xxx.h)
  hw_setup();

  //prevent nullpointers
  if(!rcin_Serial) rcin_Serial = new MF_SerialNone();
  if(!gps_Serial) gps_Serial = new MF_SerialNone();
  if(!mf_i2c) mf_i2c = new MF_I2CNone();

  cfg.begin(); //read config from EEPROM
  cli.print_i2cScan(); //print i2c scan

  rcin.setup(); //Initialize radio communication.

  //IMU: keep on trying until no error is returned (some sensors need a couple of tries...)
  while(true) {
    int rv = imu.setup(IMU_SAMPLE_RATE); //request 1000 Hz sample rate, returns 0 on success, positive on error, negative on warning
    if(rv<=0) break;
    madflight_warn("IMU: init failed rv= " + String(rv) + ". Retrying...\n");
  }

  baro.setup(BARO_SAMPLE_RATE); //Barometer sample rate 100Hz
  mag.setup(); //External Magnetometer
  bat.setup(); //Battery Monitor
  bb.setup(); //Black Box
  gps_setup(); //GPS
  alt.setup(baro.alt); //Altitude Estimator

  ahrs.setup(IMU_GYR_LP_HZ, IMU_ACC_LP_HZ, IMU_MAG_LP_HZ); //setup low pass filters for Mahony/Madgwick filters
  ahrs.setInitalOrientation(); //do this before IMU update handler is started

  //start IMU update handler
  if(!imu_loop) Serial.println("=== WARNING === 'void imu_loop()' not defined.");
  imu.onUpdate = imu_loop;
  if(!imu.waitNewSample()) madflight_die("IMU interrupt not firing. Is HW_PIN_IMU_EXTI connected?");

  #ifndef MF_SETUP_FAST
    //Calibrate for zero gyro readings, assuming vehicle not moving when powered up. Comment out to only use cfg values. (Use CLI to calibrate acc.)
    cli.calibrate_gyro();
  #endif

  cli.welcome();

  led.enable(); //Set LED off to signal end of mf.startup, and enable blinking by imu_loop()
}
