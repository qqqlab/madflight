#define MADFLIGHT_VERSION "madflight v1.3.0-DEV"

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
#define MF_TEST_OUT  0x0800

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
  #include "madflight/rcin/telem.h"
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
#if !defined(MF_TEST) || ((MF_TEST) & MF_TEST_OUT)
  #include "madflight/out/out.h" 
#endif
#if !defined(MF_TEST) || ((MF_TEST) & MF_TEST_CLI)
  #include "madflight/cli/cli.h" 
#endif


#if !defined(MF_TEST)

  extern void __attribute__((weak)) imu_loop();

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
    hw_setup(); //hardware specific setup for spi and Wire (see hw_xxx.h)
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

#endif