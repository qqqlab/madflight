#define MADFLIGHT_VERSION "madflight v2.0.0-DEV"

//madflight.h - Flight Controller for ESP32 / ESP32-S3 / RP2350 / RP2040 / STM32

/*==========================================================================================
MIT License

Copyright (c) 2023-2025 https://madflight.com

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

//#pragma once //don't use here, we want to get an error if included twice

// madflight config string
#ifndef MADFLIGHT_BOARD
  #define MADFLIGHT_BOARD ""
#endif
#ifndef MADFLIGHT_CONFIG
  #define MADFLIGHT_CONFIG ""
#endif
const char* madflight_config = MADFLIGHT_BOARD MADFLIGHT_CONFIG;

// bus abstraction
#include "madflight/hal/MF_Serial.h"
#include "madflight/hal/MF_I2C.h"

// include all "_cpp.h" modules which have compile time config options
#define MF_ALLOW_INCLUDE_CCP_H
#include "madflight/ahr/ahr_cpp.h" //TODO - convert to use gizmos
#include "madflight/alt/alt_cpp.h" //TODO - convert to use gizmos
#include "madflight/hal/hal_cpp.h"
#include "madflight/imu/imu_cpp.h" //for IMU_EXEC
#undef MF_ALLOW_INCLUDE_CCP_H

// include all other modules without compile time config options
#include "madflight/cfg/cfg.h"
#include "madflight/cli/cli.h"
#include "madflight/bar/bar.h"
#include "madflight/bat/bat.h"
#include "madflight/bbx/bbx.h"
#include "madflight/gps/gps.h"
#include "madflight/led/led.h"
#include "madflight/mag/mag.h"
#include "madflight/out/out.h"
#include "madflight/pid/pid.h"
#include "madflight/rcl/rcl.h"
#include "madflight/rdr/rdr.h"
#include "madflight/veh/veh.h"

// toolbox
#include "madflight/tbx/RuntimeTrace.h"

// prototypes
void madflight_die(String msg);
void madflight_warn(String msg);
void madflight_warn_or_die(String msg, bool die);

//===============================================================================================
// madflight_setup()
//===============================================================================================

void madflight_setup() {
  Serial.begin(115200); //start console serial

  // 6 second startup delay
  for(int i = 12; i > 0; i--) {
    Serial.printf(MADFLIGHT_VERSION " starting %d ...\n", i);
    #ifndef MF_DEBUG 
      delay(500);
    #else
      delay(100);
    #endif
  } 

  Serial.printf("Arduino library: " HAL_ARDUINO_STR "\n");

  #ifdef HAL_BOARD_NAME
    Serial.println("Board: " HAL_BOARD_NAME);
  #endif

  #ifdef HAL_MCU
    Serial.println("Processor: " HAL_MCU);
  #endif

  // CFG - Configuration parameters
  cfg.begin(); 
  #ifdef MF_CONFIG_CLEAR
    cfg.clear();
    cfg.writeToEeprom();
    madflight_die("Config cleared. comment out '#define MF_CONFIG_CLEAR' and upload again.");
  #endif
  cfg.loadFromEeprom(); //load parameters from EEPROM
  if(madflight_config) {
    if(cfg.load_madflight_param(madflight_config)) {
      Serial.println("CFG: madflight_config - Loaded");
    }else{
      Serial.println("CFG: madflight_config - Skipped (was aready stored in eeprom)");
    }
  }

  #ifdef MF_DEBUG
    if(madflight_config) {
      Serial.println("\nDEBUG: madflight_config ==============\n");
      Serial.println(madflight_config);
    }
    Serial.println("\nDEBUG: cfg.list() ================\n");
    cfg.list();
  #endif

  cfg.printPins();

  // LED - Setup LED
  led.config.pin = cfg.pin_led;
  led.config.on_value = cfg.led_on;
  led.setup();
  led.on(); //turn on to signal startup
  led.enabled = false; //do not change state until setup compled

  // HAL - Hardware abstraction layer setup: serial, spi, i2c (see hal.h)
  hal_setup();

  cli.print_i2cScan(); //print i2c scan

  rcl.config.gizmo = (Cfg::rcl_gizmo_enum)cfg.rcl_gizmo; //the gizmo to use
  rcl.config.ser_bus_id = cfg.rcl_ser_bus; //serial bus id
  rcl.config.baud = cfg.rcl_baud; //baud rate
  rcl.config.ppm_pin = cfg.getValue("pin_ser" + String(cfg.rcl_ser_bus) + "_rx", -1);
  rcl.setup(); //Initialize radio communication.

  // BAR - Barometer
  bar.config.gizmo = (Cfg::bar_gizmo_enum)cfg.bar_gizmo; //the gizmo to use
  bar.config.i2c_bus = hal_get_i2c_bus(cfg.bar_i2c_bus); //i2c bus
  bar.config.i2c_adr = cfg.bar_i2c_adr; //i2c address. 0=default address  
  bar.config.sampleRate = 100; //sample rate [Hz]
  bar.setup();

  // MAG - External Magnetometer
  mag.config.gizmo = (Cfg::mag_gizmo_enum)cfg.mag_gizmo; //the gizmo to use
  mag.config.i2c_bus = hal_get_i2c_bus(cfg.mag_i2c_bus); //i2c bus
  mag.config.i2c_adr = cfg.mag_i2c_adr; //i2c address. 0=default address
  mag.config.sampleRate = 100; //sample rate [Hz]
  mag.setup(); 

  // BAT - Battery Monitor
  bat.config.gizmo = (Cfg::bat_gizmo_enum)cfg.bat_gizmo; //the gizmo to use
  bat.config.i2c_bus = hal_get_i2c_bus(cfg.bat_i2c_bus); //i2c bus
  bat.config.i2c_adr = cfg.bat_i2c_adr; //i2c address. 0=default address
  bat.config.sampleRate = 100; //sample rate [Hz]
  bat.config.adc_pin_v = cfg.pin_bat_v;
  bat.config.adc_pin_i = cfg.pin_bat_i;
  bat.config.adc_cal_v = cfg.bat_cal_v;
  bat.config.adc_cal_i = cfg.bat_cal_i;
  bat.config.rshunt = cfg.bat_cal_i;
  bat.setup();

  //RDR
  rdr.config.gizmo = (Cfg::rdr_gizmo_enum)cfg.rdr_gizmo; //the gizmo to use
  rdr.config.ser_bus_id = cfg.rdr_ser_bus; //serial bus
  rdr.config.baud = cfg.rdr_baud; //baud rate
  rdr.setup();


  // GPS
  gps.config.gizmo = (Cfg::gps_gizmo_enum)cfg.gps_gizmo; //the gizmo to use
  gps.config.ser_bus_id = cfg.gps_ser_bus; //serial bus id
  gps.config.baud = cfg.gps_baud; //baud rate
  gps.setup();

  // BBX - Black Box
  bbx.config.gizmo = (Cfg::bbx_gizmo_enum)cfg.bbx_gizmo; //the gizmo to use
  bbx.config.spi_bus = hal_get_spi_bus(cfg.bbx_spi_bus); //SPI bus
  bbx.config.spi_cs = cfg.pin_bbx_cs; //SPI select pin
  bbx.config.pin_mmc_dat = cfg.pin_mmc_dat;
  bbx.config.pin_mmc_clk = cfg.pin_mmc_clk;
  bbx.config.pin_mmc_cmd = cfg.pin_mmc_cmd;
  bbx.setup();

  // ALT - Altitude Estimator
  alt.setup(bar.alt); 
  
  // AHR - setup low pass filters for AHRS filters
  ahr.setup(cfg.imu_gyr_lp, cfg.imu_acc_lp, cfg.mag_lp);

  // IMU - Intertial Measurement Unit (gyro/acc/mag)
  imu.config.sampleRate = cfg.imu_rate; //sample rate [Hz]
  imu.config.pin_int = cfg.pin_imu_int; //IMU data ready interrupt pin
  imu.config.gizmo = (Cfg::imu_gizmo_enum)cfg.imu_gizmo; //the gizmo to use
  imu.config.spi_bus = hal_get_spi_bus(cfg.imu_spi_bus); //SPI bus
  imu.config.spi_cs = cfg.pin_imu_cs; //SPI select pin
  imu.config.i2c_bus = hal_get_i2c_bus(cfg.imu_i2c_bus); //I2C bus (only used if spi_bus==nullptr)
  imu.config.i2c_adr = cfg.imu_i2c_adr; //i2c address. 0=default address
  // Some sensors need a couple of tries...
  int tries = 10;
  while(true) {
    int rv = imu.setup(); //request 1000 Hz sample rate, returns 0 on success, positive on error, negative on warning
    if(rv<=0) break;
    tries--;
    madflight_warn_or_die("IMU init failed rv= " + String(rv) + ".", (tries <= 0) );
  }
  if(!imu.installed() && (Cfg::imu_gizmo_enum)cfg.imu_gizmo != Cfg::imu_gizmo_enum::mf_NONE) {
    madflight_die("IMU install failed.");
  }
  // start IMU update handler
  if(imu.installed()) {
    ahr.setInitalOrientation(); //do this before IMU update handler is started

    if(!imu_loop) madflight_warn("'void imu_loop()' not defined.");
    imu.onUpdate = imu_loop;
    if(!imu.waitNewSample()) madflight_die("IMU interrupt not firing. Is pin 'pin_imu_int' connected?");

    #ifndef MF_DEBUG
      //Calibrate for zero gyro readings, assuming vehicle not moving when powered up. Comment out to only use cfg values. (Use CLI to calibrate acc.)
      cli.calibrate_gyro();
    #endif
  }

  // CLI - Command Line Interface
  cli.begin();

  // Enable LED, and switch it off signal end of startup.
  led.enabled = true; 
  led.off();
}

//===============================================================================================
// HELPERS
//===============================================================================================

void madflight_die(String msg) {
  bool do_print = true;
  led.enabled = true;
  for(;;) {
    if(do_print) Serial.print("FATAL ERROR: " + msg + " Use CLI or reboot.\n");
    for(int i=0;i<20;i++) {
      led.toggle();
      uint32_t ts = millis();
      while(millis() - ts < 50) {
        if(cli.update()) do_print = false; //process CLI commands, stop error output after first command
        rcl.update(); //keep rcl (mavlink?) running
      } 
    }
  }
}
void madflight_warn(String msg) { 
  Serial.print("WARNING: " + msg + "\n");
  //flash LED for 1 second
  for(int i=0;i<20;i++) {
    led.toggle();
    delay(50);
  }
}

void madflight_warn_or_die(String msg, bool die) {
  if(die) {
    madflight_die(msg);
  }else{
    madflight_warn(msg);
  }
}
