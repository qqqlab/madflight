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

//madflight.h - Flight Controller for ESP32 / ESP32-S3 / RP2350 / RP2040 / STM32

//#pragma once //don't use here, we want to get an error if this file is included twice

extern const char madflight_config[];

#ifdef MF_BOARD
  //the board header file must define const char madflight_board[] and should define MF_BOARD_NAME, MF_MCU_NAME
  #include MF_BOARD
#else
  const char madflight_board[] = "";
#endif

#ifndef ALT_USE
  #define ALT_USE ALT_USE_KALMAN3
#endif

#include <Arduino.h> //keep PlatformIO happy
#include "madflight_version.h"

// bus abstraction
#include "hal/MF_Serial.h"
#include "hal/MF_I2C.h"

// include all "_cpp.h" modules which have compile time config options
#define MF_ALLOW_INCLUDE_CCP_H
#include "alt/alt_cpp.h" //Altitude estimator (TODO - convert to use gizmos)
#include "hal/hal_cpp.h"
#include "imu/imu_cpp.h" //for IMU_EXEC
#undef MF_ALLOW_INCLUDE_CCP_H

// include all other modules without compile time config options
#include "ahr/ahr.h" //AHRS
#include "cfg/cfg.h" //Config
#include "cli/cli.h" //Command Line Interface
#include "bar/bar.h" //Barometer sensor
#include "bat/bat.h" //Battery sensor
#include "bbx/bbx.h" //Blackbox SDCARD
#include "gps/gps.h" //GPS
#include "led/led.h" //LED
#include "lua/lua.h" //Lua scripting
#include "mag/mag.h" //Magnetometer sensor
#include "ofl/ofl.h" //Optical flow sensor
#include "out/out.h" //Outputs (motor, servo)
#include "pid/pid.h" //PIDController control
#include "rcl/rcl.h" //RC radio link
#include "rdr/rdr.h" //Radar, lidar, ultrasonic sensors
#include "tbx/tbx.h" //Toolbox common tools
#include "veh/veh.h" //Vehicle info

// toolbox
#include "tbx/RuntimeTrace.h"

// prototypes
void madflight_die(String msg);
void madflight_warn(String msg);
void madflight_warn_or_die(String msg, bool die);

//===============================================================================================
// madflight_setup()
//===============================================================================================

// vehicle setup by defines VEH_TYPE, VEH_FLIGHTMODE_AP_IDS, VEH_FLIGHTMODE_NAMES
#ifndef VEH_TYPE
  #define VEH_TYPE VEH_TYPE_GENERIC
#endif
#ifndef VEH_FLIGHTMODE_AP_IDS
  #define VEH_FLIGHTMODE_AP_IDS {0,1,2,3,4,5}
#endif
#ifndef VEH_FLIGHTMODE_NAMES
  #define VEH_FLIGHTMODE_NAMES {"FM0","FM1","FM2","FM3","FM4","FM5"}
#endif
const uint8_t Veh::mav_type = VEH_TYPE; //mavlink vehicle type
const uint8_t Veh::flightmode_ap_ids[6] = VEH_FLIGHTMODE_AP_IDS; //mapping from flightmode to ArduPilot flight mode id
const char* Veh::flightmode_names[6] = VEH_FLIGHTMODE_NAMES; //define flightmode name strings for telemetry

void madflight_setup() {
  hal_usb_setup(); //setup USB CDC/MSC
  Serial.begin(115200); //start console serial
  
  // CFG - Configuration parameters (execute before delay to start LED)
  cfg.begin();
  #ifdef MF_CONFIG_CLEAR
    cfg.clear();
    cfg.writeToEeprom();
    madflight_die("Config cleared. comment out '#define MF_CONFIG_CLEAR' and upload again.");
  #endif
  cfg.loadFromEeprom(); //load parameters from EEPROM
  cfg.load_madflight(madflight_board, madflight_config); //load config

  // LED - Setup LED (execute before delay)
  led.config.gizmo = (Cfg::led_gizmo_enum)cfg.led_gizmo;
  led.config.pin = cfg.pin_led;
  led.setup();
  led.color(0x0000ff); //turn on blue to signal startup
  led.enabled = false; //do not change state until setup compled

  // BBX - Black Box (execute before delay to start USB-MSC)
  bbx.config.gizmo = (Cfg::bbx_gizmo_enum)cfg.bbx_gizmo; //the gizmo to use
  bbx.config.spi_bus = hal_get_spi_bus(cfg.bbx_spi_bus); //SPI bus
  bbx.config.spi_cs = cfg.pin_bbx_cs; //SPI select pin
  bbx.config.pin_mmc_dat = cfg.pin_mmc_dat;
  bbx.config.pin_mmc_clk = cfg.pin_mmc_clk;
  bbx.config.pin_mmc_cmd = cfg.pin_mmc_cmd;
  bbx.setup();

  // 6 second startup delay
  for(int i = 12; i > 0; i--) {
    Serial.printf(MADFLIGHT_VERSION " starting %d ...\n", i);
    Serial.flush();
    #ifndef MF_DEBUG 
      delay(500);
    #else
      delay(100);
    #endif
  } 

  Serial.printf("Arduino library: " HAL_ARDUINO_STR "\n");

  #ifdef MF_BOARD_NAME
    Serial.println("Board: " MF_BOARD_NAME);
  #endif

  #ifdef MF_MCU_NAME
    Serial.println("Processor: " MF_MCU_NAME);
  #endif

  // Rerun CFG to show output after startup delay
  cfg.clear();
  cfg.loadFromEeprom(); //load parameters from EEPROM
  cfg.load_madflight(madflight_board, madflight_config); //load config

  // Pins sorted by GPIO number
  cfg.printPins();

  // HAL - Hardware abstraction layer setup: serial, spi, i2c (see hal.h)
  hal_setup();

  cli.print_i2cScan(); //print i2c scan

  // LED and BBX summary
  cfg.printModule("led");
  bbx.printSummary();

  // RCL - Radio Control Link
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
  rdr.config.rdr_ser_bus  = cfg.rdr_ser_bus; //serial bus
  rdr.config.rdr_baud     = cfg.rdr_baud; //baud rate
  rdr.config.pin_rdr_trig = cfg.pin_rdr_trig;
  rdr.config.pin_rdr_echo = cfg.pin_rdr_echo;
  rdr.config.rdr_i2c_bus  = cfg.rdr_i2c_bus;
  rdr.config.rdr_i2c_adr  = cfg.rdr_i2c_adr;
  rdr.setup();

  //OFL
  ofl.config.ofl_gizmo    = (Cfg::ofl_gizmo_enum)cfg.ofl_gizmo; //the gizmo to use
  ofl.config.ofl_spi_bus  = cfg.ofl_spi_bus; // spi bus
  ofl.config.pin_ofl_cs   = cfg.pin_ofl_cs;  // spi cs pin
  ofl.config.ofl_ser_bus  = cfg.ofl_ser_bus; // serial bus
  ofl.config.ofl_baud     = cfg.ofl_baud;    // baud rate (0 = default)
  ofl.config.ofl_align    = (Cfg::ofl_align_enum)cfg.ofl_align;  // xy-axis orientation. Example: ES means positive x-axis points East (right) and positive y-axis points South (back)
  ofl.config.ofl_cal_rad  = cfg.ofl_cal_rad; // manual calibration factor from pixels to radians, leave at 0 to use calibration from gizmo
  ofl.setup();

  // GPS
  gps.config.gizmo = (Cfg::gps_gizmo_enum)cfg.gps_gizmo; //the gizmo to use
  gps.config.ser_bus_id = cfg.gps_ser_bus; //serial bus id
  gps.config.baud = cfg.gps_baud; //baud rate
  gps.setup();

  // ALT - Altitude Estimator
  if(rdr.installed()) {
    alt.setup(rdr.dist);
  }else{
    alt.setup(bar.alt);
  }

  // AHR - setup low pass filters for AHRS filters
  ahr.config.gizmo = (Cfg::ahr_gizmo_enum)cfg.ahr_gizmo; //the gizmo to use
  ahr.config.gyrLpFreq = cfg.imu_gyr_lp; //gyro low pass filter freq [Hz]
  ahr.config.accLpFreq = cfg.imu_acc_lp; //accelerometer low pass filter freq [Hz]
  ahr.config.magLpFreq = cfg.mag_lp; //magnetometer low pass filter freq [Hz]
  ahr.config.pimu = &imu; //pointer to Imu to use
  ahr.config.pmag = &mag; //pointer to Mag to use
  ahr.config.gyr_offset = &(cfg.imu_cal_gx); //gyro offset[3] [deg/sec]
  ahr.config.acc_offset = &(cfg.imu_cal_ax); //acc offset[3] [G]
  ahr.config.mag_offset = &(cfg.mag_cal_x); //mag offset[3] [adc_lsb]
  ahr.config.mag_scale = &(cfg.mag_cal_sx); //mag scale[3] [uT/adc_lsb]
  ahr.setup();

  // IMU - Intertial Measurement Unit (gyro/acc/mag)
  imu.config.sampleRate = cfg.imu_rate; //sample rate [Hz]
  imu.config.pin_int = cfg.pin_imu_int; //IMU data ready interrupt pin
  imu.config.gizmo = (Cfg::imu_gizmo_enum)cfg.imu_gizmo; //the gizmo to use
  imu.config.spi_bus = hal_get_spi_bus(cfg.imu_spi_bus); //SPI bus
  imu.config.spi_cs = cfg.pin_imu_cs; //SPI select pin
  imu.config.i2c_bus = hal_get_i2c_bus(cfg.imu_i2c_bus); //I2C bus (only used if spi_bus==nullptr)
  imu.config.i2c_adr = cfg.imu_i2c_adr; //i2c address. 0=default address
  imu.config.uses_i2c = ((Cfg::imu_bus_type_enum)cfg.imu_bus_type == Cfg::imu_bus_type_enum::mf_I2C);
  imu.config.pin_clkin = cfg.pin_imu_clkin; //CLKIN pin for ICM-42866-P - only tested for RP2 targets

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
      // switch off LED to signal calibration
      led.enabled = true;
      led.off();
      led.enabled = false;
      
      //Calibrate for zero gyro readings, assuming vehicle not moving when powered up. Comment out to only use cfg values. (Use CLI to calibrate acc.)
      cli.calibrate_gyro();
    #endif
  }

  // LUA - Start Lua script /madflight.lua from SDCARD (when #define MF_LUA_ENABLE 1)
  lua.begin();

  // CLI - Command Line Interface
  cli.begin();

  // Enable LED, and switch it to green to signal end of startup.
  led.enabled = true;
  led.color(0x00ff00); //switch color to green
  led.on();
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
