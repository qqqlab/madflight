#pragma once

#define MADFLIGHT_VERSION "madflight v2.2.4-DEV"

#include <Arduino.h>

// include all modules
#include "ahr/ahr.h" //AHRS
#include "alt/alt.h" //Altitude estimators
#include "cfg/cfg.h" //Config
#include "cli/cli.h" //Command Line Interface
#include "bar/bar.h" //Barometer sensor
#include "bat/bat.h" //Battery sensor
#include "bbx/bbx.h" //Blackbox SDCARD
#include "gps/gps.h" //GPS
#include "hal/hal.h" //HAL
#include "imu/imu.h" //IMU
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
