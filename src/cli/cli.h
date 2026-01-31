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

#pragma once

#include <Arduino.h> //String
#include "../rcl/RclGizmoMavlink.h"
#include "../tbx/RuntimeTrace.h"

//cli command extension, return true if command was processed
extern bool cli_execute(String cmd, String arg1, String arg2) __attribute__((weak));

/* example cli command extension
bool cli_execute(String cmd, String arg1, String arg2) {
  if(cmd == "mycommand") {
    Serial.println("ASDFASDFASDF");
    return true;
  }
  return false;
}
*/

class Cli {
public:
  uint32_t updated_cnt = 0;
  void setup();
  void begin();
  void help();
  void ps();

protected:
  friend void cli_task(void *pvParameters);
  bool update(); //returns true if a command was processed (even an invalid one)

private:
  RuntimeTrace runtimeTrace = RuntimeTrace("CLI");

//========================================================================================================================//
//                                          COMMAND PROCESSING                                                            //
//========================================================================================================================//

public:
  enum cli_mode_enum {
    MODE_CLI,
    MODE_MAV,
    MODE_MSP
  };

private:
  RclGizmoMavlink* mavlink = nullptr;
  cli_mode_enum cli_mode = MODE_CLI;
  bool update_MODE_CLI();
  bool update_MODE_MSP();
  bool update_MODE_MAV();

private:
  String cmdline = "";
  char prev_c = 0;

public:
  void cmd_execute_batch(const char *batch);
  void cmd_clear();

private:
  bool cmd_process_char(char c);
  String getCmdPart(uint32_t &pos);
  void processCmd();

public:
  void executeCmd(String cmd, String arg1 = "", String arg2 = "");

//========================================================================================================================//
//                                          HELPERS                                                                       //
//========================================================================================================================//

public:
  void print_i2cScan();

//========================================================================================================================//
//                                          CALIBRATION FUNCTIONS                                                         //
//========================================================================================================================//

public:
  void calibrate_gyro();
  void calibrate_IMU();
  void calibrate_IMU2(bool gyro_only = false);
  void calibrate_Magnetometer();
private:
  bool _calibrate_Magnetometer(float bias[3], float scale[3]);
  void calibrate_info(int seconds = 0);

//========================================================================================================================//
//                                                PRINT FUNCTIONS                                                         //
//========================================================================================================================//
public:
  //add custom print command, returns true if added
  bool add_print_command(const char *cmd, const char *info, void (*function)(void));

private:
  uint32_t cli_print_time = 0;

  void cli_print_all(bool val);
  void cli_print_loop();
};

extern Cli cli;
