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

#include "../cfg/cfg.h"
#include "Binlog.h"
#include <SPI.h>

struct BbxConfig {
  public:
    Cfg::bbx_gizmo_enum gizmo = Cfg::bbx_gizmo_enum::mf_NONE; //the gizmo to use
    SPIClass *spi_bus = nullptr; //spi bus
    uint8_t i2c_adr = 0; //i2c address. 0=default address
    int32_t spi_cs = -1;
    int32_t pin_mmc_dat = -1;
    int32_t pin_mmc_clk = -1;
    int32_t pin_mmc_cmd = -1;
};

class BbxGizmo {
public:
  virtual ~BbxGizmo() {}
  virtual void setup() = 0; //setup the file system (can be called multiple times)
  
  virtual bool writeOpen() = 0; //create new file for writing (closes previously opened file first)
  virtual void write(const uint8_t *buf, const uint8_t len) = 0; //write to file
  virtual void close() = 0; //close file

  virtual void erase() = 0; //erase all files
  virtual void dir() = 0; //list files
  virtual void bench() = 0; //benchmark read/write
  virtual void info() = 0; //card info
};

class Bbx {
  private:
    int gizmo_create();

  public:
    BbxConfig config;

    BbxGizmo *gizmo = nullptr;

  public:
    //Blackbox Interface
    int setup(); //setup blackbox
    void start(); //start logging (create new file)
    void stop();  //stop logging (closes file)
    void erase(); //erase all log files
    void dir();   //list log files
    void bench(); //benchmark read/write to blackbox
    void info();  //blackbox info (memory size, free space, etc.)

    //loggers
    void log_bar();
    void log_bat();
    void log_gps();
    void log_imu();
    void log_mode(uint8_t fm, const char* name);
    void log_msg(const char* msg);
    void log_parm(const char* name, float value, float default_value);
    void log_pid();
    void log_att();
    void log_ahrs();
    void log_sys();


};

extern Bbx bbx;
