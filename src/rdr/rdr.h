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
#include "../tbx/RuntimeTrace.h"
#include "../tbx/msg.h"

struct RdrState {
  public:
    // Sensor state vars
    float dist = -1; //distance in m, or negative error code (-1: no data, -2: invalid data)

    // Rdr state vars
    uint32_t update_ts = 0; //timestamp last successful update() call
    uint32_t update_cnt = 0; //number of updates since start
};

struct RdrConfig {
  public:
    Cfg::rdr_gizmo_enum gizmo = Cfg::rdr_gizmo_enum::mf_NONE;
    int rdr_ser_bus = -1; //Serial bus id
    int rdr_baud = 0; //baud rate. 0=autobaud
    int pin_rdr_trig = -1; //trigger pulse output pin
    int pin_rdr_echo = -1; //echo pulse input pin
    int rdr_i2c_bus = -1; //I2C bus id
    int rdr_i2c_adr = 0; //I2C address
};

class RdrGizmo {
  public:
    virtual ~RdrGizmo() {}
    virtual bool update() = 0; //returns true if new sample was taken
};

class Rdr : public RdrState {
  public:
    RdrConfig config;
    RdrGizmo *gizmo = nullptr;
    MsgTopic<RdrState> topic = MsgTopic<RdrState>("rdr");

    int setup();      // Use config to setup gizmo, returns 0 on success, or error code
    bool installed() {return (gizmo != nullptr); } // Returns true if a gizmo was setup

  protected:
    friend void sensor_task(void *pvParameters);
    bool update();    // Returns true if state was updated

  private:
    RuntimeTrace runtimeTrace = RuntimeTrace("RDR");
};

//Global module instance
extern Rdr rdr;
