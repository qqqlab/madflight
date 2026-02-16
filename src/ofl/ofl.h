/*==========================================================================================
MIT License

Copyright (c) 2025 https://madflight.com

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
#include "../tbx/MsgBroker.h"

struct __attribute__((aligned(4))) OflState {
  public:
    // Sensor state vars
    int dx_raw = 0;  // raw sensor reading in [pixels] (x-axis of sensor frame)
    int dy_raw = 0;  // raw sensor reading in [pixels] (y-axis of sensor frame)

    // Ofl state vars
    float dx = 0;      // movement in N direction of vehicle NED frame in [radians] (dx_raw,dy_raw rotated by ofl_align, multiplied by ofl_cal_rad)
    float dy = 0;      // movement in E direction of vehicle NED frame in [radians] (dx_raw,dy_raw rotated by ofl_align, multiplied by ofl_cal_rad)
    float x = 0;       // position in N direction of vehicle NED frame in [radians]
    float y = 0;       // position in E direction of vehicle NED frame in [radians]
    uint32_t update_ts = 0; // update timestamp [us]
    uint32_t update_cnt = 0; // number of updates since start
};

struct OflConfig {
  public:
    Cfg::ofl_gizmo_enum ofl_gizmo = Cfg::ofl_gizmo_enum::mf_NONE;
    int ofl_spi_bus = -1; //SPI bus id
    int pin_ofl_cs = -1; //SPI cs pin
    int ofl_ser_bus = -1; //Serial bus id
    int ofl_baud = 0; //baud rate. 0=autobaud
    Cfg::ofl_align_enum ofl_align = Cfg::ofl_align_enum::mf_NE; //orientation of mounted sensor in relation to vehicle NED frame
    float ofl_cal_rad = 0; // manual calibration factor from pixels to radians, leave at 0 to use calibration from gizmo (updated by gizmo when 0)
};

class OflGizmo {
  public:
    virtual ~OflGizmo() {}
    virtual bool update() = 0; //returns true if new sample was taken
};

class Ofl : public OflState {
  public:
    OflConfig config;
    OflGizmo *gizmo = nullptr;
    MsgTopic<OflState> topic = MsgTopic<OflState>("ofl");

    int setup();      // Use config to setup gizmo, returns 0 on success, or error code
    bool installed() {return (gizmo != nullptr); } // Returns true if a gizmo was setup

  protected:
    friend void sensor_task(void *pvParameters);
    bool update();    // Returns true if state was updated

  private:
    RuntimeTrace runtimeTrace = RuntimeTrace("OFL");
};

//Global module instance
extern Ofl ofl;
