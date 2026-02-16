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

#include "../hal/MF_I2C.h"
#include "../hal/MF_Schedule.h"
#include "../cfg/cfg.h"
#include "../tbx/RuntimeTrace.h"
#include "../tbx/MsgBroker.h"

struct __attribute__((aligned(4))) BarState {
  public:
    float press = 0;  // Pressure in [Pascal]
    float alt = 0;    // Approximate International Standard Atmosphere (ISA) Altitude in [m]
    float temp = 0;   // Temperature in [Celcius]
    uint32_t ts = 0;  // Sample timestamp in [us]
    float dt = 0;     // Time since last sample in [seconds]
    float ground_level; // Ground level in [m]
};

struct BarConfig {
  public:
    uint32_t sample_rate = 100; //requested sample rate [Hz]
    Cfg::bar_gizmo_enum gizmo = Cfg::bar_gizmo_enum::mf_NONE; //the gizmo to use
    MF_I2C *i2c_bus = nullptr; //i2c bus
    uint8_t i2c_adr = 0; //i2c address. 0=default address
};

class BarGizmo {
public:
  virtual ~BarGizmo() {}
  virtual const char* name() = 0;
  virtual bool update(float *press, float *temp) = 0; //returns true if pressure was updated
};

class Bar : public BarState {
  public:
    BarConfig config;
    BarGizmo *gizmo = nullptr;
    MsgTopic<BarState> topic = MsgTopic<BarState>("bar");

    int setup();      // Use config to setup gizmo, returns 0 on success, or error code
    bool installed() {return (gizmo != nullptr); } // Returns true if a gizmo was setup
    const char* name() {return (gizmo ? gizmo->name() : "NONE");}

  protected:
    friend void sensor_task(void *pvParameters);
    bool update();    // Returns true if state was updated

  private:
    RuntimeTrace runtimeTrace = RuntimeTrace("BAR");
};

//Global module instance
extern Bar bar;
