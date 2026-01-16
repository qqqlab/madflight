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
#include "../cfg/cfg.h"

struct BatState {
  public:
    float i = 0;     // Battery current [A]
    float v = 0;     // Battery voltage [V]
    float w = 0;     // Battery power [W]
    float mah = 0;   // Battery usage [mAh]
    float wh = 0;    // Battery usage [Wh]
    uint32_t ts = 0; // Last update time stamp [us]
};

struct BatConfig {
  public:
    uint32_t sample_rate = 100; //sample rate [Hz]
    Cfg::bat_gizmo_enum gizmo = Cfg::bat_gizmo_enum::mf_NONE; //the gizmo to use
    MF_I2C *i2c_bus = nullptr; //i2c bus
    uint8_t i2c_adr = 0; //i2c address. 0=default address
    int32_t adc_pin_v = -1;
    int32_t adc_pin_i = -1;
    float adc_cal_v = 1.0; //voltage conversion factor: set to [Actual measured Volt] / [bat_v ADC reading when cal_v=1.0]
    float adc_cal_i = 1.0; //current conversion factor: set to [Actual measured Amperes] / [bat_i ADC reading when cal_i=1.0]
    float rshunt = 1.0; //shunt resistor value [Ohm]
};

class BatGizmo {
  public:
    virtual ~BatGizmo() {}
    virtual bool update() = 0; //returns true if new sample was taken
};

class Bat : public BatState {
  public:
    BatConfig config;

    BatGizmo *gizmo = nullptr;

    int setup();      // Use config to setup gizmo, returns 0 on success, or error code
    bool update();    // Returns true if state was updated
    bool installed() {return (gizmo != nullptr); } // Returns true if a gizmo was setup
};

//Global module instance
extern Bat bat;
