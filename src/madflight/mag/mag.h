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

struct MagState {
  public:
    float x = 0; //"North" magnetic flux [uT]
    float y = 0; //"East" magnetic flux [uT]
    float z = 0; //"Down" magnetic flux [uT]
    uint32_t ts = 0; //last sample time in [us]
};

struct MagConfig {
  public:
    uint32_t sampleRate = 100; //sample rate [Hz]
    Cfg::mag_gizmo_enum gizmo = Cfg::mag_gizmo_enum::mf_NONE; //the gizmo to use
    MF_I2C *i2c_bus = nullptr; //i2c bus
    uint8_t i2c_adr = 0; //i2c address. 0=default address
};

class MagGizmo {
public:
  virtual ~MagGizmo() {};
  virtual bool update(float *x, float *y, float *z) = 0; //returns true if new sample was retrieved
};

class Mag : public MagState {
  public:
    MagConfig config;

    MagGizmo *gizmo = nullptr;

    int setup();      // Use config to setup gizmo, returns 0 on success, or error code
    bool update();    // Returns true if state was updated
    bool installed() {return (gizmo != nullptr); } // Returns true if a gizmo was setup

  protected:
    uint32_t _samplePeriod = 0; //gizmo sample period in [us]

    bool _installed = false;
};

//Global module instance
extern Mag mag;
