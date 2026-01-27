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

#include "mag.h"
#include "../hal/MF_I2C.h"
#include "RM3100/RM3100.h"

class MagGizmoRM3100: public MagGizmo {
protected:
  MagGizmoRM3100() {}; //protected constructor
  RM3100 *sensor = nullptr;

public:
  const char* name() override {return "RM3100";}
  static MagGizmoRM3100* create(MF_I2C *i2c) {
    uint8_t probe_adr = RM3100::probe(i2c);
    if(probe_adr == 0) {
      Serial.printf("MAG: ERROR: RM3100 not detected\n");
      return nullptr;
    }

    //create and configure gizmo
    auto gizmo = new MagGizmoRM3100();
    uint16_t cycle_count = 294; //cycle_count=294 gives approx 100Hz update rate
    gizmo->sensor = new RM3100(i2c, probe_adr, cycle_count);
    Serial.printf("MAG: RM3100 detected - i2c_adr:0x%02X cycle_count:%d resolution:%d nT/LSB\n", probe_adr, cycle_count, (int)(gizmo->sensor->scale_uT*1000));
    return gizmo;
  }

  bool update(float *x, float *y, float *z) override {
    sensor->read(x,y,z);
    return true;
  }

  ~MagGizmoRM3100() {
    delete sensor;
  }
};