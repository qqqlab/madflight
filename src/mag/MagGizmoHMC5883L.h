/*==========================================================================================
MIT License

Copyright (c) 2026 https://madflight.com

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
==========================================================================================*/

#pragma once

#include "mag.h"
#include "../hal/MF_I2C.h"
#include "HMC5883L/HMC5883L.h"

// Magnetometer gizmo for the Honeywell HMC5883L (GY-87 10DOF board)
class MagGizmoHMC5883L: public MagGizmo {
  protected:
    HMC5883L sensor;

  public:
    const char* name() override {return "HMC5883L";}

    MagGizmoHMC5883L(MF_I2C *i2c, uint8_t i2c_adr) {
      sensor.begin(i2c, i2c_adr);
      Serial.printf("MAG: HMC5883L i2c_adr=0x%02X detect=%d chip_id=0x%02X\n",
        sensor.getAdr(), sensor.detect(), sensor.chipIdA());
    }

    bool update_nwu(float *n, float *w, float *u) override {
      //always write the output, also when there is no new measurement, so that the
      //caller never uses uninitialized values
      return sensor.read_uT(n, w, u);
    }
};
