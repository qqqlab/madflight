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
===========================================================================================*/

#pragma once

#include "mag.h"
#include "../imu/imu.h"
#include "../tbx/common.h"

class MagGizmoIMU : public MagGizmo {
private:
  float n = 0;
  float w = 0;
  float u = 0;
  uint32_t ts = 0;
  uint32_t last_ts = 0;
public:

  bool push_nwu_from_imu(float n, float w, float u) {
    if(n != this->n || w != this->w || u != this->u ) {
      this->n = n;
      this->w = w;
      this->u = u;
      ts = micros();
      return true;
    }
    return false;
  }

  const char* name() override {return "IMU";}

  MagGizmoIMU() {}

  bool update_nwu(float *n, float *w, float *u) override {
    if(last_ts == ts) return false;
    *n = this->n;
    *w = this->w;
    *u = this->u;
    last_ts = ts;
    return true;
  }
};
