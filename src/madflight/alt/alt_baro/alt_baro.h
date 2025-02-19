/*==========================================================================================
alt_baro.h - madflight altitude estimator based on filtered barometer readings

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

#include "../alt_interface.h" //AltEst
#include "../../baro/baro_interface.h"
#include "../../common/common.h" //lowpass_to_beta()

class AltEst_Baro : public AltEst {
public:
  void setup(float alt) {
    Serial.printf("ALT:  ALT_USE_BARO\n");

    float sampleRate = baro.getSampleRate();
    float filterHHertz = 2.0;
    float filterVHertz = 0.5;
    
    B_h = lowpass_to_beta(filterHHertz, sampleRate);
    B_v = lowpass_to_beta(filterVHertz, sampleRate);
    h = alt;
    v = 0;
    ts = 0;
  }

  void updateAccelUp(float a, uint32_t ts) {}; //a: accel up in [m/s^2], ts: timestamp in [us]
  
  void updateBaroAlt(float alt, uint32_t ts) { //altitude: barometric altitude in [m], ts: timestamp in [us]
    if(this->ts != 0) {
      float dt = 1e-6 * (ts - this->ts);
      float hnew = h + B_h * (alt - h); //Low-pass filtered altitude
      float vnew = (hnew - h) / dt;
      v += B_v * (vnew - v); //Low-pass filtered velocity
      h = hnew;
    }
    this->ts = ts;
  }
  
  float getH() {return h;} //altitude estimate in [m]
  float getV() {return v;} //vertical up speed (climb rate) estimate in [m/s]

  void toString(char *s) {
    int n = 0;
    n += sprintf(s+n, "alt.h:%.2f\t", h);
    n += sprintf(s+n, "alt.v:%+.2f\t", v);
  }
  
  float h = 0;    // Filtered approximate International Standard Atmosphere (ISA) Altitude in [m]
  float v = 0;    // Filtered vertical speed in [m/s], up is positive

protected:
  float B_h = 1.0; //alt filter constant
  float B_v = 1.0; //vz filter constant
  uint32_t ts = 0;
};
