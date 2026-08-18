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

#include "../alt.h" //AltEst
#include "../../bar/bar.h"
#include "../../tbx/MF_Filter.h"

class AltEst_Bar : public AltEst {
public:
  void setup(float alt) {
    Serial.printf("ALT: BARO\n");

    float sample_rate = bar.config.sample_rate;
    float filterHHertz = 2.0;
    float filterVHertz = 2.0;
    
    MF_Filter::setup(filter_h, MF_FilterType::mf_PT1, sample_rate, filterHHertz, -1);
    MF_Filter::setup(filter_v, MF_FilterType::mf_PT1, sample_rate, filterVHertz, -1);
    
    h = alt;
    v = 0;
    ts = 0;
  }

  void updateAccelUp(float a, uint32_t ts) override { (void)a; (void)ts; }; //a: accel up in [m/s^2], ts: timestamp in [us]
  
  void updateBarAlt(float alt, uint32_t ts) override { //altitude: barometric altitude in [m], ts: timestamp in [us]
    if(this->ts != 0) {
      float dt = 1e-6 * (ts - this->ts);
      h = filter_h->apply(alt); //Low-pass filtered altitude
      if(dt > 0) v = filter_v->apply((alt - alt_prev) / dt); //Low-pass filtered velocity
      alt_prev = alt;
    }else{
      h = alt;
      v = 0;
      alt_prev = alt;
    }
    this->ts = ts;
  }
  
  float getH() override {return h;} //altitude estimate in [m]
  float getV() override {return v;} //vertical up speed (climb rate) estimate in [m/s]

  void toString(char *s) {
    if(s) s[0] = 0;
  }

  float h = 0;    // Filtered approximate International Standard Atmosphere (ISA) Altitude in [m]
  float v = 0;    // Filtered vertical speed in [m/s], up is positive

protected:
  MF_Filter *filter_h = nullptr;
  MF_Filter *filter_v = nullptr;
  uint32_t ts = 0;
  float alt_prev = 0;
};
