/*==========================================================================================
alt_kalman2.h - madflight altitude estimator interface

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

#include "altitude_kf.h"

class AltEst_Kalman2 : public AltEst {
public:
  void setup(float alt) {\
    //default parameters
    float altCov = 0.2;  //meadured stdev BME280 = 0.4 [m]
    float accCov = 0.01; //measured stdev MPU6500 @ 16G = 0.003 [G]

    Serial.printf("ALT: KALMAN2  altCov=%f accCov=%f\n", altCov, accCov);

    filter.setup(altCov, accCov);
    filter.h = alt;
    filter.v = 0;

    tsa = 0;
  }

  //a: accel up in [m/s^2], ts: timestamp in [us]
  void updateAccelUp(float a, uint32_t ts) {
    if(tsa!=0) {
      float dt = 1e-6 * (ts - tsa);
      filter.propagate(a, dt);
    }
    tsa = ts;
  };
  
  //altitude: barometric altitude in [m], ts: timestamp in [us]
  void updateBarAlt(float alt, uint32_t ts) {
    filter.update(alt);
  }
  
  float getH() {return filter.h;} //altitude estimate in [m]
  float getV() {return filter.v;} //vertical up speed (climb rate) estimate in [m/s]

  void toString(char *s) {
    int n = 0;
    n += sprintf(s+n, "alt.h:%.2f\t", filter.h);
    n += sprintf(s+n, "alt.v:%+.2f\t", filter.v);
  }

protected:
  Altitude_KF filter;
  uint32_t tsa = 0;
};