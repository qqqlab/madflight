/*==========================================================================================
alt_comp.h - madflight altitude estimator complementary filter

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
#include <math.h>
#include <stdlib.h>
#include <string.h>



class AltEst_Comp: public AltEst {
public:
  void setup(float alt) {\
    setup2(alt, 0.4, 0.03, 0.1, 12); //baroAltitude[m], sigmaBaro[m], sigmaAccel[m/s2], accelThreshold[m/s2], ZUPT_SIZE[n]);
    ts = 0;

    //Serial.printf("ALT: COMP gain=%f,%f\n", gain0, gain1);
  }

  //a: accel up in [m/s^2], ts: timestamp in [us]
  void updateAccelUp(float a, uint32_t ts) {
    alt_a += a;
    alt_acnt++;
  };
  
  //altitude: barometric altitude in [m], ts: timestamp in [us]
  void updateBarAlt(float alt, uint32_t ts) {
    if(this->ts != 0) {
      float dt = 1e-6 * (ts - this->ts);
      a = alt_a/alt_acnt;
      estimate(alt, a, dt);
      alt_a = 0;
      alt_acnt = 0;

    }
    this->ts = ts;
  }
  
  float getH() {return h;} //altitude estimate in [m]
  float getV() {return v;} //vertical up speed (climb rate) estimate in [m/s]

  void toString(char *s) {
    int n = 0;
    n += sprintf(s+n, "alt.h:%.2f\t", h);
    n += sprintf(s+n, "alt.v:%+.2f\t", v);
    n += sprintf(s+n, "alt.a:%+.2f\t", a);
  }

  void setup2(float barAltitude, float sigmaAccel, float sigmaBaro, float accelThreshold, int ZUPT_SIZE)
  {
    // Compute the filter gain
    gain0 = sqrt(2 * sigmaAccel / sigmaBaro);
    gain1 = sigmaAccel / sigmaBaro;
    // If acceleration is below the threshold the ZUPT counter will be increased
    this->accelThreshold = accelThreshold;
    // initialize zero-velocity update
    this->ZUPT_SIZE = ZUPT_SIZE;
    ZUPTIdx = 0;
    pastAltitude = barAltitude;
    h = barAltitude;
    v = 0;
  }

  void estimate(float barAltitude, float a, float dt)
  {
    // Apply complementary filter
    h += (v + (gain0 + gain1*dt/2) * (baroAltitude - pastAltitude)) * dt + a*dt*dt/2;
    v += (gain1 * (barAltitude - pastAltitude) + a) * dt;
    pastAltitude = barAltitude;
    
    // zero-velocity update if more than ZUPT_SIZE small acc were received
    if(fabs(a) >= accelThreshold) {
      ZUPTIdx = 0;
    }else if(ZUPTIdx < ZUPT_SIZE) {
      ZUPTIdx++;
    }else{
      v = 0;
    }
  }

protected:
  float h;
  float v;
  float pastAltitude;
  // filter gain
  float gain0, gain1;
  // Zero-velocity update
  float accelThreshold = 0.1;
  int ZUPT_SIZE = 12;
  int ZUPTIdx;

  uint32_t ts = 0;
  float alt_a = 0;
  int alt_acnt = 0;
  float a = 0;
};
