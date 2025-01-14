/*==========================================================================================
alt.h - madflight altitude estimator

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


/*=================================================================================================
Each ALT_USE_xxx section in this file defines a specific altimeter estimator class
=================================================================================================*/

#pragma once

#define ALT_USE_NONE    1
#define ALT_USE_KALMAN2 2 // Kalman filter estimates h and v from barometer and acceleration
#define ALT_USE_KALMAN3 3 // Kalman filter estimates h, v, and abias from barometer and acceleration
#define ALT_USE_BARO    4 // Filtered barometer
#define ALT_USE_COMP    5 // Complementary filter

#include "../interface.h" //declares AltEst - Altimeter Estimator class

//=================================================================================================
// None or undefined
//=================================================================================================
#if ALT_USE == ALT_USE_NONE || !defined ALT_USE

class AltEst_None : public AltEst {
public:
  void setup(float alt) {
    Serial.printf("ALT:  ALT_USE_NONE\n");
  }
  void updateAccelUp(float a, uint32_t ts) {};
  void updateBaroAlt(float alt, uint32_t ts) {} 
  float getH() {return 0;}
  float getV() {return 0;}
  void print() {}
};

AltEst_None alt_instance;

//=================================================================================================
// ALT_USE_KALMAN2 - Simple Kalman filter estimates h and vup from barometer and acceleration
//=================================================================================================
#elif ALT_USE == ALT_USE_KALMAN2

#include "alt_kalman2/alt_kalman2.h"

AltEst_Kalman2 alt_instance;


//=================================================================================================
// ALT_USE_KALMAN3 - Kalman filter estimates h, vup, and abias from barometer and acceleration
//=================================================================================================
#elif ALT_USE == ALT_USE_KALMAN3

#include "alt_kalman3/alt_kalman3.h"

AltEst_Kalman3 alt_instance;


//=================================================================================================
// ALT_USE_BARO - Filtered barometer only
//=================================================================================================
#elif ALT_USE == ALT_USE_BARO

#include "alt_baro/alt_baro.h"

AltEst_Baro alt_instance;


//=================================================================================================
// ALT_USE_COMP - Complementary filter altitude and accelation
//=================================================================================================
#elif ALT_USE == ALT_USE_COMP

#include "alt_comp/alt_comp.h"

AltEst_Comp alt_instance;


//=================================================================================================
// ERROR OTHER VALUE 
//=================================================================================================
#else
  #error "Invalid #define ALT_USE value"
#endif

AltEst &alt = alt_instance;