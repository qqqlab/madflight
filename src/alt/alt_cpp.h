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

/*=================================================================================================
Each ALT_USE_xxx section in this file defines a specific altimeter estimator class
=================================================================================================*/

// Make sure this file is included from madflight.h and not from somewhere else
#ifndef MF_ALLOW_INCLUDE_CCP_H
  #error "Only include this file from madflight.h"
#endif
//#pragma once //don't use here, we want to get an error if included twice

#define ALT_USE_NONE    0
#define ALT_USE_KALMAN2 1 // Kalman filter estimates h and v from barometer and acceleration
#define ALT_USE_KALMAN3 2 // Kalman filter estimates h, v, and abias from barometer and acceleration
#define ALT_USE_BAR     3 // Filtered barometer
#define ALT_USE_COMP    4 // Complementary filter

#include "alt.h" //declares AltEst - Altimeter Estimator base class

//default for ALT (Altitude Estimation)
#ifndef ALT_USE
  #define ALT_USE ALT_USE_BARO
#endif

//=================================================================================================
// None or undefined
//=================================================================================================
#if ALT_USE == ALT_USE_NONE

class AltEst_None : public AltEst {
public:
  void setup(float alt) {
    (void)alt;
    Serial.printf("ALT: NONE\n");
  }
  void updateAccelUp(float a, uint32_t ts) {(void)a;(void)ts;};
  void updateBarAlt(float alt, uint32_t ts) {(void)alt;(void)ts;} 
  float getH() {return 0;}
  float getV() {return 0;}
  void toString(char *s) {if(s) s[0] = 0;}
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
// ALT_USE_BAR - Filtered barometer only
//=================================================================================================
#elif ALT_USE == ALT_USE_BAR

#include "alt_bar/alt_bar.h"

AltEst_Bar alt_instance;


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
