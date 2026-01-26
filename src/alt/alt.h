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

#ifndef ALT_USE
  #define ALT_USE ALT_USE_KALMAN3
#endif

#define ALT_USE_NONE    0
#define ALT_USE_KALMAN2 1 // Kalman filter estimates h and v from barometer and acceleration
#define ALT_USE_KALMAN3 2 // Kalman filter estimates h, v, and abias from barometer and acceleration
#define ALT_USE_BAR     3 // Filtered barometer
#define ALT_USE_COMP    4 // Complementary filter

//=================================================================================================
// ALT - Altitude Estimator
//=================================================================================================

class AltEst {
  public:
    virtual void setup(float alt) = 0; //setup with default parameters and initial altitude in [m]
    virtual void updateAccelUp(float a, uint32_t ts) = 0; //a: accel up in [m/s^2], ts: timestamp in [us]
    virtual void updateBarAlt(float alt, uint32_t ts) = 0; //alt: barometric altitude in [m], ts: timestamp in [us]
    virtual float getH() = 0; //altitude estimate in [m]
    virtual float getV() = 0; //vertical up speed (climb rate) estimate in [m/s]
    virtual void toString(char *s); //print state info to s (max 100 chars)
};

extern AltEst &alt;
