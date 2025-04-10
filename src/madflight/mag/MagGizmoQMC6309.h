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

class MagGizmoQMC6309 : public MagGizmo {
private:
  MF_I2CDevice *dev = nullptr;

public:
  MagGizmoQMC6309(MF_I2C *i2c, int8_t i2c_adr) {
    i2c_adr = 0x7C; // fixed: 0x7C
    this->dev = new MF_I2CDevice(i2c, i2c_adr);

    //setup for 16 sample moving average (my interpretation of data sheet OSR2 setting), sample rate = 1500Hz (continous mode)
    dev->writeReg(0x0B, 0x04); //ODR=1Hz, Scale=8G, Reset
    dev->writeReg(0x0A, 0xFD); //OSR2(filter)=16, OSR=1, Continuous Mode
  }


  ~MagGizmoQMC6309() {
    delete dev;
  }


  bool update(float *x, float *y, float *z) override {
    uint8_t d[6];
    dev->readReg(0x01, d, 6);
    int16_t mx = d[0] | (d[1] << 8); //16 bit litte-endian signed
    int16_t my = d[2] | (d[3] << 8);
    int16_t mz = d[4] | (d[5] << 8);

    *x = 200e-9 * mx; //in [T]
    *y = 200e-9 * my;
    *z = 200e-9 * mz;
    return true;
  }
};
