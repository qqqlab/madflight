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

#include "../../hal/MF_I2C.h"

class RM3100 {
  public:
    float scale_uT = 1;

    //probe for RM3100, returns i2c address on success, or 0 on fail
    static uint8_t probe(MF_I2C *i2c);

    RM3100(MF_I2C *i2c, uint8_t i2c_adr, uint16_t cycle_count=200); 

    ~RM3100();

    bool dataReady();

    //returns xyz[3] with counts
    void readRaw(int32_t *xyz);
    
    //returns x,y,z as uT values
    void read(float *x, float *y, float *z);

  protected:
    MF_I2CDevice *dev;
};