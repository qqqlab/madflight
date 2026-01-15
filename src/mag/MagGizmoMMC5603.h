/*==========================================================================================
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

//Datasheet:  https://www.memsic.com/Public/Uploads/uploadfile/files/20220119/MMC5603NJDatasheetRev.B.pdf

#pragma once

#include "mag.h"
#include "../hal/MF_I2C.h"

class MagGizmoMMC5603 : public MagGizmo {
private:
  MF_I2CDevice *dev = nullptr;

public:
  const float scale_uT = 0.00625; //scale factor to uT
    int32_t mx; //raw adc values
    int32_t my;
    int32_t mz;

  ~MagGizmoMMC5603() {
    delete dev;
  }

  MagGizmoMMC5603(MF_I2C *i2c) {
    i2c->setClock(400000);

    this->dev = new MF_I2CDevice(i2c, 0x30); //i2c address is always 0x30

    //check who-am-i
    uint8_t wai = dev->readReg(0x39);
    if( wai != 0x10) {
      Serial.printf("\nMAG: WARNING: MMC5603 got incorrect WAI 0x%02X, expected 0x10\n\n");
    }

    //configure 75Hz with auto set/reset every 100 samples
    dev->writeReg(0x1A, 75); //odr: 75Hz @ bw00 with auto sr
    dev->writeReg(0x1B, 0xA0); //control0: Cmm_freq_en=80 calc freq, Auto_SR_en=20 auto set/reset, 
    dev->writeReg(0x1C, 0x00); //control1: bw00 (6.6ms)
    dev->writeReg(0x1D, 0x1B); //control2: Cmm_en=10 continuous mode, En_prd_set=8 auto set/reset, Prd_set[2:0]=3 set/reset every 100 samples
  }

  bool update_raw() {
uint8_t wai = dev->readReg(0x39);
    //v2.2.2 disabled this - did not work...
    //if((dev->readReg(0x18) & 0x80) == 0x00) return false; //exit if no new data

    uint8_t d[9];
    dev->readReg(0x00, d, 9);
    mx = (d[0]<<12 | d[1]<<4 | d[6]>>4 ) - (1<<19);
    my = (d[2]<<12 | d[3]<<4 | d[7]>>4 ) - (1<<19);
    mz = (d[4]<<12 | d[5]<<4 | d[8]>>4 ) - (1<<19);
    return true;
  }

  bool update(float *x, float *y, float *z) override {
    if(!update_raw()) return false;

    *x =  scale_uT * mx; 
    *y =  scale_uT * my;
    *z =  scale_uT * mz;
    return true;
  }
};
