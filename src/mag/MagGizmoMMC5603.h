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
#include "../tbx/common.h"

class MagGizmoMMC5603 : public MagGizmo {
private:
  MF_I2CDevice *dev = nullptr;

public:
    const char* name() override {return "MMC5603";}
    const float scale_uT = 0.00625; //scale factor to uT
    int32_t mx; //raw adc values
    int32_t my;
    int32_t mz;

  ~MagGizmoMMC5603() {
    delete dev;
  }

  static MagGizmoMMC5603* create(MagConfig *config, MagState *state) {
    if(!config->i2c_bus) return nullptr;
    MagGizmoMMC5603 *gizmo = new MagGizmoMMC5603(config->i2c_bus);
    if(!gizmo->begin()) {
      delete gizmo;
      return nullptr;
    }
    return gizmo;
  }

private:
  MagGizmoMMC5603(MF_I2C *i2c) {
    i2c->setClockMax(400000);
    dev = new MF_I2CDevice(i2c, 0x30); //i2c address is always 0x30
  }

  bool begin() {
    //check who-am-i
    uint8_t wai = dev->readReg(0x39);
    if( wai != 0x10) {
      Serial.printf("MAG: WARNING: MMC5603 got incorrect WAI 0x%02X, expected 0x10\n");
    }

    /* CONTINUOUS MODE
    1. set ODR[7:0] to non-zero
    2. Then Cmm_freq_en is set to 1 to let the internal circuitry to calculate the target number for the counter
    3. After that Cmm_en is set to 1 and the continuous mode is started and the internal counter starts to count at the same time.
    Note: odr and control regs are write-only
    */
    
    //software reset
    dev->writeReg(0x1C, 0x80); //control1: Sw_reset=0x80
    delay(30); //datasheet: 20 ms

    //configure 75Hz with auto set/reset every 100 samples
    dev->writeReg(0x1C, 0x00); //control1: bw=0x00 (6.6ms)
    dev->writeReg(0x1A, 75);   //odr: 75Hz @ bw=0x00
    dev->writeReg(0x1B, 0xA0); //control0: Cmm_freq_en=0x80 calc freq, Auto_SR_en=0x20 auto set/reset, 
    dev->writeReg(0x1D, 0x1B); //control2: Cmm_en=0x10 continuous mode, En_prd_set=0x08 auto set/reset, Prd_set[2:0]=3 set/reset every 100 samples

    //test sensor
    uint32_t ts = micros();
    while(micros() - ts < 20000) {
      if(update_raw()) return true;
    }

    Serial.println("\nMAG: ERROR: MMC5603 no samples received, disabling magnetometer\n\n");
    return false;
  }

public:
  bool update_raw() {
    if((dev->readReg(0x18) & 0x40) == 0x00) return false; //exit if no new mag data

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
