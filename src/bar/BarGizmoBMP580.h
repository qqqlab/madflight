/*==========================================================================================
MIT License

Copyright (c) 2026 https://madflight.com

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

//Driver for BMP580 / BMP581 / BMP585 pressure sensor

#pragma once

#include "bar.h"
#include "../hal/MF_I2C.h"
#include "../tbx/common.h"

/*
NORMAL MODE
ODR(Hz), max OSR_P, max OSR_T
240 2 4
220 4 2
140 8 2
80 16 4
45 32 2
20 64 32
10 128 64

CONTINOUS MODE
OSR_P Pressure oversampling, OSR_T Temperature oversampling, Typical pressure RMS noise at 100kPa, Typical ODR in CONTINUOUS mode
Lowest power
×1 ×1 0.78 Pa 498 Hz 
×2 ×1 0.58 Pa 374 Hz
Standard resolution
×4 ×1 0.41 Pa 255 Hz
×8 ×1 0.30 Pa 155 Hz
High resolution
×16 ×1 0.21 Pa 87 Hz
×32 ×2 0.15 Pa 46 Hz
×64 ×4 0.11 Pa 24 Hz
Highest resolution
×128 ×8 0.08 Pa 12 Hz
*/

class BarGizmoBMP580 : public BarGizmo {
private:
  MF_I2CDevice *dev = nullptr;
  BarState *state = nullptr;

public:
    const char* name() override {return "BMP580";}

  ~BarGizmoBMP580() {
    delete dev;
  }

  static BarGizmoBMP580* create(BarConfig *config, BarState *state) {
    if(!config->i2c_bus) return nullptr;
    BarGizmoBMP580 *gizmo = new BarGizmoBMP580(config->i2c_bus, config->i2c_adr, state);
    if(!gizmo->begin()) {
      delete gizmo;
      return nullptr;
    }
    return gizmo;
  }

private:
  BarGizmoBMP580(MF_I2C *i2c, uint8_t i2c_adr, BarState *state) : state{state} {
    i2c->setClockMax(1000000);
    if(i2c_adr == 0) i2c_adr = 0x47; // fixed: 0x47 or 0x46
    dev = new MF_I2CDevice(i2c, i2c_adr);
  }

  bool begin() {
    // check who-am-i
    const uint8_t wai_exp = 0x50;
    uint8_t wai = dev->readReg(0x01);
    if( wai != wai_exp) {
      Serial.printf("BAR: WARNING: BMP580 got incorrect WAI 0x%02X, expected 0x%02X\n", wai, wai_exp);
    }

    // software reset
    dev->writeReg(0x7E, 0xB6); //reset
    delay(4); //datasheet: 2 ms    
    
    // interrupt
    dev->writeReg(0x15, 0x00);       // INT_SOURCE: off
    dev->readReg(0x27);              // INT_STATUS: clear status
    dev->writeReg(0x14, 0b00111010); // INT_CONFIG: b7-4:0011=default drive strength, b3:1=enabled, b2:0=push-pull, b1:1=active high, b0:0=pulsed
    dev->writeReg(0x15, 0b00000001); // INT_SOURCE: b0:drdy_data_reg_en

    // 87Hz continuous mode
    dev->writeReg(0x36, 0b01100000); // OSR_CONFIG b7:0=res, b6:1=pres enable, b5-3:OSR_P 100=16x, b2-0:OSR_T 000=1x
    dev->writeReg(0x37, 0b00000011); // ODR_CONFIG b7:1=deep standby disable, b6-2:odr, b1-0:11=continuous mode

    // test sensor
    uint32_t ts = micros();
    while(micros() - ts < 50000) {
      if(data_ready()) return true;
    }
    Serial.println("\nBAR: ERROR: BMP580 no samples received, disabling sensor\n\n");
    return false;
  }

  bool data_ready() {
    return dev->readReg(0x27) & 0x01; // INT_STATUS b0:data_ready
  }

public:
  bool update(float *press, float *temp) override {
    // exit if far away from next sample time (don't read i2c status if we know the answer already)
    if(micros() - state->ts < 10000) return false; //10ms - sample rate is 87Hz, 11.7ms, 

    // exit if no new data
    if(!data_ready()) return false;

    // get new data
    uint8_t d[6];
    if(dev->readReg(0x1D, d, sizeof(d)) != 6) return false; //0x1D = TEMP_DATA_XLSB
    int32_t t = d[0] | (d[1]<<8) | (d[2]<<16);
    if(d[2]&0x80) t |= 0xFF000000;
    float t_new = ((float)t) / 65536;
    int32_t p = d[3] | (d[4]<<8) | (d[5]<<16);
    float p_new = ((float)p) / 64;

    // spike filter
    static float p_last = 0;
    bool updated = fabs(p_new - p_last) < 50; //50Pa => 4m/12ms = 333m/s -> should not move vertically with speed of sound.
    if(updated) {
      *temp = t_new;
      *press = p_new;
    }
    p_last = p_new;

    return updated;
  }
};
