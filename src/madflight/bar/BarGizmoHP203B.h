#pragma once

#include "bar.h"
#include "../hal/MF_I2C.h"

class BarGizmoHP203B: public BarGizmo {
private:
  MF_I2CDevice *dev;
public:
  BarGizmoHP203B(MF_I2C *i2c, int8_t i2c_adr, uint32_t sampleRate) {
    (void) sampleRate; //TODO
    if(i2c_adr == 0) i2c_adr = 0XEC; // fixed: 0XEC or 0xEE
    this->dev = new MF_I2CDevice(i2c, i2c_adr);
    dev->writeReg(0b01010000, nullptr, 0); // ADC_CVT = 010, 100 OSR=256 (8.2ms), 00 press+temp
  }

  ~BarGizmoHP203B() {
    delete dev;
  }

  bool update(float *press, float *temp) override {
    uint8_t d[6];
    dev->readReg(0x10, d, 6); //READ_PT
    int32_t t = ((d[0] << 16) | (d[1] << 8) | d[2]) & 0x000fffff; //20 bit big-endian signed
    if(t & 0x00080000) t |= 0xfff00000; //20bit sign expansion
    uint32_t p = ((d[3] << 16) | (d[4] << 8) | d[5]) & 0x000fffff; //20 bit big-endian unsigned

    *temp = t * 0.01; //temperature in [C]
    *press = 0.01 * p; //pressure in [Pa]
    return true;
  }
};
