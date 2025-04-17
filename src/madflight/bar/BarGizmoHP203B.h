#pragma once

#include "bar.h"
#include "../hal/MF_I2C.h"

class BarGizmoHP203B: public BarGizmo {
private:
  MF_I2CDevice *dev;
public:
  BarGizmoHP203B(MF_I2C *i2c, int8_t i2c_adr, uint32_t sampleRate) {
    (void) sampleRate; //TODO
    if(i2c_adr == 0) i2c_adr = 0x76; // fixed: 0x76 or 0x77
    this->dev = new MF_I2CDevice(i2c, i2c_adr);

    dev->writeReg(0x06, nullptr, 0); //SOFT_RST soft reset
    dev->writeReg(0x28, nullptr, 0); //ANA_CAL Re-calibrate the Internal analog Blocks
    dev->writeReg(0b01010000, nullptr, 0); //start conversion: ADC_CVT = 010, 100 OSR=256 (8.2ms), 00 press+temp
  }

  ~BarGizmoHP203B() {
    delete dev;
  }



/*
OSR  T[ms]  ADC_CVT
 128   4.1 0b01010100
 256   8.2 0b01010000
 512  16.4 0b01001100
1024  32.8 0b01001000
2048  65.6 0b01000100
4096 131.1 0b01000000 
*/
  bool update(float *press, float *temp) override {
    uint8_t d[6];
    dev->readReg(0x10, d, 6); //READ_PT pressure [Pa] + temp [0.01*C]
    //dev->readReg(0x11, d, 6); //READ_AT altitude [0.01*m] + temp [0.01*C]
    dev->writeReg(0b01010000, nullptr, 0); //start conversion: ADC_CVT = 010, 100 OSR=256 (8.2ms), 00 press+temp

    int32_t t = ((d[0] << 16) | (d[1] << 8) | d[2]) & 0x000fffff; //20 bit big-endian signed
    if(t & 0x00080000) t |= 0xfff00000; //20bit sign expansion
    uint32_t p = ((d[3] << 16) | (d[4] << 8) | d[5]) & 0x000fffff; //20 bit big-endian unsigned

    *temp = t * 0.01; //temperature in [C]
    *press = p; //pressure in [Pa]

    return true;
  }
};
