#pragma once

#include "mag.h"
#include "../hal/MF_I2C.h"


class MagGizmoQMC6309 : public MagGizmo {
private:
  MF_I2CDevice *dev;
public:
  MagGizmoQMC6309(MF_I2C *i2c, int8_t i2c_adr) {
    i2c_adr = 0x7C; // fixed: 0x7C
    this->dev = new MF_I2CDevice(i2c, i2c_adr);

    //setup for 16 sample moving average (my interpretation of data sheet OSR2 setting), sample rate = 1500Hz (continous mode)
    dev->write(0x0B, 0x04); //ODR=1Hz, Scale=8G, Reset
    dev->write(0x0A, 0xFD); //OSR2(filter)=16, OSR=1, Continuous Mode
  }

  bool update(float *x, float *y, float *z) override {
    uint8_t d[6];
    dev->read(0x01, d, 6);
    int16_t mx = d[0] | (d[1] << 8); //16 bit litte-endian signed
    int16_t my = d[2] | (d[3] << 8);
    int16_t mz = d[4] | (d[5] << 8);

    *x = 200e-9 * mx; //in [T]
    *y = 200e-9 * my;
    *z = 200e-9 * mz;
    return true;
  }
};
