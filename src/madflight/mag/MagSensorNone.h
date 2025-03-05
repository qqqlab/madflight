#pragma once

#include "mag_interface.h"

class MagSensorNone: public MagSensor {
  public:
    int setup(MF_I2C *i2c, int8_t i2c_adr, uint32_t sampleRate) override { (void)i2c; (void)i2c_adr, (void)sampleRate; return 0; }
    bool read_uT(float *x, float *y, float *z) override { (void)x; (void)y; (void)z; return 0; }
};
