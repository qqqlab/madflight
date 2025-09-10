#pragma once

#include "bar.h"
#include "../hal/MF_I2C.h"
#include "dps3/src/Dps3xx.h"

class BarGizmoDPS310: public BarGizmo {
protected:
  Dps3xx pressureSensor;
public:
  BarGizmoDPS310(MF_I2C *i2c, int8_t i2c_adr, uint32_t sampleRate) {
    if (i2c_adr == 0) {
      i2c_adr = 0x77;
    }
    pressureSensor.begin(*i2c, i2c_adr);
  }

  bool update(float *press, float *temp) override {
    int16_t ret;

    ret = pressureSensor.measurePressureOnce(*press);
    if (ret != 0) {
      return false;
    }

    ret = pressureSensor.measureTempOnce(*temp);
    if (ret != 0) {
      return false;
    }

    return true;
  }
};
