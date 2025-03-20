#pragma once

#include "mag.h"
#include "../hal/MF_I2C.h"
#include "QMC5883L/QMC5883L.h"

class MagGizmoQMC5883L: public MagGizmo {
  protected:
    QMC5883L mag_QMC5883L;

  public:
    MagGizmoQMC5883L(MF_I2C *i2c, int8_t i2c_adr) {
      mag_QMC5883L.begin(i2c, i2c_adr);
      Serial.printf("MAG: QMC5883L detect=%d\n", mag_QMC5883L.detect());
      //return (mag_QMC5883L.detect() ? 0 : 1);
    }

    bool read_uT(float *x, float *y, float *z) override {
      mag_QMC5883L.read_uT(x, y, z);
      return true; 
    }
};
