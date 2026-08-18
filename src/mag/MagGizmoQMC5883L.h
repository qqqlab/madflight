#pragma once

#include "mag.h"
#include "../hal/MF_I2C.h"
#include "QMC5883L/QMC5883L.h"

class MagGizmoQMC5883L: public MagGizmo {
  protected:
    QMC5883L sensor;

  public:
    const char* name() override {return "QMC5883L";}
    MagGizmoQMC5883L(MF_I2C *i2c, int8_t i2c_adr) {
      sensor.begin(i2c, i2c_adr);
      Serial.printf("MAG: QMC5883L detect=%d\n", sensor.detect());
      //return (mag_QMC5883L.detect() ? 0 : 1);
    }

    bool update_nwu(float *n, float *w, float *u) override {
      sensor.read_uT(n, w, u);
      return true; 
    }
};
