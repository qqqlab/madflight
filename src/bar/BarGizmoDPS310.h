#pragma once

#include "bar.h"
#include "../hal/MF_I2C.h"
#include "dps3/src/Dps3xx.h"

class BarGizmoDPS310: public BarGizmo {
protected:
  Dps3xx* pressureSensor;
  BarGizmoDPS310() {} // private constructor
public:
  static BarGizmoDPS310* create(MF_I2C *i2c, int8_t i2c_adr, uint32_t sampleRate) {
    if (i2c_adr == 0) {
      i2c_adr = 0x77;  // default i2c address for the DPS310
    }
    Dps3xx* pres = new Dps3xx();
    pres->begin(*i2c, i2c_adr);
    int16_t ret = pres->startMeasureBothCont(
      DPS__MEASUREMENT_RATE_128, DPS__OVERSAMPLING_RATE_64,
      DPS__MEASUREMENT_RATE_128, DPS__OVERSAMPLING_RATE_64
    );

    if (ret != 0) {
      Serial.println("DPS310 init failed");
      return nullptr;
    }
    
    auto gizmo = new BarGizmoDPS310();
    gizmo->pressureSensor = pres;
    return gizmo;
  }

  bool update(float *press, float *temp) override {
    uint8_t measureCount = 1;
    return pressureSensor->getContResults(temp, measureCount, press, measureCount) == 0;
  }
};
