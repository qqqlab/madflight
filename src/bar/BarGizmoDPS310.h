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
      i2c_adr = 0x77;  // default i2c address for the DPS310
    }
    pressureSensor.begin(*i2c, i2c_adr);
    int16_t ret = pressureSensor.startMeasureBothCont(
      DPS__MEASUREMENT_RATE_128, DPS__OVERSAMPLING_RATE_2,
      DPS__MEASUREMENT_RATE_128, DPS__OVERSAMPLING_RATE_2
    );
    // TODO: find a way to make gizmo check fail (bar.cpp line 90)
    if (ret != 0) {
      Serial.println("bar init start measure cont failed");
      while (1);
    }
  }

  bool update(float *press, float *temp) override {
    uint8_t measureCount = 1;
    return pressureSensor.getContResults(temp, measureCount, press, measureCount) == 0;
  }
};
