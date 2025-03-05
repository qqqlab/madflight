#pragma once

#include "baro_interface.h"

class BaroSensorNone: public BaroSensor {
public:
  int setup(MF_I2C *i2c, int8_t i2c_adr, uint32_t sampleRate) override {
    (void) i2c;
    (void) i2c_adr;
    (void) sampleRate;
    Serial.println("BARO: BARO_USE_NONE");
    return 0;
  }

  //returns true if pressure was updated
  bool update(float *press, float *temp) override {
    (void) press;
    (void) temp;
    return false;
  }
};
