#pragma once

#include "../common/MF_I2C.h"

class Battery {
  public:
    float i = 0; //Battery current (A)
    float v = 0; //battery voltage (V)
    float w = 0; //battery power (W)
    float mah = 0; //battery usage (mAh)
    float wh = 0; //battery usage (Wh)

    virtual void begin(MF_I2C *i2c, int8_t i2c_adr) = 0;
    virtual bool update() = 0; //returns true if battery was updated

    void setup() {
      begin(mf_i2c, 0);
    }
};

extern Battery &bat;
