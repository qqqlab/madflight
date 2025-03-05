#pragma once

#include "bat_interface.h"

class BatteryNone: public Battery {
  public:
  void begin(MF_I2C *i2c, int8_t i2c_adr) override { (void)i2c; (void)i2c_adr; }
  bool update() override { return false; }
};
