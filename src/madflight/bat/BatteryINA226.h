#pragma once

#include "bat_interface.h"
#include "../common/MF_I2C.h"
#include "INA226/INA226.h"

class BatteryINA226: public Battery {
protected:
  INA226 bat_ina226;

public:
  void begin(MF_I2C *i2c, int8_t i2c_adr) override {
    if(i2c_adr == 0) i2c_adr = 0x40; //default address
    bat_ina226.begin(i2c, i2c_adr);

    // Configure INA226 -> sample time = 2 * 128 * 140us = 36ms => 28Hz
    bat_ina226.configure(INA226_AVERAGES_128, INA226_BUS_CONV_TIME_140US, INA226_SHUNT_CONV_TIME_140US, INA226_MODE_SHUNT_BUS_CONT);

    float Rshunt = cfg.BAT_CAL_I; //ohm
    bat_ina226.calibrate(Rshunt);
  }

  bool update() override {
    static uint32_t ts = micros();
    if(!bat_ina226.isConversionReady()) return false;
    uint32_t now = micros();
    float dt_h = (now - ts) / 3600e6;
    ts = now;
    i = bat_ina226.readShuntCurrent();
    v = bat_ina226.readBusVoltage();
    //w = bat_ina226.readBusPower(); //note w is not always equal to v * i, because w, v, and i are averaged values
    w = v * i; //This appears to be more accurate, specially for low values.
    mah += i * dt_h * 1000;
    wh += w * dt_h;
    return true;
  }
};
