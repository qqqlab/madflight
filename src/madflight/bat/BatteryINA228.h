#pragma once

#include "bat_interface.h"
#include "../common/MF_I2C.h"
#include "INA228/INA228.h"

class BatteryINA228: public Battery {
protected:
  INA228 bat_ina228;

public:
  void begin(MF_I2C *i2c, int8_t i2c_adr) override {
    if(i2c_adr == 0) i2c_adr = 0x40; //default address

    float Rshunt = cfg.bat_cal_i; //ohm

    // Configure INA226 -> sample time = 512 * 2 * 50us = 51.2ms => 20Hz
    bat_ina228.begin(i2c, i2c_adr);
    bat_ina228.setADCRange(false); //  false => 164 mV, true => 41 mV
    bat_ina228.setBusVoltageConversionTime(INA228_50_us);
    bat_ina228.setShuntVoltageConversionTime(INA228_50_us);
    bat_ina228.setTemperatureConversionTime(INA228_50_us);
    bat_ina228.setAverage(INA228_512_SAMPLES);
    bat_ina228.calibrate(Rshunt);
    bat_ina228.setMode(INA228_MODE_CONT_BUS_SHUNT);
  }

  bool update() override {
    if(!bat_ina228.isConversionReady()) return false;
    i   = bat_ina228.getAmpere();
    v   = bat_ina228.getBusVolt();
    w   = bat_ina228.getWatt();
    mah = bat_ina228.getCoulomb() * (1000.0 / 3600);
    wh  = bat_ina228.getWattHour();
    return true;
  }
};
