#pragma once

#include "bat.h"
#include "../hal/MF_I2C.h"
#include "INA228/INA228.h"

class BatGizmoINA228: public BatGizmo {
protected:
  INA228 bat_ina228;

public:
  BatState *state;

  BatGizmoINA228(BatState *state, MF_I2C *i2c_bus, uint8_t i2c_adr, float rshunt) { //rshunt in ohm
    this->state = state;
    state->i = 0;
    state->v = 0;
    state->mah = 0;
    state->wh = 0;
    state->ts = micros();

    if(i2c_adr == 0) i2c_adr = 0x40; //default address

    // Configure INA226 -> sample time = 512 * 2 * 50us = 51.2ms => 20Hz
    bat_ina228.begin(i2c_bus, i2c_adr);
    bat_ina228.setADCRange(false); //  false => 164 mV, true => 41 mV
    bat_ina228.setBusVoltageConversionTime(INA228_50_us);
    bat_ina228.setShuntVoltageConversionTime(INA228_50_us);
    bat_ina228.setTemperatureConversionTime(INA228_50_us);
    bat_ina228.setAverage(INA228_512_SAMPLES);
    bat_ina228.calibrate(rshunt);
    bat_ina228.setMode(INA228_MODE_CONT_BUS_SHUNT);
  }

  bool update() override {
    if(!bat_ina228.isConversionReady()) return false;
    state->i   = bat_ina228.getAmpere();
    state->v   = bat_ina228.getBusVolt();
    state->w   = bat_ina228.getWatt();
    state->mah = bat_ina228.getCoulomb() * (1000.0 / 3600);
    state->wh  = bat_ina228.getWattHour();
    state->ts = micros();
    return true;
  }
};
