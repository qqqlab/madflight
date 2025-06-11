#pragma once

#include "bat.h"
#include "../hal/MF_I2C.h"
#include "INA226/INA226.h"

class BatGizmoINA226: public BatGizmo {
protected:
  INA226 bat_ina226;

public:
  BatState *state;

  BatGizmoINA226(BatState *state, MF_I2C *i2c_bus, uint8_t i2c_adr, float rshunt) { //rshunt in ohm
    this->state = state;
    state->i = 0;
    state->v = 0;
    state->mah = 0;
    state->wh = 0;
    state->ts = micros();

    if(i2c_adr == 0) i2c_adr = 0x40; //default address
    bat_ina226.begin(i2c_bus, i2c_adr);

    // Configure INA226 -> sample time = 2 * 128 * 140us = 36ms => 28Hz
    bat_ina226.configure(INA226_AVERAGES_128, INA226_BUS_CONV_TIME_140US, INA226_SHUNT_CONV_TIME_140US, INA226_MODE_SHUNT_BUS_CONT);

    bat_ina226.calibrate(rshunt);
  }

  bool update() override {
    if(!bat_ina226.isConversionReady()) return false;
    uint32_t now = micros();
    float dt_h = (now - state->ts) / 3600e6;
    state->i = bat_ina226.readShuntCurrent();
    state->v = bat_ina226.readBusVoltage();
    //state->w = bat_ina226.readBusPower(); //note w is not always equal to v * i, because w, v, and i are averaged values
    state->w = state->v * state->i; //This appears to be more accurate, specially for low values.
    state->mah += state->i * dt_h * 1000;
    state->wh += state->w * dt_h;
    state->ts = now;    
    return true;
  }
};
