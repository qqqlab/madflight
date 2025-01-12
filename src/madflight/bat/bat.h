/*=================================================================================================
Each BAT_USE_xxx section in this file defines a specific Battery class
=================================================================================================*/

#pragma once

#define BAT_USE_NONE 1
#define BAT_USE_ADC 2
#define BAT_USE_INA226 3
#define BAT_USE_INA228 4

#include "../interface.h"

/* INTERFACE
class Battery {
  public:
      float i = 0; //Battery current (A)
      float v = 0; //battery voltage (V)
      float w = 0; //battery power (W)
      float mah = 0; //battery usage (Ah)
      float wh = 0; //battery usage (Wh)
      uint32_t interval_us = 10000; //update interval in us

  virtual void setup() = 0;
  virtual bool update() = 0; //returns true if battery was updated
};

extern Battery &bat;
*/

//=================================================================================================
// None or undefined
//=================================================================================================
#if BAT_USE == BAT_USE_NONE || !defined BAT_USE

class BatteryNone: public Battery {
  public:
  void setup() {}
  bool update() { return false; }
};

BatteryNone bat_instance;

//=================================================================================================
// ADC Sensor
//=================================================================================================
#elif BAT_USE == BAT_USE_ADC

#include "BatteryADC.h"

BatteryADC bat_instance;

//=================================================================================================
// INA226 Sensor
//=================================================================================================
#elif BAT_USE == BAT_USE_INA226

// Default INA226 address is 0x40
#ifndef BAT_I2C_ADR
  #define BAT_I2C_ADR 0x40
#endif

#include "INA226/INA226.h"
INA226 bat_ina226;

class BatteryINA226: public Battery {
  public:
  void setup() {
    float Rshunt = cfg.BAT_CAL_I; //ohm

    bat_ina226.begin(i2c, BAT_I2C_ADR);

    // Configure INA226 -> sample time = 2 * 128 * 140us = 36ms => 28Hz
    bat_ina226.configure(INA226_AVERAGES_128, INA226_BUS_CONV_TIME_140US, INA226_SHUNT_CONV_TIME_140US, INA226_MODE_SHUNT_BUS_CONT);

    bat_ina226.calibrate(Rshunt);
  }
  bool update() {
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

BatteryINA226 bat_instance;

//=================================================================================================
// INA228 Sensor
//=================================================================================================
#elif BAT_USE == BAT_USE_INA228

// Default INA228 address is 0x40
#ifndef BAT_I2C_ADR
  #define BAT_I2C_ADR 0x40
#endif

#include "INA228/INA228.h"
INA228 bat_ina228;

class BatteryINA228: public Battery {
  public:
  void setup() {
    float Rshunt = cfg.BAT_CAL_I; //ohm

    // Configure INA226 -> sample time = 512 * 2 * 50us = 51.2ms => 20Hz
    bat_ina228.begin(i2c, BAT_I2C_ADR);
    bat_ina228.setADCRange(false); //  false => 164 mV, true => 41 mV
    bat_ina228.setBusVoltageConversionTime(INA228_50_us);
    bat_ina228.setShuntVoltageConversionTime(INA228_50_us);
    bat_ina228.setTemperatureConversionTime(INA228_50_us);
    bat_ina228.setAverage(INA228_512_SAMPLES);
    bat_ina228.calibrate(Rshunt);
    bat_ina228.setMode(INA228_MODE_CONT_BUS_SHUNT);
  }
  bool update() {
    if(!bat_ina228.isConversionReady()) return false;
    i   = bat_ina228.getAmpere();
    v   = bat_ina228.getBusVolt();
    w   = bat_ina228.getWatt();
    mah = bat_ina228.getCoulomb() * (1000.0 / 3600);
    wh  = bat_ina228.getWattHour();
    return true;
  }
};

BatteryINA228 bat_instance;

//=================================================================================================
// Invalid value
//=================================================================================================
#else
  #error "invalid BAT_USE value"
#endif

Battery &bat = bat_instance;