/*=================================================================================================
Each BAT_USE_xxx section in this file defines a specific Battery class
=================================================================================================*/

#pragma once

#define BAT_USE_NONE 1
#define BAT_USE_ADC 2
#define BAT_USE_INA226 3

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

#include "INA226/INA226.h"
INA226 bat_ina226;

class BatteryINA226: public Battery {
  public:
  void setup() {
    float Rshunt = cfg.BAT_CAL_I; //ohm
    float iMaxExpected = 0.080 / Rshunt; // ampere (max 80mv shunt voltage)

    // Default INA226 address is 0x40
    bat_ina226.begin(i2c);

    // Configure INA226 -> sample time = 2 * 128 * 140us = 36ms => 28Hz
    bat_ina226.configure(INA226_AVERAGES_128, INA226_BUS_CONV_TIME_140US, INA226_SHUNT_CONV_TIME_140US, INA226_MODE_SHUNT_BUS_CONT);

    // Calibrate INA226.
    bat_ina226.calibrate(Rshunt, iMaxExpected);
  }
  bool update() {
    static uint32_t ts = micros();
    if(!bat_ina226.isConversionReady()) return false;
    uint32_t now = micros();
    float dt_h = (now - ts) / 3600e6;
    ts = now;
    i = bat_ina226.readShuntCurrent();
    v = bat_ina226.readBusVoltage();
    w = bat_ina226.readBusPower(); //note w is not always equal to v * i, because w, v, and i are averaged values
    mah += i * dt_h * 1000;
    wh += w * dt_h;
    return true;
  }
};

BatteryINA226 bat_instance;

//=================================================================================================
// Invalid value
//=================================================================================================
#else
  #error "invalid BAT_USE value"
#endif

Battery &bat = bat_instance;