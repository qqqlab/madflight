/*=================================================================================================
Each BAT_USE_xxx section in this file defines a specific Battery class
=================================================================================================*/

#pragma once

#define BAT_USE_NONE 1
#define BAT_USE_ADC 2

#include "../interface.h"

/* INTERFACE
class Battery {
  public:
      float i = 0; //Battery current (A)
      float v = 0; //battery voltage (V)
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
  void setup() {)
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
// Invalid value
//=================================================================================================
#else
  #error "invalid BAT_USE value"
#endif

Battery &bat = bat_instance;