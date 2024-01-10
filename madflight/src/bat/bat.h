/*========================================================================================================================
This file contains all necessary functions and code used for battery monitors to avoid cluttering the main code

Each USE_BAT_xxx section in this file defines a Battery class like BatteryNone
========================================================================================================================*/

#pragma once

class BatteryNone {
  public:
      float i = 0; //Battery current (A)
      float v = 0; //battery voltage (V)
      float mah = 0; //battery usage (Ah)
      float wh = 0; //battery usage (Wh)
      uint32_t interval_us = 10000; //update interval in us

  void setup() {
  }

  //returns true if battery was updated
  bool loop() {
    return false;
  }
};

#if defined HW_PIN_BAT_V || defined HW_PIN_BAT_I
  #include "BatteryADC.h"
  typedef BatteryADC Battery; //use typedef to avoid inheritance
#else
  typedef BatteryNone Battery; //use typedef to avoid inheritance
#endif

Battery bat;