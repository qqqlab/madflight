/*========================================================================================================================
This file contains all necessary functions and code used for battery monitors to avoid cluttering the main code

Each BAT_USE_xxx section in this file defines a Battery class like BatteryNone
========================================================================================================================*/

#pragma once

#define BAT_USE_NONE 1
#define BAT_USE_ADC 2

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

//=====================================================================================================================
// ADC Sensor
//=====================================================================================================================
#if BAT_USE == BAT_USE_ADC
  #include "BatteryADC.h"
  typedef BatteryADC Battery; //use typedef to avoid inheritance
  
//=====================================================================================================================
// None or undefined
//=====================================================================================================================
#elif BAT_USE == BAT_USE_NONE || !defined BAT_USE
  typedef BatteryNone Battery; //use typedef to avoid inheritance
  
//=====================================================================================================================
// Invalid value
//=====================================================================================================================
#else
  #error "invalid BAT_USE value"
#endif


Battery bat;