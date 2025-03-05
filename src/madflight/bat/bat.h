#pragma once

#define BAT_USE_NOINST -1
#define BAT_USE_NONE 0
#define BAT_USE_ADC 1
#define BAT_USE_INA226 2
#define BAT_USE_INA228 3

#include "bat_interface.h"
#include "BatteryNone.h"
#include "BatteryADC.h"
#include "BatteryINA226.h"
#include "BatteryINA228.h"

#ifndef BAT_I2C_ADR
  #define BAT_I2C_ADR 0
#endif

#ifndef BAT_USE
  #define BAT_USE BAT_USE_NONE
#endif

//create global Battery class instance
#if BAT_USE == BAT_USE_NOINST
  //do nothing
#elif BAT_USE == BAT_USE_NONE
  BatteryNone bat_instance;
  Battery &bat = bat_instance;
#elif BAT_USE == BAT_USE_ADC
  BatteryADC bat_instance;
  Battery &bat = bat_instance;
#elif BAT_USE == BAT_USE_INA226
  BatteryINA226 bat_instance;
  Battery &bat = bat_instance;
#elif BAT_USE == BAT_USE_INA228
  BatteryINA228 bat_instance;
  Battery &bat = bat_instance;
#else
  #error "invalid BAT_USE value"
#endif
