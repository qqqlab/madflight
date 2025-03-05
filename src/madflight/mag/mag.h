/*=================================================================================================
Each MAG_USE_xxx section in this file defines a specific Magnetometer class

The nominal magnetometer sample rate is 100Hz

Body frame is NED: x-axis North(front), y-axis East(right), z-axis Down

Unit of Measure is uT (micro Tesla)
=================================================================================================*/

#pragma once

#define MAG_USE_NOINST -1
#define MAG_USE_NONE 0
#define MAG_USE_QMC5883L 1

#ifndef MAG_USE
  #define MAG_USE MAG_USE_NONE
#endif

#include "mag_interface.h"
#include "../common/MF_I2C.h"
#include "MagSensorNone.h"
#include "MagSensorQMC5883L.h"

//create global Mag class instance
#if BARO_USE == BARO_USE_NOINST
  //do nothing
#elif MAG_USE == MAG_USE_NONE
  Mag mag(new MagSensorNone);
#elif MAG_USE == MAG_USE_QMC5883L
  Mag mag(new MagSensorQMC5883L);
#else
  #error "invalid MAG_USE value"
#endif
