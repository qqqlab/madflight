/*========================================================================================================================
This file contains all necessary functions and code used for magetometer sensors to avoid cluttering the main code

Each MAG_USE_xxx section in this file defines:
 - int mag_Setup() - init, returns 0 on success.
 - bool mag_Read(float *mx,float *mz,float *my) - read a magnetometer sample in uT

The magnetometer sample rate is 100Hz

Body frame is NED: x-axis North(front), y-axis East(right), z-axis Down
========================================================================================================================*/

#define MAG_USE_NONE 1
#define MAG_USE_QMC5883L 2

#ifndef MAG_I2C_ADR
  #define MAG_I2C_ADR 0
#endif

uint32_t mag_time = 0;

void mag_Read2(float *mx, float *my, float *mz);

bool mag_Read(float *mx, float *my, float *mz) {
  if(micros() - mag_time < 10000) return false;
  mag_time = micros();
  mag_Read2(mx, my, mz);
  return true;
}

//========================================================================================================================
// QMC5883L
//========================================================================================================================
#if MAG_USE == MAG_USE_TYPEAQMC5883L

#include "QMC5883L.h"
QMC5883L<HW_WIRETYPE> mag(i2c);

int mag_Setup() {
  mag.begin();
  Serial.printf("MAG_USE_QMC5883L detect=%d\n",mag.detect());
  return 0;
}

void mag_Read2(float *mx, float *my, float *mz) {
  mag.read_uT(mx, my, mz);
}

//=====================================================================================================================
// None or undefined
//=====================================================================================================================
#elif MAG_USE == MAG_USE_NONE || !defined MAG_USE
int mag_Setup() {
  return 0;
}

void mag_Read2(float *mx, float *my, float *mz) {
  *mx = 0;
  *my = 0;
  *mz = 0;
}
  
//=====================================================================================================================
// Invalid value
//=====================================================================================================================
#else
  #error "invalid MAG_USE value"
#endif