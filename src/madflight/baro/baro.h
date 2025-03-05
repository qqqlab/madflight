#pragma once

#define BARO_USE_NOINST -1 //do not create global Baro instance
#define BARO_USE_NONE 0
#define BARO_USE_BMP280 1
#define BARO_USE_BMP388 2
#define BARO_USE_BMP390 3
#define BARO_USE_MS5611 4

#ifndef BARO_I2C_ADR
  #define BARO_I2C_ADR 0 //use default address
#endif

#ifndef BARO_USE
  #define BARO_USE BARO_USE_NONE
#endif

#include "baro_interface.h"
#include "BaroSensorNone.h"
#include "BaroSensorBMP280.h"
#include "BaroSensorBMP390.h"
#include "BaroSensorMS5611.h"

//create global Baro class instance
#if BARO_USE == BARO_USE_NOINST
  //do nothing
#elif BARO_USE == BARO_USE_NONE
  Baro baro(new BaroSensorNone);
#elif BARO_USE == BARO_USE_BMP280
  Baro baro(new BaroSensorBMP280);
#elif BARO_USE == BARO_USE_BMP390 || BARO_USE == BARO_USE_BMP388
  Baro baro(new BaroSensorBMP390);
#elif BARO_USE == BARO_USE_MS5611
  Baro baro(new BaroSensorMS5611); //fix i2c address 0x77
#else
  #error "invalid BARO_USE value"
#endif

//========================================================================================================================//
// Baro Class Implementation
//========================================================================================================================//
#include <math.h>

bool Baro::installed() {
  #if BARO_USE == BARO_USE_NONE || !defined BARO_USE
    return false;
  #else
    return true;
  #endif
}

int Baro::setup(uint32_t sampleRate) {
  _sampleRate = sampleRate;
  _samplePeriod = 1000000 / sampleRate;
  dt = 0;
  ts = micros();
  int rv = sensor->setup(mf_i2c, BARO_I2C_ADR, sampleRate);
  if(rv != 0) {
    press = 0;
    temp = 0;
    alt = 0;
  }else{
    update(); //get first reading
  }
  Serial.printf("BARO: sample_rate=%dHz rv=%d\n", (int)sampleRate, rv);
  return rv;
}

bool Baro::update() {
  if (micros() - ts >= _samplePeriod) {
    uint32_t tsnew = micros();
    dt = (tsnew - ts) / 1000000.0;
    sensor->update(&press, &temp);
    float P = press;
    //float T = temp;
    //alt = 153.84348f * (1 - pow(P/101325.0f, 0.19029496f)) * (T + 273.15f); //hypsometric formula - reduces to barometric with T=15C
    alt = 44330.0f * (1 - pow(P/101325.0f, 0.19029496f)); //barometric formula  0.19029496 = 1/5.255
    //alt = (101325.0f - P) / 12.0f; //linearisation of barometric formula at sealevel
    ts = tsnew;
    return true;
  }
  return false;
}
