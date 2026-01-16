#pragma once

#include "bar.h"
#include "../hal/MF_I2C.h"
#include "MS5611/MS5611.h"

class BarGizmoMS5611: public BarGizmo {
protected: 
  MS5611 ms5611;
public:
  //float press_pa = 0;
  //float temp_c = 0;

  BarGizmoMS5611(MF_I2C *i2c, int8_t i2c_adr, uint32_t sample_rate) {
    (void) sample_rate;
    (void) i2c_adr; //gizmo has fixed address 0x77
    // Initialize MS5611 gizmo
    // Ultra high resolution: MS5611_ULTRA_HIGH_RES
    // (default) High resolution: MS5611_HIGH_RES
    // Standard: MS5611_STANDARD
    // Low power: MS5611_LOW_POWER
    // Ultra low power: MS5611_ULTRA_LOW_POWER
    int tries = 5;
    while(!ms5611.begin(i2c, MS5611_ULTRA_HIGH_RES))
    {
      Serial.println("BAR: MS5611 failed, retry...\n");
      delay(500);
      tries--;
      if(tries<=0) return;
    }
    Serial.printf("BAR: MS5611 I2C_ADR=0x77 refresh_rate=%dHz\n", (int)1000000/ms5611.getDelayUs());
  }

  bool update(float *press, float *temp) override {
      return (ms5611.getMeasurements(press, temp) == 1); //ms5611.getMeasurements returns: 0=no update, 1=pressure updated, 2=temp updated
  }
};