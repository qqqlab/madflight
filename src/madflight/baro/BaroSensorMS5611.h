#pragma once

#include "baro_interface.h"
#include "../common/MF_I2C.h"
#include "MS5611/MS5611.h"

class BaroSensorMS5611: public BaroSensor {
protected: 
  MS5611 ms5611;
public:
  //float press_pa = 0;
  //float temp_c = 0;

  int setup(MF_I2C *i2c, int8_t i2c_adr, uint32_t sampleRate) override {
    (void) sampleRate;
    (void) i2c_adr; //sensor has fixed address 0x77
    // Initialize MS5611 sensor
    // Ultra high resolution: MS5611_ULTRA_HIGH_RES
    // (default) High resolution: MS5611_HIGH_RES
    // Standard: MS5611_STANDARD
    // Low power: MS5611_LOW_POWER
    // Ultra low power: MS5611_ULTRA_LOW_POWER
    while(!ms5611.begin(i2c, MS5611_ULTRA_HIGH_RES))
    {
      Serial.println("BARO: BARO_USE_MS5611 failed, retry...\n");
      delay(500);
    }
    Serial.printf("BARO: BARO_USE_MS5611 BARO_I2C_ADR=0x%02X  refresh_rate=%dHz\n", BARO_I2C_ADR, (int)1000000/ms5611.getDelayUs());
    return 0;
  }

  bool update(float *press, float *temp) override {
      return (ms5611.getMeasurements(press, temp) == 1); //ms5611.getMeasurements returns: 0=no update, 1=pressure updated, 2=temp updated
  }
};