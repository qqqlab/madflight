/*========================================================================================================================
This file contains all necessary functions and code used for barometer sensors to avoid cluttering the main code

Each USE_BARO_xxx section in this file defines a Barometer class like BarometerNone
========================================================================================================================*/

#pragma once

class BarometerNone {
public:
  float press_pa = 0; //pressure in Pascal
  float temp_c = 0; //temperature in Celcius

  int setup() {
    Serial.println("USE_BARO_NONE");
    return 0;
  }

  //returns true if pressure was updated
  bool update() {
    return false;
  }
};

#ifndef BARO_I2C_ADR
  #define BARO_I2C_ADR 0
#endif

//========================================================================================================================
// BMP280
//========================================================================================================================
#if defined USE_BARO_BMP280 

#include "BMP280.h"

class BarometerBMP280 {
private: 
  Adafruit_BMP280 bmp(i2c);

public:
  float press_pa = 0;
  float temp_c = 0;

  int setup() {
    Serial.println();
    unsigned status;
    status = bmp.begin(BARO_I2C_ADR, BMP280_CHIPID);
    Serial.printf("USE_BARO_BMP280   BARO_I2C_ADR 0x%02X  SensorID: 0x%02X\n", BARO_I2C_ADR, bmp.sensorID());

    if (!status) {
      Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                        "try a different address!"));
      Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
    }

    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X1,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_OFF,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */
    return status;
  }

  bool update() {
    //driver does not return whether data is fresh, return true if pressure changed
    float pressure_pa_new = bmp.readPressure();
    bool rv = (pressure_pa_new != press_pa)
    press_pa = pressure_pa_new;
    temp_c = bmp.readTemperature();
    return rv;
  }
};
typedef BarometerBMP280 Barometer;

//========================================================================================================================
// MS5611
//========================================================================================================================
#elif defined USE_BARO_MS5611

#include "MS5611.h"

class BarometerMS5611 {
private: 
  MS5611 ms5611;

public:
  float press_pa = 0;
  float temp_c = 0;

  int setup() {
    // Initialize MS5611 sensor
    // Ultra high resolution: MS5611_ULTRA_HIGH_RES
    // (default) High resolution: MS5611_HIGH_RES
    // Standard: MS5611_STANDARD
    // Low power: MS5611_LOW_POWER
    // Ultra low power: MS5611_ULTRA_LOW_POWER
    while(!ms5611.begin(MS5611_ULTRA_HIGH_RES))
    {
      Serial.println("USE_BARO_MS5611 failed, retry...\n");
      delay(500);
    }
    Serial.printf("USE_BARO_MS5611  BARO_I2C_ADR 0x%02X  refresh rate: %d Hz\n", BARO_I2C_ADR, (int)1000000/ms5611.getDelayUs());
    return 0;
  }

  bool update() {
      return (ms5611.getMeasurements(&press_pa, &temp_c) == 1); //ms5611.getMeasurements returns: 0=no update, 1=pressure updated, 2=temp updated
  }
};
typedef BarometerMS5611 Barometer;

//========================================================================================================================
// NONE
//========================================================================================================================
#else
typedef BarometerNone Barometer;
#endif

Barometer baro;
