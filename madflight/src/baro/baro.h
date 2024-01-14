/*=================================================================================================
Each BARO_USE_xxx section in this file defines a specific Barometer class
=================================================================================================*/

#pragma once

#define BARO_USE_NONE 1
#define BARO_USE_BMP280 2
#define BARO_USE_MS5611 3

#include "../interface.h"

/* INTERFACE
class Barometer {
public:
  float press_pa = 0; //pressure in Pascal
  float temp_c = 0; //temperature in Celcius
  virtual int setup() = 0;
  virtual bool update() = 0; //returns true if pressure was updated
};

extern Barometer &baro;
*/

#ifndef BARO_I2C_ADR
  #define BARO_I2C_ADR 0
#endif

//=================================================================================================
// None or undefined
//=================================================================================================
#if BARO_USE == BARO_USE_NONE || !defined BARO_USE
class BarometerNone: public Barometer {
public:
  //float press_pa = 0; //pressure in Pascal
  //float temp_c = 0; //temperature in Celcius

  int setup() {
    Serial.println("BARO_USE_NONE");
    return 0;
  }

  //returns true if pressure was updated
  bool update() {
    return false;
  }
};

BarometerNone baro_instance;

//=================================================================================================
// BMP280
//=================================================================================================
#elif BARO_USE == BARO_USE_BMP280 

#include "BMP280.h"

Adafruit_BMP280 baro_BMP280(i2c);

class BarometerBMP280: public Barometer {

public:
  //float press_pa = 0;
  //float temp_c = 0;

  int setup() {
    Serial.println();
    unsigned status;
    status = baro_BMP280.begin(BARO_I2C_ADR, BMP280_CHIPID);
    Serial.printf("BARO_USE_BMP280   BARO_I2C_ADR 0x%02X  SensorID: 0x%02X\n", BARO_I2C_ADR, baro_BMP280.sensorID());

    if (!status) {
      Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                        "try a different address!"));
      Serial.print("SensorID was: 0x"); Serial.println(baro_BMP280.sensorID(),16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
    }

    baro_BMP280.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X1,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_OFF,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */
    return status;
  }

  bool update() {
    //driver does not return whether data is fresh, return true if pressure changed
    float pressure_pa_new = baro_BMP280.readPressure();
    bool rv = (pressure_pa_new != press_pa);
    press_pa = pressure_pa_new;
    temp_c = baro_BMP280.readTemperature();
    return rv;
  }
};

BarometerBMP280 baro_instance;

//=================================================================================================
// MS5611
//=================================================================================================
#elif BARO_USE == BARO_USE_MS5611

#include "MS5611.h"

class BarometerMS5611: public Barometer {
private: 
  MS5611 ms5611;

public:
  //float press_pa = 0;
  //float temp_c = 0;

  int setup() {
    // Initialize MS5611 sensor
    // Ultra high resolution: MS5611_ULTRA_HIGH_RES
    // (default) High resolution: MS5611_HIGH_RES
    // Standard: MS5611_STANDARD
    // Low power: MS5611_LOW_POWER
    // Ultra low power: MS5611_ULTRA_LOW_POWER
    while(!ms5611.begin(MS5611_ULTRA_HIGH_RES))
    {
      Serial.println("BARO_USE_MS5611 failed, retry...\n");
      delay(500);
    }
    Serial.printf("BARO_USE_MS5611  BARO_I2C_ADR 0x%02X  refresh rate: %d Hz\n", BARO_I2C_ADR, (int)1000000/ms5611.getDelayUs());
    return 0;
  }

  bool update() {
      return (ms5611.getMeasurements(&press_pa, &temp_c) == 1); //ms5611.getMeasurements returns: 0=no update, 1=pressure updated, 2=temp updated
  }
};

BarometerMS5611 baro_instance;

//=================================================================================================
// Invalid value
//=================================================================================================
#else
  #error "invalid BARO_USE value"
#endif

Barometer &baro = baro_instance;
