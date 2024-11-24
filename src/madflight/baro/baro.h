/*=================================================================================================
Each BARO_USE_xxx section in this file defines a specific Barometer class
=================================================================================================*/

#pragma once

#define BARO_USE_NONE 1
#define BARO_USE_BMP280 2
#define BARO_USE_BMP388 3
#define BARO_USE_BMP390 4
#define BARO_USE_MS5611 5


class BarometerSensor {
public:
  virtual int setup(uint32_t sampleRate) = 0;
  virtual bool update(float *press, float *temp) = 0; //returns true if pressure was updated
};


#ifndef BARO_I2C_ADR
  #define BARO_I2C_ADR 0
#endif

//=================================================================================================
// None or undefined
//=================================================================================================
#if BARO_USE == BARO_USE_NONE || !defined BARO_USE
class BarometerNone: public BarometerSensor {
public:
  int setup(uint32_t sampleRate) {
    (void) sampleRate;
    Serial.println("BARO: BARO_USE_NONE");
    return 0;
  }

  //returns true if pressure was updated
  bool update(float *press, float *temp) {
    (void) press;
    (void) temp;
    return false;
  }
};

BarometerNone baro_sensor;

//=================================================================================================
// BMP280
//=================================================================================================
#elif BARO_USE == BARO_USE_BMP280 

#include "BMP280.h"

Adafruit_BMP280 baro_BMP280(i2c);

class BarometerBMP280: public BarometerSensor {

public:
  int setup(uint32_t sampleRate) {
    (void) sampleRate;
    unsigned status;
    status = baro_BMP280.begin(BARO_I2C_ADR, BMP280_CHIPID);
    Serial.printf("BARO: BARO_USE_BMP280   BARO_I2C_ADR: 0x%02X  SensorID: 0x%02X\n", BARO_I2C_ADR, baro_BMP280.sensorID());

    if (!status) {
      Serial.println(F("Could not find a valid BMP280 sensor, check wiring or try a different address!"));
      Serial.print("SensorID was: 0x");
      Serial.println(baro_BMP280.sensorID(),16);
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

  bool update(float *press, float *temp) {
    //driver does not return whether data is fresh, return true if pressure changed
    float press_new = baro_BMP280.readPressure();
    bool rv = (press_new != *press);
    *press = press_new;
    *temp = baro_BMP280.readTemperature();
    return rv;
  }
};

BarometerBMP280 baro_sensor;

//=================================================================================================
// BMP390 / BMP388
//=================================================================================================
#elif BARO_USE == BARO_USE_BMP390 || BARO_USE == BARO_USE_BMP388

#include "bmp3/bmp3.h"

bfs::Bmp3 bmp(&Wire, (bfs::Bmp3::I2cAddr)BARO_I2C_ADR);

class BarometerBMP390: public BarometerSensor {

public:
  int setup(uint32_t sampleRate) {
    Serial.printf("BARO: BARO_USE_BMP390/BARO_USE_BMP388  BARO_I2C_ADR: 0x%02X\n", BARO_I2C_ADR);
    if (!bmp.Begin()) {
      Serial.println("BARO: BARO_USE_BMP390/BARO_USE_BMP388 sensor not found");
      return 1;
    }

    bmp.ConfigFilterCoef(bfs::Bmp3::FILTER_COEF_OFF);

    //set equal or next higher sample rate
    if(sampleRate > 100) {
      //f = 200;
      bmp.ConfigOsMode(bfs::Bmp3::OS_MODE_PRES_1X_TEMP_1X);
    }else if(sampleRate > 50) {
      //f = 100;
      bmp.ConfigOsMode(bfs::Bmp3::OS_MODE_PRES_2X_TEMP_1X);
    }else if(sampleRate > 25) {
      //f = 50;
      bmp.ConfigOsMode(bfs::Bmp3::OS_MODE_PRES_8X_TEMP_1X);
    }else if(sampleRate > 12) {
      //f = 25;
      bmp.ConfigOsMode(bfs::Bmp3::OS_MODE_PRES_16X_TEMP_2X);
    }else {
      //f = 12.5;
      bmp.ConfigOsMode(bfs::Bmp3::OS_MODE_PRES_32X_TEMP_2X); //12.5Hz
    }

    return 0;
  }

  bool update(float *press, float *temp) {
    if (!bmp.Read()) return false;
    *press = bmp.pres_pa();
    *temp = bmp.die_temp_c();
    return true;
  }
};

BarometerBMP390 baro_sensor;

//=================================================================================================
// MS5611
//=================================================================================================
#elif BARO_USE == BARO_USE_MS5611

#include "MS5611.h"

class BarometerMS5611: public BarometerSensor {
private: 
  MS5611 ms5611;

public:
  //float press_pa = 0;
  //float temp_c = 0;

  int setup(uint32_t sampleRate) {
    (void) sampleRate;
    // Initialize MS5611 sensor
    // Ultra high resolution: MS5611_ULTRA_HIGH_RES
    // (default) High resolution: MS5611_HIGH_RES
    // Standard: MS5611_STANDARD
    // Low power: MS5611_LOW_POWER
    // Ultra low power: MS5611_ULTRA_LOW_POWER
    while(!ms5611.begin(MS5611_ULTRA_HIGH_RES))
    {
      Serial.println("BARO: BARO_USE_MS5611 failed, retry...\n");
      delay(500);
    }
    Serial.printf("BARO: BARO_USE_MS5611  BARO_I2C_ADR 0x%02X  refresh rate: %d Hz\n", BARO_I2C_ADR, (int)1000000/ms5611.getDelayUs());
    return 0;
  }

  bool update(float *press, float *temp) {
      return (ms5611.getMeasurements(press, temp) == 1); //ms5611.getMeasurements returns: 0=no update, 1=pressure updated, 2=temp updated
  }
};

BarometerMS5611 baro_sensor;

//=================================================================================================
// Invalid value
//=================================================================================================
#else
  #error "invalid BARO_USE value"
#endif


//========================================================================================================================//
// Barometer Class Implementation
//========================================================================================================================//

#include "../interface.h"

int Barometer::setup(uint32_t sampleRate, float filterAltHertz, float filterVzHertz) {
  _sampleRate = sampleRate;
  _samplePeriod = 1000000 / sampleRate;
  B_alt = constrain(1 - exp(-2 * PI * filterAltHertz / sampleRate), 0.0f, 1.0f);
  B_vz = constrain(1 - exp(-2 * PI * filterVzHertz / sampleRate), 0.0f, 1.0f);
  dt = 0;
  ts = micros();
  int rv = baro_sensor.setup(sampleRate);
  if(rv != 0) {
    press = 0;
    temp = 0;
    alt = 0;
    altRaw = 0;
    vz = 0;
  }else{
    update(); //get first reading
    alt = altRaw; //jumpstart filtered altitude
  }
  Serial.printf("BARO: sample_rate=%dHz filt_alt=%.1fHz filt_vz=%.1fHz rv=%d \n", (int)sampleRate, filterAltHertz, filterVzHertz, rv);
  return rv;
}

bool Barometer::update() {
  if (micros() - ts >= _samplePeriod) {
    uint32_t tsnew = micros();
    dt = (tsnew - ts) / 1000000.0;
    baro_sensor.update(&press, &temp);
    altRaw = (101325.0 - press) / 12.0;
    float altnew = alt + B_alt * (altRaw - alt); //Low-pass filtered altitude
    float vznew = (altnew - alt) / dt;
    vz += B_vz * (vznew - vz); //Low-pass filtered velocity
    alt = altnew;
    ts = tsnew;
    return true;
  }
  return false;
}

//global Barometer class instance
Barometer baro;
