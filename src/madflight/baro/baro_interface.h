#pragma once

#include "../common/MF_I2C.h"

class BaroSensor {
public:
  virtual int setup(MF_I2C *i2c, int8_t i2c_adr, uint32_t sampleRate) = 0;
  virtual bool update(float *press, float *temp) = 0; //returns true if pressure was updated
};

class Baro {
  public:
    //Barometer sample data
    uint32_t ts = 0;  // Sample timestamp in [us]
    float dt = 0;     // Time since last sample in [seconds]
    float press = 0;  // Pressure in [Pascal]
    float alt = 0;    // Approximate International Standard Atmosphere (ISA) Altitude in [m]
    float temp = 0;   // Temperature in [Celcius]

    Baro(BaroSensor *sensor) {this->sensor = sensor;}
    BaroSensor *sensor;

    bool installed(); //returns true if a sensor is installed
    int setup(uint32_t sampleRate = 100); //default: 100 Hz sample rate
    bool update(); //returns true if pressure was updated
    uint32_t getSampleRate() {return _sampleRate;}  //sensor sample rate in [Hz]
    uint32_t getSamplePeriod() {return _samplePeriod;} //sensor sample period in [us]

  protected:
    uint32_t _sampleRate = 0; //sensor sample rate in [Hz]
    uint32_t _samplePeriod = 0; //sensor sample period in [us]
};

// The global Baro instance
extern Baro baro;
