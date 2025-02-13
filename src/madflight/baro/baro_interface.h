#pragma once

class Barometer {
  public:
    //Barometer sample data
    uint32_t ts = 0;  // Sample timestamp in [us]
    float dt = 0;     // Time since last sample in [seconds]
    float press = 0;  // Pressure in [Pascal]
    float alt = 0;    // Approximate International Standard Atmosphere (ISA) Altitude in [m]
    float temp = 0;   // Temperature in [Celcius]

    bool installed(); //returns true if a sensor is installed
    int setup(uint32_t sampleRate = 100); //default: 100 Hz sample rate
    bool update(); //returns true if pressure was updated
    uint32_t getSampleRate() {return _sampleRate;}  //sensor sample rate in [Hz]
    uint32_t getSamplePeriod() {return _samplePeriod;} //sensor sample period in [us]

  protected:
    uint32_t _sampleRate = 0; //sensor sample rate in [Hz]
    uint32_t _samplePeriod = 0; //sensor sample period in [us]
};

extern Barometer baro;
