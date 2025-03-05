#pragma once

#ifndef MAG_I2C_ADR
  #define MAG_I2C_ADR 0 //use default address
#endif

class MagSensor {
public:
  virtual int setup(MF_I2C *i2c, int8_t i2c_adr, uint32_t sampleRate) = 0;
  virtual bool read_uT(float *x, float *y, float *z) = 0; //returns true if new sample was retrieved
};

class Mag {
  public:
    float x = 0; //"North" magnetic flux in uT
    float y = 0; //"East" magnetic flux in uT
    float z = 0; //"Down" magnetic flux in uT
    
    Mag(MagSensor *sensor) { 
      this->sensor = sensor; 
    }
    MagSensor *sensor;

    bool installed() { //returns true if a sensor is installed
    #if MAG_USE == MAG_USE_NONE | !defined MAG_USE
      return false;
    #else
       return true;
    #endif
    }

    int setup(uint32_t sampleRate = 100) {
      _samplePeriod = 1000000 / sampleRate;
      return sensor->setup(mf_i2c, MAG_I2C_ADR, sampleRate);
    }

    bool update() { //returns true if new sample received
      if(micros() - mag_time < _samplePeriod) return false;
      mag_time = micros();
      return sensor->read_uT(&x, &y, &z);
    }
  private:
    uint32_t mag_time = 0; //last sample time in [us]
    uint32_t _samplePeriod = 0; //sensor sample period in [us]
};

extern Mag mag;
