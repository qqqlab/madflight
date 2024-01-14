/*=================================================================================================
interface.h - This file defines the interfaces for sensors and other devices
=================================================================================================*/

#pragma once

//=================================================================================================
// Radio Receiver
//=================================================================================================

class Rcin {
  public:
    int pwm[RCIN_MAX_CHANNELS]; //reveived channel pwm values
    virtual void setup() = 0;
    bool update(); //returns true if channel pwm data was updated
    bool connected();
  private:
    uint32_t update_time = 0;
    virtual bool _update() = 0; //returns true if channel pwm data was updated
};

extern Rcin &rcin;

//=================================================================================================
// IMU
//=================================================================================================

class Imu {
  public:
    float ax = 0; //"North" acceleration in G
    float ay = 0; //"East" acceleration in G
    float az = 0; //"Down" acceleration in G
    float gx = 0; //"North" rotation speed in deg/s
    float gy = 0; //"East" rotation speed in deg/s
    float gz = 0; //"Down" rotation speed in deg/s
    float mx = 0; //"North" magnetic flux in uT
    float my = 0; //"East" magnetic flux in uT
    float mz = 0; //"Down" magnetic flux in uT
    virtual bool hasMag() = 0; //returns true if IMU has a magnetometer
    virtual int setup(uint32_t sampleRate) = 0;
    virtual void update() = 0;
    uint32_t getSampleRate() {return _sampleRate;}
  protected:
    uint32_t _sampleRate = 0; //sensor sample rate in Hz
};

extern Imu &imu;

//=================================================================================================
// Barometer
//=================================================================================================

class Barometer {
  public:
    float press_pa = 0; //pressure in Pascal
    float temp_c = 0; //temperature in Celcius
    virtual int setup() = 0;
    virtual bool update() = 0; //returns true if pressure was updated
};

extern Barometer &baro;

//=================================================================================================
// Magnetometer
//=================================================================================================

class Magnetometer {
  public:
    float x = 0; //"North" magnetic flux in uT
    float y = 0; //"East" magnetic flux in uT
    float z = 0; //"Down" magnetic flux in uT
    virtual bool installed() = 0; //returns true if a sensor is installed
    virtual int setup() = 0; //returns 0 on success
    bool update(); //returns true if values updated
  private:
    uint32_t mag_time = 0;
    virtual void _update() = 0;
};

extern Magnetometer &mag;

//=================================================================================================
// Battery
//=================================================================================================

class Battery {
  public:
    float i = 0; //Battery current (A)
    float v = 0; //battery voltage (V)
    float mah = 0; //battery usage (Ah)
    float wh = 0; //battery usage (Wh)
    uint32_t interval_us = 10000; //update interval in us
    virtual void setup() = 0;
    virtual bool update() = 0; //returns true if battery was updated
};

extern Battery &bat;
