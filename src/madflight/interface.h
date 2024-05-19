/*=================================================================================================
interface.h - This file defines the interfaces for sensors and other devices
=================================================================================================*/

#pragma once

//=================================================================================================
// Radio Receiver
//=================================================================================================

class Rcin {
  public:
    uint16_t *pwm; //pwm channel data. values: 988-2012
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
    //sample data
    uint32_t ts = 0; //sample low level interrupt trigger timestamp in us
    float dt = 0; //time since last sample in seconds
    uint32_t update_cnt = 0; //number of updates
    float ax = 0; //"North" acceleration in G
    float ay = 0; //"East" acceleration in G
    float az = 0; //"Down" acceleration in G
    float gx = 0; //"North" rotation speed in deg/s
    float gy = 0; //"East" rotation speed in deg/s
    float gz = 0; //"Down" rotation speed in deg/s
    float mx = 0; //"North" magnetic flux in uT
    float my = 0; //"East" magnetic flux in uT
    float mz = 0; //"Down" magnetic flux in uT

    //interrupt statistics
    uint32_t overrun_cnt = 0; //number of interrupt overruns (should stay 0)
    bool _imu_interrupt_busy = false; //is interrupt handler running?
    uint32_t runtime_int = 0; //interrupt latency from start of interrupt handler to start of interrupt task in us
    uint32_t runtime_bus = 0; //runtime of SPI/I2C bus transfer in us
    uint32_t runtime_tot_max = 0; //max runtime imu update including transfer in us

    //pointer to onUpdate event handler
    void (*onUpdate)(void) = NULL;

    //methods
    int setup(uint32_t sampleRate);
    bool waitNewSample(); //wait for new sample, returns false on fail
    bool hasMag(); //returns true if IMU has a magnetometer
    bool usesI2C(); //returns true if IMU uses I2C bus (not SPI bus)    
    uint32_t getSampleRate() {return _sampleRate;}  //sensor sample rate in Hz
    uint32_t getSamplePeriod() {return (_sampleRate != 0 ? 1000000 / _sampleRate : 1000000);} //sensor sample period in us

    //low level interrupt handler (should be private, but is public, because called from interrupt)
    void _interrupt_handler();

  protected:
    uint32_t _sampleRate = 0; //sensor sample rate in Hz
};

extern Imu imu;

//=================================================================================================
// Barometer
//=================================================================================================

class Barometer {
  public:
    //sample data
    uint32_t ts = 0; //sample timestamp in us
    float dt = 0; //time since last sample in seconds
    float press = 0; //pressure in Pascal
    float alt = 0; // Approximate International Standard Atmosphere (ISA) Altitude in meter
    float temp = 0; //temperature in Celcius

    int setup(uint32_t sampleRate);
    bool update(); //returns true if pressure was updated
    uint32_t getSampleRate() {return _sampleRate;}  //sensor sample rate in Hz
    uint32_t getSamplePeriod() {return _samplePeriod;} //sensor sample period in us

  protected:
    uint32_t _sampleRate = 0; //sensor sample rate in Hz
    uint32_t _samplePeriod = 0; //sensor sample period in us
};

extern Barometer baro;

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
    float w = 0; //battery power (W)    
    float mah = 0; //battery usage (Ah)
    float wh = 0; //battery usage (Wh)
    uint32_t interval_us = 10000; //update interval in us
    virtual void setup() = 0;
    virtual bool update() = 0; //returns true if battery was updated
};

extern Battery &bat;

//=================================================================================================
// LED
//=================================================================================================

class Led {
  public:
    virtual void setup(int pin, uint8_t led_on_value) = 0;
    virtual void set(bool set_on) = 0;
    void on();
    void off();
    void toggle();
    void blink(int times);
  protected:
    bool state = false;
    int pin = -1;
    uint8_t led_on_value = 0;
};

extern Led &led;
