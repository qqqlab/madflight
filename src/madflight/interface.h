/*==========================================================================================
interface.h - This file defines the interfaces for sensors and other devices

MIT License

Copyright (c) 2024 https://madflight.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
===========================================================================================*/

#pragma once

//=================================================================================================
// AHRS
//=================================================================================================
class Ahrs {
  public:
    float gx = 0, gy = 0, gz = 0; //corrected and filtered imu gyro measurements in [deg/sec]
    float ax = 0, ay = 0, az = 0; //corrected and filtered imu accel measurements in [g]
    float mx = 0, my = 0, mz = 0; //corrected and filtered external magnetometer or internal imu mag measurements in [uT]
    float q[4] = {1,0,0,0};  //quaternion NED reference frame
    float roll = 0;          //roll in degrees: -180 to 180, roll right is positive
    float pitch = 0;         //pitch in degrees: -90 to 90, pitch up is positive
    float yaw = 0;           //yaw in degrees: -180 to 180, yaw right is positive
    float B_gyr = 1.0; //gyr filter constant
    float B_acc = 1.0; //acc filter constant
    float B_mag = 1.0; //mag filter constant
    uint32_t ts = 0; //IMU sample timestamp

    static constexpr float rad_to_deg = 57.2957795132f;
    static constexpr float deg_to_rad = 0.0174532925199f;

    virtual void setup(float gyrLpFreq, float accLpFreq, float magLpFreq) = 0;
    virtual void setInitalOrientation() {}
    void update(); //get imu+mag data, filter it, and call fusionUpdate() to update q

    float getAccelUp(); //get acceleration in earth-frame up direction in [m/s^2]

  protected:
    virtual void fusionUpdate() = 0;
    void computeAngles();
    void setFromMag(float *q);
};

extern Ahrs &ahrs;

//=================================================================================================
// RCIN - Radio Receiver
//=================================================================================================

class Rcin_interface {
  public:
    virtual void setup() = 0;
    virtual bool update() = 0; //returns true if channel pwm data was updated
    virtual bool connected() = 0;
    virtual void calibrate() = 0; //interactive calibration

    uint16_t *pwm; //pwm channel data. values: 988-2012
    float throttle = 0; //throttle stick value 0.0 (zero throttle/stick back) to 1.0 (full throttle/stick forward)
    float roll = 0; //roll stick value -1.0 (left) to 1.0 (right)
    float pitch = 0; //pitch stick value -1.0 (pitch up/stick back) to 1.0 (pitch down/stick forward)
    float yaw = 0; //yaw stick value -1.0 (left) to 1.0 (right)
    float vspeed = 0; //vertical speed stick value -1.0 (descent/stick back) to 1.0 (ascent/stick forward)
    bool arm = false; //arm switch state
    uint8_t flightmode = 0; //flightmode 0 to 5
};

extern Rcin_interface &rcin;

//telemetry
void rcin_telemetry_gps(int32_t latitude, int32_t longitude, uint16_t groundspeed, uint16_t gps_heading, uint16_t altitude, uint8_t num_satellites);
void rcin_telemetry_flight_mode(const char *flight_mode);
void rcin_telemetry_attitude(float pitch, float roll, float yaw);
void rcin_telemetry_battery(float voltage_V, float current_A, int fuel_mAh, uint8_t remaining);

//=================================================================================================
// IMU
//=================================================================================================

class Imu {
  public:
    //imu sample data (raw, uncorrected and unfiltered) 
    uint32_t ts = 0; //sample low level interrupt trigger timestamp in us
    float dt = 0; //time since last sample in seconds
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
    volatile uint32_t interrupt_cnt = 0; //number of times interrupt was triggered since start
    uint32_t update_cnt = 0; //number of times imu task was executed since start
    uint32_t overrun_cnt = 0; //number of interrupt overruns (should stay 0)
    bool _imu_interrupt_busy = false; //is interrupt handler running?
    
    //statistics - sum of values since last statReset()
    uint32_t stat_cnt = 0; //number of accumulated samples
    uint32_t stat_reset_ts = 0; //last time statReset() was called
    uint32_t stat_latency = 0; //summed  interrupt latency from start of interrupt handler to start of interrupt task in us
    uint32_t stat_io_runtime = 0; //summed  runtime of SPI/I2C io transfer in us
    uint32_t stat_runtime = 0; //summed  runtime imu update including io transfer in us
    uint32_t stat_runtime_max = 0; //max runtime imu update including transfer in us, since last reset to 0

    //pointer to onUpdate event handler
    void (*onUpdate)(void) = NULL;

    //methods
    int setup(uint32_t sampleRate = 1000); //default: 1000 Hz sample rate
    bool waitNewSample(); //wait for new sample, returns false on fail
    bool hasMag(); //returns true if IMU has a magnetometer
    bool usesI2C(); //returns true if IMU uses I2C bus (not SPI bus)
    uint32_t getSampleRate() {return _sampleRate;}  //sensor sample rate in Hz
    uint32_t getSamplePeriod() {return (_sampleRate != 0 ? 1000000 / _sampleRate : 1000000);} //sensor sample period in us
    void statReset();

    //low level interrupt handler (should be private, but is public, because called from interrupt)
    void _interrupt_handler();

  protected:
    uint32_t _sampleRate = 0; //sensor sample rate in Hz
};

extern Imu imu;

//=================================================================================================
// BARO - Barometer
//=================================================================================================

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

//=================================================================================================
// MAG - Magnetometer
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
// BAT - Battery
//=================================================================================================

class Battery {
  public:
    float i = 0; //Battery current (A)
    float v = 0; //battery voltage (V)
    float w = 0; //battery power (W)
    float mah = 0; //battery usage (mAh)
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
    virtual void setup() = 0;
    virtual void set(bool set_on) = 0;
    void on();
    void off();
    void toggle();
    void blink(int times);
    void enable();
  protected:
    bool state = false;
    int pin = -1;
    uint8_t led_on_value = 0;
    bool enabled = false;
};

extern Led &led;

//=================================================================================================
// BB - Black Box Logging
//=================================================================================================

class BlackBox {
  public:
    //loggers
    virtual void log_baro() {}
    virtual void log_bat() {}
    virtual void log_gps() {}
    virtual void log_imu() {}
    virtual void log_mode(uint8_t fm, const char* name) {(void)fm;(void)name;}
    virtual void log_msg(const char* msg) {(void)msg;}
    virtual void log_parm(const char* name, float value, float default_value) {(void)name;(void)value;(void)default_value;}
    virtual void log_pid() {}
    virtual void log_att() {}
    virtual void log_ahrs() {}
    virtual void log_sys() {}

    //Blackbox Interface
    virtual void setup() {} //setup blackbox
    virtual void start() {} //start logging (create new file)
    virtual void stop()  {} //stop logging (closes file)
    virtual void erase() {} //erase all log files
    virtual void dir()   {} //list log files
    virtual void bench() {} //benchmark read/write to blackbox
    virtual void info()  {} //blackbox info (memory size, free space, etc.)
};

extern BlackBox &bb;

//=================================================================================================
// OUT - Motor and servo output driver
//=================================================================================================

class Out {
  public:
    bool armed = false; //output is enabled when armed == true

    void setup();
    bool setupMotor(uint8_t i, int pin, int freq_hz = 400, int pwm_min_us = 950, int pwm_max_us = 2000);
    bool setupServo(uint8_t i, int pin, int freq_hz = 400, int pwm_min_us = 950, int pwm_max_us = 2000);
    void set(uint8_t i, float value); //set output (might not be output value because of armed == false)
    float get(uint8_t i); //get last set value (might not be output value because of armed == false)
    char getType(uint8_t i); //type 'M' or 'S'

  private:
    bool _setupOutput(char typ, uint8_t i, int pin, int freq_hz, int pwm_min_us, int pwm_max_us);
};

extern Out out;

//=================================================================================================
// PID - PID controllers
//=================================================================================================

class PID {
  public:
    float PID = 0; //PID output value
};

PID PIDroll;
PID PIDpitch;
PID PIDyaw;


//=================================================================================================
// ALT - Altitude Estimator
//=================================================================================================

class AltEst {
  public:
    virtual void setup(float alt) = 0; //setup with default parameters and initial altitude in [m]
    virtual void updateAccelUp(float a, uint32_t ts) = 0; //a: accel up in [m/s^2], ts: timestamp in [us]
    virtual void updateBaroAlt(float alt, uint32_t ts) = 0; //alt: barometric altitude in [m], ts: timestamp in [us]
    virtual float getH() = 0; //altitude estimate in [m]
    virtual float getV() = 0; //vertical up speed (climb rate) estimate in [m/s]
    virtual void print(); //print state info
};

extern AltEst &alt;