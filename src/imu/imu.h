/*==========================================================================================
MIT License

Copyright (c) 2023-2025 https://madflight.com

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

#include "../hal/MF_I2C.h"
#include <SPI.h>
#include "../cfg/cfg.h"

struct ImuConfig {
  public:
    uint32_t sampleRate = 1000; //sample rate [Hz]
    int pin_int = -1; //IMU data ready interrupt pin
    Cfg::imu_gizmo_enum gizmo = Cfg::imu_gizmo_enum::mf_NONE; //the gizmo to use
    SPIClass *spi_bus = nullptr; //SPI bus
    int spi_cs = -1; //SPI select pin
    MF_I2C *i2c_bus = nullptr; //i2c bus (only used if spi_bus == nullptr)
    uint8_t i2c_adr = 0; //i2c address. 0=default address
    bool uses_i2c = false; //use I2C bus?
};

//imu sample data (raw, uncorrected and unfiltered) 
struct ImuState {
public:
    uint32_t ts = 0; //sample low level interrupt trigger timestamp [us]
    float dt = 0; //time since last sample [seconds]
    float ax = 0; //"North" acceleration [G]
    float ay = 0; //"East" acceleration [G]
    float az = 0; //"Down" acceleration [G]
    float gx = 0; //"North" rotation speed [deg/s]
    float gy = 0; //"East" rotation speed [deg/s]
    float gz = 0; //"Down" rotation speed [deg/s]
    float mx = 0; //"North" magnetic flux [uT]
    float my = 0; //"East" magnetic flux [uT]
    float mz = 0; //"Down" magnetic flux [uT]
    float temp = 0; //temperature [C]
    // sensor fusion IMUs return quaternions
    float q[4] = {1.0, 0.0, 0.0, 0.0};
};

class ImuGizmo {
public:
  virtual int begin(int gyro_scale_dps, int acc_scale_g, int rate_hz) = 0;
  virtual bool update() {return false;}
  
  virtual int get_rate() = 0;

  virtual ~ImuGizmo() {}
  virtual void getMotion6NED(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) = 0;
  virtual void getMotion9NED(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *mx, float *my, float *mz) {(void)ax;(void)ay;(void)az;(void)gx;(void)gy;(void)gz;(void)mx;(void)my;(void)mz;};
  // if has_sensor_fusion, one of these has to be implemented depending on whether Magnetometer is available (9DOF) or not (6DOF)
  virtual void get6DOF(float *q0, float *q1, float *q2, float *q3) {(void)q0;(void)q1;(void)q2;(void)q3;}
  virtual void get9DOF(float *q0, float *q1, float *q2, float *q3) {(void)q0;(void)q1;(void)q2;(void)q3;}
  virtual bool installed() {return true;}
  bool has_mag = false;
  bool uses_i2c = false;
  bool has_sensor_fusion = false;
  bool interrupt_has_rising_edge = true;
};

class Imu : public ImuState {
  public:
    ImuConfig config;

    ImuGizmo *gizmo = nullptr;

    int setup(); // Use config to setup gizmo, returns 0 on success, or error code
    bool update(); // Returns true if state was updated
    bool installed() {return (gizmo != nullptr); } //returns true if a gizmo is installed

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
    bool waitNewSample(); //wait for new sample, returns false on fail
    bool hasMag(); //returns true if IMU has a magnetometer
    bool hasSensorFusion(); //returns true if IMU has sensor fusion
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

extern void __attribute__((weak)) imu_loop();
