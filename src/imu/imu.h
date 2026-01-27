/*==========================================================================================
MIT License

Copyright (c) 2023-2026 https://madflight.com

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

//Available excecution methods (not all platforms support all methods)
#define IMU_EXEC_IRQ 1            //execute in IRQ context on first core (works on STM32. Does NOT work on ESP32, RP2040)
#define IMU_EXEC_FREERTOS 2       //execute as IRQ triggered high priority FreeRTOS task on same core as setup() (works on ESP32, RP2040)
#define IMU_EXEC_FREERTOS_OTHERCORE 3 //execute as IRQ triggered high priority FreeRTOS task on second core (works on RP2040)

#include "../hal/MF_I2C.h"
#include <SPI.h>
#include "../cfg/cfg.h"
#include "../mag/mag.h"

//default settings
#ifndef IMU_GYRO_DPS
  #define IMU_GYRO_DPS 2000 //Full scale gyro range in deg/sec. Most IMUs support 250,500,1000,2000. Can use any value here, driver will pick next greater setting.
#endif
#ifndef IMU_ACCEL_G
  #define IMU_ACCEL_G 16 //Full scale accelerometer range in G's. Most IMUs support 2,4,8,16. Can use any value here, driver will pick next greater setting.
#endif

struct ImuConfig {
  public:
    uint32_t sample_rate_requested = 1000; //requested sample rate [Hz]
    int pin_int = -1; //IMU data ready interrupt pin
    Cfg::imu_gizmo_enum gizmo = Cfg::imu_gizmo_enum::mf_NONE; //the gizmo to use
    SPIClass *spi_bus = nullptr; //SPI bus
    int spi_cs = -1; //SPI select pin
    MF_I2C *i2c_bus = nullptr; //i2c bus (only used if spi_bus == nullptr)
    uint8_t i2c_adr = 0; //i2c address. 0=default address
    bool uses_i2c = false; //use I2C bus?
    int pin_clkin = -1; //IMU clkin pin
    Mag *pmag = nullptr; //mag pointer, used to store IMU internal magnetometer samples

    //config values returned by gizmo
    bool has_mag = false; //true if IMU has built-in magnetometer
    uint32_t sample_rate = 0; //actual sample rate [Hz]

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
    float temp = 0; //temperature [C]
};

//Note: Instantiate a gizmo with: ImuGizmo *newgizmo = new ImuGizmoXXX::create(ImuConfig *config, ImuState *state)
class ImuGizmo {
public:
  virtual ~ImuGizmo() {}
  virtual const char* name() = 0;
  virtual void getMotion6NED(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) = 0;
  virtual void getMotion9NED(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *mx, float *my, float *mz) {(void)ax;(void)ay;(void)az;(void)gx;(void)gy;(void)gz;(void)mx;(void)my;(void)mz;};
};

class Imu : public ImuState {
  public:
    ImuConfig config;
    ImuGizmo *gizmo = nullptr;

    int setup(); // Use config to setup gizmo, returns 0 on success, or error code
    bool update(); // Returns true if state was updated
    bool installed() {return (gizmo != nullptr); } //returns true if a gizmo is installed
    const char* name() {return (gizmo ? gizmo->name() : "NONE");}

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
    void statReset();
    uint32_t getSampleRate() {return config.sample_rate;}  //sensor sample rate in Hz

    //low level interrupt handler (should be private, but is public, because called from interrupt)
    void _interrupt_handler();
};

extern Imu imu;

extern void __attribute__((weak)) imu_loop();
