/*==========================================================================================
MIT License

Copyright (c) 2026 https://madflight.com

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

// Driver for TDK Invensense MPU6000/6050/6500/9150/9250

#pragma once

#include "../common/SensorDevice.h"
#include "AK8963.h"
#include "AK8975.h"

class MPUXXXX {

  public:

    enum MPU_Type {
        UNKNOWN = 0,
        AUTO, //autodetect
        MPU6000,
        MPU6050,
        MPU60X0,
        MPU6500,
        MPU9150,
        MPU9250
    };

    //raw measurements in NED frame
    int16_t rawa[3]; //accelerometer
    int16_t rawg[3]; //gyroscope
    int16_t rawm[3]; //magnetometer
    int16_t rawt; //temperature

    float acc_multiplier;
    float gyro_multiplier;
    float mag_multiplier[3]; //multipliers for magnetometer in NED frame
    bool has_mag = false;

  private:
  
    SensorDevice *_dev;
    MPU_Type _type;
    int _rate_hz;

    uint32_t freq_slow = 0;
    uint32_t freq_fast = 0;

    //some MPU6000/6050 revisions have half acc resolution
    uint8_t rev1 = 0;
    uint8_t rev2 = 0;
    int acc_resolution = 32786;

  public:
    AK8963 *mag9250 = nullptr; //magnetometer of MPU9250
    AK8975 *mag9150 = nullptr; //magnetometer of MPU9150
 
    ~MPUXXXX() {
      delete mag9250;
      delete mag9150;
    }

    //return 0 on success, positive on error, negative on warning
    //Actual sample rate is set to the max possible rate smaller than/equal to the requested rate. 
    //I.e. 99999..1000->1000, 999..500->500, 499..333->333, 332..250->250, etc
    int begin(MPU_Type type, SensorDevice *dev, int gyro_scale_dps, int acc_scale_g, int rate_hz);
    int get_rate();
    const char* type_name();
    unsigned int whoami();
    void set_acc_resolution();
    void set_acc_scale_g(int scale_in_g);
    void set_gyro_scale_dps(int scale_in_dps);
    float getTemperature();
    //read raw sensor data (axis as defined by sensor)
    void read6();
    void read9();
};
