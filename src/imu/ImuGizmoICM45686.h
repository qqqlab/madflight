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

#pragma once

#include "./imu.h"
#include "./ICM45686/ICM45686.h"

class ImuGizmoICM45686 : public ImuGizmo {
  private:
    ImuGizmoICM45686() {} //private constructor
    ImuState *state = nullptr;
    PWM *pwm_clkin = nullptr;

  public:
    const char* name() override {return "ICM45686";}
    ICM45686 *sensor = nullptr;
    SensorDevice *dev = nullptr;

    ~ImuGizmoICM45686() {
        delete sensor;
        delete dev;
        delete pwm_clkin;
    }

    static ImuGizmo* create(ImuConfig *config, ImuState *state) {
        if(!config || !state) return nullptr;

        // Create SensorDevice (SPI or I2C)
        SensorDevice *dev = SensorDevice::createImuDevice(config);
        if(!dev) return nullptr;

        bool use_clkin = (config->pin_clkin >= 0);
        PWM *pwm_clkin = nullptr;
        if(use_clkin) {
            //32 kHz output
            int freq = 32000;
            pwm_clkin = new PWM();
            pwm_clkin->begin(config->pin_clkin, freq, 0, 1000000.0/freq);
            pwm_clkin->writeFactor(0.5);
        }

        auto *sensor = new ICM45686();
        int rv = sensor->begin(dev, config->sample_rate_requested, use_clkin);

        if(rv < 0) {
            delete sensor;
            delete dev;
            return nullptr;
        }

        //create gizmo
        auto gizmo = new ImuGizmoICM45686();
        gizmo->state = state;
        gizmo->sensor = sensor;
        gizmo->dev = dev;
        gizmo->pwm_clkin = pwm_clkin;
        //return config
        config->sample_rate = sensor->sampleRateActual;

        return gizmo;
    }

/* Get sensor data in NED frame
   x=North (forward), y=East (right), z=Down 
   acc: gravitation force is positive in axis direction: 
        [ax,ay,az] nose-down:[1,0,0], right-down:[0,1,0], level:[0,0,1]
   gyr: direction of positive rotation by right hand rule: 
        [gx,gy,gz] roll-right:[positive,0,0], pitch-up:[0,positive,0], yaw-right:[0,0,positive]

ICM45686 has NWU orientation

  * +--+         Y
    |  | --> X   ^   Z-up
    +--+         |
    
*/
  
  void getMotion6NED(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) override {
    sensor->read();
    *ax = -sensor->acc[0]; //-N = -N
    *ay =  sensor->acc[1]; //-E =  W
    *az =  sensor->acc[2]; //-D =  U
    *gx =  sensor->gyr[0]; // N =  N
    *gy = -sensor->gyr[1]; // E = -W
    *gz = -sensor->gyr[2]; // D = -U
  }
};
