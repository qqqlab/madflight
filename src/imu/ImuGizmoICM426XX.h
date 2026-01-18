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
#include "./ICM426XX/ICM426XX.h"

class ImuGizmoICM426XX : public ImuGizmo {
  private:
    ImuGizmoICM426XX() {} //private constructor
    ImuState *state = nullptr;

  public:
    ICM426XX *sensor = nullptr;
    SensorDevice *dev = nullptr;

    ~ImuGizmoICM426XX() {
      delete sensor;
      delete dev;
    }

    static ImuGizmo* create(ImuConfig *config, ImuState *state) {
      if(!config || !state) return nullptr;

      // Create SensorDevice
      SensorDevice *dev = SensorDevice::createImuDevice(config);
      if(!dev) return nullptr;

      // Detect sensor, exit without printing anything
      if (!ICM426XX::detect(dev)) {
          delete dev;
          return nullptr;
      }

      // Create sensor
      ICM426XX *sensor = ICM426XX::create(dev, config->pin_clkin);
      if(!sensor) {
        delete dev;
        return nullptr;
      }

      // Create gizmo
      auto gizmo = new ImuGizmoICM426XX();
      gizmo->state = state;
      gizmo->sensor = sensor;
      gizmo->dev = dev;
      //return config
      strncpy(config->name, sensor->type_name(), sizeof(config->name));
      config->sample_rate = sensor->sampling_rate_hz;

      return gizmo;
    }

/* Get sensor data in NED frame
   x=North (forward), y=East (right), z=Down 
   acc: gravitation force is positive in axis direction: 
        [ax,ay,az] nose-down:[1,0,0], right-down:[0,1,0], level:[0,0,1]
   gyr: direction of positive rotation by right hand rule: 
        [gx,gy,gz] roll-right:[positive,0,0], pitch-up:[0,positive,0], yaw-right:[0,0,positive]

ICM42688 has NWU orientation

  * +--+         Y
    |  | --> X   ^   Z-up
    +--+         |
    
*/
  
  void getMotion6NED(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) override {
    int16_t accgyr[7];
    sensor->read(accgyr);
    *ax = -accgyr[0] * sensor->acc_scale; //-N = -N
    *ay =  accgyr[1] * sensor->acc_scale; //-E =  W
    *az =  accgyr[2] * sensor->acc_scale; //-D =  U
    *gx =  accgyr[3] * sensor->gyr_scale; // N =  N
    *gy = -accgyr[4] * sensor->gyr_scale; // E = -W
    *gz = -accgyr[5] * sensor->gyr_scale; // D = -U
  }
};
