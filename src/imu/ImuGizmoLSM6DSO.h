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

#include "imu.h"
#include "common/SensorDevice.h"
#include "LSM6DSO/LSM6DSO.h"

class ImuGizmoLSM6DSO : public ImuGizmo {
  private:
    ImuGizmoLSM6DSO() {} //private constructor
    ImuState *state = nullptr;

  public:
    LSM6DSO *sensor = nullptr;
    SensorDevice *dev = nullptr;

    ~ImuGizmoLSM6DSO() {
      delete sensor;
      delete dev;
    }

    static ImuGizmo* create(ImuConfig *config, ImuState *state) {
      if(!config || !state) return nullptr;

      // Create SensorDevice (SPI or I2C)
      SensorDevice *dev = SensorDevice::createImuDevice(config);
      if(!dev) return nullptr;

      // Detect sensor, exit without printing anything
      if (!LSM6DSO::detect(dev)) {
          delete dev;
          return nullptr;
      }

      // Create sensor
      auto *sensor = new LSM6DSO();
      int rv = sensor->begin(dev);
      if(rv < 0) {
          delete sensor;
          delete dev;
          Serial.printf("IMU: LSM6DSO init failed, rv=%d\n", rv);
          return nullptr;
      }

      // Create gizmo
      auto gizmo = new ImuGizmoLSM6DSO();
      gizmo->state = state;
      gizmo->sensor = sensor;
      gizmo->dev = dev;
      //return config
      strncpy(config->name, "LSM6DSO", sizeof(config->name));
      config->sample_rate = sensor->actual_sample_rate_hz;
      Serial.printf("IMU: LSM6DSO started, sample_rate:%d\n", sensor->actual_sample_rate_hz);
      return gizmo;
  }

/* Get sensor data in NED frame
   x=North (forward), y=East (right), z=Down 
   acc: gravitation force is positive in axis direction: 
        [ax,ay,az] nose-down:[1,0,0], right-down:[0,1,0], level:[0,0,1]
   gyr: direction of positive rotation by right hand rule: 
        [gx,gy,gz] roll-right:[positive,0,0], pitch-up:[0,positive,0], yaw-right:[0,0,positive]

LSM6DSO has NWU orientation

  * +--+         Y
    |  | --> X   ^   Z-up
    +--+         |
    
*/

  void getMotion6NED(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) override {
    int16_t raw[6];
    sensor->readraw(raw);
    *gx =  raw[0] * sensor->gyr_scale; // N =  N
    *gy = -raw[1] * sensor->gyr_scale; // E = -W
    *gz = -raw[2] * sensor->gyr_scale; // D = -U
    *ax = -raw[3] * sensor->acc_scale; //-N = -N
    *ay =  raw[4] * sensor->acc_scale; //-E =  W
    *az =  raw[5] * sensor->acc_scale; //-D =  U
  }
};
