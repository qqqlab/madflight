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
#include "MPUxxxx/MPUxxxx.h"

class ImuGizmoMPUXXXX : public ImuGizmo {
  private:
    ImuGizmoMPUXXXX() {} //private constructor
    ImuState *state = nullptr;

  public:
    MPUXXXX *sensor = nullptr;
    SensorDevice *dev = nullptr;

    ~ImuGizmoMPUXXXX() {
      delete sensor;
      delete dev;
    }

    static ImuGizmo* create(ImuConfig *config, ImuState *state) {
      if(!config || !state) return nullptr;

      // Create SensorDevice
      SensorDevice *dev = SensorDevice::createImuDevice(config);
      if(!dev) return nullptr;

      //translate cfg to mpu type
      MPUXXXX::MPU_Type mpu_type = MPUXXXX::MPU_Type::UNKNOWN;
      switch(config->gizmo) {
        case Cfg::imu_gizmo_enum::mf_AUTO    : mpu_type = MPUXXXX::MPU_Type::AUTO; break;
        case Cfg::imu_gizmo_enum::mf_MPU6000 : mpu_type = MPUXXXX::MPU_Type::MPU6000; break;
        case Cfg::imu_gizmo_enum::mf_MPU6050 : mpu_type = MPUXXXX::MPU_Type::MPU6050; break;
        case Cfg::imu_gizmo_enum::mf_MPU6500 : mpu_type = MPUXXXX::MPU_Type::MPU6500; break;
        case Cfg::imu_gizmo_enum::mf_MPU9150 : mpu_type = MPUXXXX::MPU_Type::MPU9150; break;
        case Cfg::imu_gizmo_enum::mf_MPU9250 : mpu_type = MPUXXXX::MPU_Type::MPU9250; break;
        default:
          delete dev;
          return nullptr;
      }

      // Create sensor
      MPUXXXX* sensor = new MPUXXXX();
      int rv = sensor->begin(mpu_type, dev, 16, 2000, config->sample_rate_requested);
      if(rv < 0) {
        if(config->gizmo != Cfg::imu_gizmo_enum::mf_AUTO) Serial.printf("IMU: ERROR creating MPU sensor, rv=%d\n",rv);
        delete sensor;
        delete dev;
        return nullptr;
      }

      // Create gizmo
      auto gizmo = new ImuGizmoMPUXXXX();
      gizmo->state = state;
      gizmo->sensor = sensor;
      gizmo->dev = dev;
      //return config
      strncpy(config->name, sensor->type_name(), sizeof(config->name));
      config->sample_rate = sensor->get_rate();

      return gizmo;
    }

    //Get sensor data in NED frame
    //x=North (forward), y=East (right), z=Down 
    //acc: gravitation force is positive in axis direction
    //gyro: direction of positive rotation by right hand rule, i.e. positive is: roll right, pitch up, yaw right
    void getMotion6NED(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) override
    {
      sensor->read6();
      *ax = sensor->rawa[0] * sensor->acc_multiplier;
      *ay = sensor->rawa[1] * sensor->acc_multiplier;
      *az = sensor->rawa[2] * sensor->acc_multiplier;
      *gx = sensor->rawg[0] * sensor->gyro_multiplier;
      *gy = sensor->rawg[1] * sensor->gyro_multiplier;
      *gz = sensor->rawg[2] * sensor->gyro_multiplier;
    }

    //Get sensor data in NED frame
    //x=North (forward), y=East (right), z=Down 
    //acc: gravitation force is positive in axis direction (sensor reports negative)
    //gyro: direction of positive rotation by right hand rule, i.e. positive is: yaw right, roll right, pitch up
    void getMotion9NED(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *mx, float *my, float *mz) override
    {
      sensor->read9();
      *ax = sensor->rawa[0] * sensor->acc_multiplier;
      *ay = sensor->rawa[1] * sensor->acc_multiplier;
      *az = sensor->rawa[2] * sensor->acc_multiplier;
      *gx = sensor->rawg[0] * sensor->gyro_multiplier;
      *gy = sensor->rawg[1] * sensor->gyro_multiplier;
      *gz = sensor->rawg[2] * sensor->gyro_multiplier;
      *mx = sensor->rawm[0] * sensor->mag_multiplier[0];
      *my = sensor->rawm[1] * sensor->mag_multiplier[1];
      *mz = sensor->rawm[2] * sensor->mag_multiplier[2];
    }
};
