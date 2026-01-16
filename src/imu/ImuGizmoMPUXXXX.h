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
#include "MPUxxxx/MPU_interface.h"
#include "MPUxxxx/MPUxxxx.h"

class ImuGizmoMPUXXXX : public ImuGizmo {
  private:
    ImuGizmoMPUXXXX() {} //private constructor

  public:
    static ImuGizmo* create(ImuConfig *config, ImuState *state) {
        if(!config || !state) return nullptr;

        //check bus config
        if (!config->uses_i2c && (!config->spi_bus || config->spi_cs < 0)) {
          Serial.println("IMU: ERROR check config - SPI sensor without imu_spi_bus and/or pin_imu_cs");
          return nullptr;
        }
        if (config->uses_i2c && !config->i2c_bus) {
          Serial.println("IMU: ERROR check config imu_bus_type - I2C sensor without imu_i2c_bus");
          return nullptr;
        }

        //create gizmo
        ImuGizmo *gizmo = nullptr;
        if (!config->uses_i2c) {
          //------------
          // SPI Sensors
          //------------
          switch(config->gizmo) {
            case Cfg::imu_gizmo_enum::mf_MPU9250 : {
              auto mpu_iface = new MPU_InterfaceSPI(config->spi_bus, config->spi_cs);
              gizmo = new MPUXXXX(MPUXXXX::MPU9250, mpu_iface);
              //return config
              strncpy(config->name, "MPU9250", sizeof(config->name));
              break;
            }
            case Cfg::imu_gizmo_enum::mf_MPU6500 : {
              auto mpu_iface = new MPU_InterfaceSPI(config->spi_bus, config->spi_cs);
              gizmo = new MPUXXXX(MPUXXXX::MPU6500, mpu_iface);
              //return config
              strncpy(config->name, "MPU6500", sizeof(config->name));
              break;
            }
            case Cfg::imu_gizmo_enum::mf_MPU6000 : {
              auto mpu_iface = new MPU_InterfaceSPI(config->spi_bus, config->spi_cs);
              gizmo = new MPUXXXX(MPUXXXX::MPU6000, mpu_iface);
              //return config
              strncpy(config->name, "MPU6000", sizeof(config->name));
              break;
            }
            default : {
              Serial.println("IMU: ERROR check config imu_bus_type - Sensor does not support SPI bus");
            }
          }
        } else {
          //------------
          // I2C Sensors
          //------------
          switch(config->gizmo) {
            case Cfg::imu_gizmo_enum::mf_MPU9250 : {
              auto mpu_iface = new MPU_InterfaceI2C(config->i2c_bus, config->i2c_adr);
              gizmo = new MPUXXXX(MPUXXXX::MPU9250, mpu_iface);
              //return config
              strncpy(config->name, "MPU9250", sizeof(config->name));
              config->has_mag = true;
              break;
            }
            case Cfg::imu_gizmo_enum::mf_MPU9150 : {
              auto mpu_iface = new MPU_InterfaceI2C(config->i2c_bus, config->i2c_adr);
              gizmo = new MPUXXXX(MPUXXXX::MPU9150, mpu_iface);
              //return config
              strncpy(config->name, "MPU9150", sizeof(config->name));
              config->has_mag = true;
              break;
            }
            case Cfg::imu_gizmo_enum::mf_MPU6500 : {
              auto mpu_iface = new MPU_InterfaceI2C(config->i2c_bus, config->i2c_adr);
              gizmo = new MPUXXXX(MPUXXXX::MPU6500, mpu_iface);
              //return config
              strncpy(config->name, "MPU6500", sizeof(config->name));
              break;
            }
            case Cfg::imu_gizmo_enum::mf_MPU6050 : {
              auto mpu_iface = new MPU_InterfaceI2C(config->i2c_bus, config->i2c_adr);
              gizmo = new MPUXXXX(MPUXXXX::MPU6050, mpu_iface);
              //return config
              strncpy(config->name, "MPU6050", sizeof(config->name));
              break;
            }
            case Cfg::imu_gizmo_enum::mf_MPU6000 : {
              auto mpu_iface = new MPU_InterfaceI2C(config->i2c_bus, config->i2c_adr);
              gizmo = new MPUXXXX(MPUXXXX::MPU6000, mpu_iface);
              //return config
              strncpy(config->name, "MPU6000", sizeof(config->name));
              break;
            }
            default : {
              Serial.println("IMU: ERROR check config - Sensor does not support I2C bus");
            }
          }
        }

        return gizmo;
    }

  //dummy function, as this gizmo never gets instantiated
  void getMotion6NED(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) override {
    (void)ax;
    (void)ay;
    (void)az;
    (void)gx;
    (void)gy;
    (void)gz;
  }
};
