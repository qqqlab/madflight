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
#include "ImuGizmoICM45686.h"
#include "ImuGizmoICM426XX.h"
#include "ImuGizmoLSM6DSV.h"

class ImuGizmoAuto : public ImuGizmo {
  private:
    ImuGizmoAuto() {} //private constructor

  public:
    static ImuGizmo* create(ImuConfig *config, ImuState *state) {
        if(!config || !state) return nullptr;

        // AUTO is currently only for SPI sensors
        if(!config->spi_bus || config->spi_cs < 0) return nullptr;

        MPU_Interface *dev = new MPU_InterfaceSPI(config->spi_bus, config->spi_cs);
        dev->setFreq(1000000); //low speed

        // Detect sensor type and attempt to create gizmo for it
        int tries = 5;
        ImuGizmo* gizmo = nullptr;
        while(!gizmo && tries--){
          if(dev->readReg(0x72) == 0xE9) {
            Serial.println("IMU: AUTO detected ICM45686");
            gizmo = ImuGizmoICM45686::create(config, state);

          }else if(dev->readReg(0x75) == 0x47) {
            Serial.println("IMU: AUTO detected ICM42688P");
            gizmo = ImuGizmoICM426XX::create(config, state);

          }else if(dev->readReg(0x0F) == 0x70) {
            Serial.println("IMU: AUTO detected LSM6DSV");
            gizmo = ImuGizmoLSM6DSV::create(config, state);
          }
          delay(10);
        }

        if(!gizmo) Serial.println("IMU: AUTO nothing detected");

        delete dev;

        return gizmo;
    }

  bool update() override {
    return false;
  }

  void getMotion6NED(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) override {
    (void)ax;
    (void)ay;
    (void)az;
    (void)gx;
    (void)gy;
    (void)gz;
  }

  int begin(int gyro_scale_dps, int acc_scale_g, int rate_hz) {
    (void) gyro_scale_dps;
    (void) acc_scale_g;
    (void) rate_hz;
    return 0;
  }

  int get_rate() {
    return 0;
  }
};
