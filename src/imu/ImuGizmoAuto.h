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
#include "ImuGizmoICM45686.h"
#include "ImuGizmoICM426XX.h"
#include "ImuGizmoLSM6DSV.h"
#include "ImuGizmoLSM6DSO.h"
#include "ImuGizmoBMI270.h"
#include "ImuGizmoMPUXXXX.h"

class ImuGizmoAuto : public ImuGizmo {
  private:
    ImuGizmoAuto() {} //private constructor

  public:
    const char* name() override {return "AUTO";}
    static ImuGizmo* create(ImuConfig *config, ImuState *state) {
        if(!config || !state) return nullptr;

        // Detect sensor type and attempt to create gizmo for it
        int tries = 5;
        ImuGizmo* gizmo = nullptr;
        while(!gizmo && tries--){
          if(!gizmo) gizmo = ImuGizmoICM45686::create(config, state);
          if(!gizmo) gizmo = ImuGizmoICM426XX::create(config, state);
          if(!gizmo) gizmo = ImuGizmoLSM6DSV::create(config, state);
          if(!gizmo) gizmo = ImuGizmoLSM6DSO::create(config, state);
          if(!gizmo) gizmo = ImuGizmoBMI270::create(config, state);
          if(!gizmo) gizmo = ImuGizmoMPUXXXX::create(config, state);
          delay(10);
        }

        if(!gizmo) {
          Serial.println("IMU: AUTO no sensor detected");
        }else{
          Serial.printf("IMU: AUTO detected %s\n", gizmo->name());
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
