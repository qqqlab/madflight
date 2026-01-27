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

#include <Arduino.h> //Serial
#include "mag.h"
#include "MagGizmoQMC5883L.h"
#include "MagGizmoQMC6309.h"
#include "MagGizmoRM3100.h"
#include "MagGizmoQMC5883P.h"
#include "MagGizmoMMC5603.h"
#include "MagGizmoBMM150.h"

//create global module instance
Mag mag;

int Mag::setup() {
  cfg.printModule("IMU");

  _samplePeriod = 1000000 / config.sample_rate;

  //clear state
   mx = 0; //"North" magnetic flux [uT]
   my = 0; //"East" magnetic flux [uT]
   mz = 0; //"Down" magnetic flux [uT]
   ts = 0; //last sample time in [us]

  //create gizmo
  delete gizmo;
  switch(config.gizmo) {
    case Cfg::mag_gizmo_enum::mf_NONE :
      gizmo = nullptr;
      break;
    case Cfg::mag_gizmo_enum::mf_QMC5883L :
    case Cfg::mag_gizmo_enum::mf_QMC5883 :
      if(config.i2c_bus) {
        gizmo = new MagGizmoQMC5883L(config.i2c_bus, config.i2c_adr);
      }
      break;
    case Cfg::mag_gizmo_enum::mf_QMC6309 :
      if(config.i2c_bus) {
        gizmo = new MagGizmoQMC6309(config.i2c_bus); //i2c address is always 0x7C
      }
      break;
    case Cfg::mag_gizmo_enum::mf_RM3100 :
      gizmo = MagGizmoRM3100::create(config.i2c_bus);
      break;
    case Cfg::mag_gizmo_enum::mf_QMC5883P :
      if(config.i2c_bus) {
        gizmo = new MagGizmoQMC5883P(config.i2c_bus); //i2c address is always 0x2C
      }
      break;
    case Cfg::mag_gizmo_enum::mf_MMC5603 :
      gizmo = MagGizmoMMC5603::create(&config, (MagState*)this); //i2c address is always 0x30
      break;
    case Cfg::mag_gizmo_enum::mf_BMM150 :
      gizmo = MagGizmoBMM150::create(&config, (MagState*)this);
      break;
  }

  //check gizmo
  if(!gizmo && config.gizmo != Cfg::mag_gizmo_enum::mf_NONE) {
    Serial.println("\nIMU: ERROR check pin/bus config\n");
    return -1001;
  }

  return 0;
}

bool Mag::update() {
  runtimeTrace.start();
  bool updated = (gizmo != nullptr);
  updated = updated && schedule.interval(_samplePeriod); //wait for next sample interval
  updated = updated && gizmo->update(&mx, &my, &mz);

  if(updated) {
    //handle rotation for different mounting positions
    switch((Cfg::mag_align_enum)cfg.mag_align) {
      case Cfg::mag_align_enum::mf_CW0 :
        break;
      case Cfg::mag_align_enum::mf_CW90 :
        { float tmp; tmp=mx; mx=-my; my=tmp; }
        break;
      case Cfg::mag_align_enum::mf_CW180 :
        { mx=-mx; my=-my; }
        break;
      case Cfg::mag_align_enum::mf_CW270 :
        { float tmp; tmp=mx; mx=my; my=-tmp; }
        break;
      case Cfg::mag_align_enum::mf_CW0FLIP :
        { my=-my; mz=-mz; }
        break;
      case Cfg::mag_align_enum::mf_CW90FLIP :
        { float tmp; tmp=mx; mx=my; my=tmp; mz=-mz; }
        break;
      case Cfg::mag_align_enum::mf_CW180FLIP :
        { mx=-mx; mz=-mz; }
        break;
      case Cfg::mag_align_enum::mf_CW270FLIP :
        { float tmp; tmp=mx; mx=-my; my=-tmp; mz=-mz; }
        break;
    }

    ts = micros();
  }

  runtimeTrace.stop(updated);
  return updated;
}
