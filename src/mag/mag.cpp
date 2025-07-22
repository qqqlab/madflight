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

#define MF_MOD "MAG"

#include <Arduino.h> //Serial
#include "mag.h"
#include "MagGizmoQMC5883L.h"
#include "MagGizmoQMC6309.h"
#include "MagGizmoRM3100.h"

//create global module instance
Mag mag;

int Mag::setup() {
  cfg.printModule(MF_MOD);

  _samplePeriod = 1000000 / config.sampleRate;

  //clear state
   x = 0; //"North" magnetic flux [uT]
   y = 0; //"East" magnetic flux [uT]
   z = 0; //"Down" magnetic flux [uT]
   ts = 0; //last sample time in [us]

  //create gizmo
  delete gizmo;
  switch(config.gizmo) {
    case Cfg::mag_gizmo_enum::mf_NONE :
      gizmo = nullptr;
      break;
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
  }

  //check gizmo
  if(!installed() && config.gizmo != Cfg::mag_gizmo_enum::mf_NONE) {
    Serial.println("\n" MF_MOD ": ERROR check pin/bus config\n");
    return -1001;
  }

  return 0;
}

bool Mag::update() {
  if(!gizmo) return false;

  //wait for next sample interval
  if(!schedule.interval(_samplePeriod)) return false;

  if(!gizmo->update(&x, &y, &z)) return false;

  //handle rotation for different mounting positions
  switch((Cfg::mag_align_enum)cfg.mag_align) {
    case Cfg::mag_align_enum::mf_CW0 :
      break;
    case Cfg::mag_align_enum::mf_CW90 :
      { float tmp; tmp=x; x=-y; y=tmp; }
      break;
    case Cfg::mag_align_enum::mf_CW180 :
      { x=-x; y=-y; }
      break;
    case Cfg::mag_align_enum::mf_CW270 :
      { float tmp; tmp=x; x=y; y=-tmp; }
      break;
    case Cfg::mag_align_enum::mf_CW0FLIP :
      { y=-y; z=-z; }
      break;
    case Cfg::mag_align_enum::mf_CW90FLIP :
      { float tmp; tmp=x; x=y; y=tmp; z=-z; }
      break;
    case Cfg::mag_align_enum::mf_CW180FLIP :
      { x=-x; z=-z; }
      break;
    case Cfg::mag_align_enum::mf_CW270FLIP :
      { float tmp; tmp=x; x=-y; y=-tmp; z=-z; }
      break;
  }

  ts = micros();
  return true;
}
