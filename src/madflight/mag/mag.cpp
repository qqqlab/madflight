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

//create global module instance
Mag mag;

int Mag::setup() {
  cfg.printModule(MF_MOD);

  _samplePeriod = 1000000 / config.sampleRate;

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
  if(micros() - mag_time < _samplePeriod) return false;
  mag_time = micros();
  return gizmo->read_uT(&x, &y, &z);
}
