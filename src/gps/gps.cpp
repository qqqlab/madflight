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

#define MF_MOD "GPS"

#include <Arduino.h> //Serial
#include "gps.h"

//the gizmos
#include "GpsGizmoUblox.h"
//#include "GpsGizmoNmea.h" //TODO

//create global module instance
Gps gps;

int Gps::setup() {
  cfg.printModule(MF_MOD);

  //create gizmo
  delete gizmo;
  switch(config.gizmo) {
    case Cfg::gps_gizmo_enum::mf_NONE :
      gizmo = nullptr;
      break;
    case Cfg::gps_gizmo_enum::mf_UBLOX :
      gizmo = GpsGizmoUblox::create((GpsState*)this, config.ser_bus_id, config.baud);
      break;
  }

  //check gizmo
  if(!installed() && config.gizmo != Cfg::gps_gizmo_enum::mf_NONE) {
    cfg.printModule(MF_MOD, CfgClass::printModuleMode::CFG_ERROR);
    return -1001;
  }

  return 0;
}

bool Gps::update() {
  runtimeTrace.start();
  bool updated = (gizmo != nullptr);
  updated = updated && gizmo->update();

  if(updated) topic.publish(this);
  runtimeTrace.stop(updated);
  return updated;
}

