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

#define MF_MOD "RDR"

#include <Arduino.h> //Serial
#include "rdr.h"

//the gizmos
#include "RdrGizmoLD2411S.h"
#include "RdrGizmoLD2413.h"
#include "RdrGizmoUSD1.h"
#include "RdrGizmoSR04.h"
#include "RdrGizmoDTS6012M_UART.h"
#include "RdrGizmoDTS6012M_I2C.h"

//create global module instance
Rdr rdr;

int Rdr::setup() {
  cfg.printModule(MF_MOD);

  //create gizmo
  delete gizmo;
  switch(config.gizmo) {
    case Cfg::rdr_gizmo_enum::mf_NONE :
      gizmo = nullptr;
      break;
    case Cfg::rdr_gizmo_enum::mf_LD2411S :
      gizmo = RdrGizmoLD2411S::create(&dist, config.rdr_ser_bus, config.rdr_baud);
      break;
    case Cfg::rdr_gizmo_enum::mf_LD2413 :
      gizmo = RdrGizmoLD2413::create(&dist, config.rdr_ser_bus, config.rdr_baud);
      break;
    case Cfg::rdr_gizmo_enum::mf_USD1 :
      gizmo = RdrGizmoUSD1::create(&dist, config.rdr_ser_bus, config.rdr_baud);
      break;
    case Cfg::rdr_gizmo_enum::mf_SR04 :
      gizmo = RdrGizmoSR04::create(&dist, config.pin_rdr_trig, config.pin_rdr_echo);
      break;
    case Cfg::rdr_gizmo_enum::mf_DTS6012M :
      if(config.rdr_ser_bus >= 0) {
        gizmo = RdrGizmoDTS6012M_UART::create(&dist, &config);
      }else{
        gizmo = RdrGizmoDTS6012M_I2C::create(&dist, &config);
      }
      break;
  }

  //check gizmo
  if(!installed() && config.gizmo != Cfg::rdr_gizmo_enum::mf_NONE) {
    Serial.println("\n" MF_MOD ": ERROR check pin/bus config\n");
    return -1001;
  }

  return 0;
}

bool Rdr::update() {
  if(!gizmo) return false;
  if(!gizmo->update()) return false;
  update_ts = micros();
  update_cnt++;
  return true;
}

