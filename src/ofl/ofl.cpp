/*==========================================================================================
MIT License

Copyright (c) 2025 https://madflight.com

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

#define MF_MOD "OFL"

#include <Arduino.h> //Serial
#include "ofl.h"

//the gizmos
#include "OflGizmoPMW3901.h" //SPI
#include "OflGizmoPMW3901U.h" //UART

//create global module instance
Ofl ofl;

int Ofl::setup() {
  cfg.printModule(MF_MOD);

  //create gizmo
  delete gizmo;
  switch(config.ofl_gizmo) {
    case Cfg::ofl_gizmo_enum::mf_NONE :
      gizmo = nullptr;
      break;
    case Cfg::ofl_gizmo_enum::mf_PMW3901 :
      gizmo = OflGizmoPMW3901::create(&config, this);
      break;
    case Cfg::ofl_gizmo_enum::mf_PMW3901U :
      gizmo = OflGizmoPMW3901U::create(&config, this);
      break;
  }

  //check gizmo
  if(!installed() && config.ofl_gizmo != Cfg::ofl_gizmo_enum::mf_NONE) {
    Serial.println("\n" MF_MOD ": ERROR check pin/bus config\n");
    return -1001;
  }

  return 0;
}

bool Ofl::update() {
  if(!gizmo) return false;
  if(!gizmo->update()) return false;

  //rotate sensor axis to NE(D) frame
  switch(config.ofl_align) {
    case Cfg::ofl_align_enum::mf_NW:
      dx = dx_raw;
      dy = -dy_raw;
      break;
    case Cfg::ofl_align_enum::mf_SW:
      dx = -dx_raw;
      dy = -dy_raw;
      break;
    case Cfg::ofl_align_enum::mf_SE:
      dx = -dx_raw;
      dy = dy_raw;
      break;
    case Cfg::ofl_align_enum::mf_ES:
      dx = -dy_raw;
      dy = dx_raw;
      break;
    case Cfg::ofl_align_enum::mf_EN:
      dx = dy_raw;
      dy = dx_raw;
      break;
    case Cfg::ofl_align_enum::mf_WS:
      dx = -dy_raw;
      dy = -dx_raw;
      break;
    case Cfg::ofl_align_enum::mf_WN:
      dx = dy_raw;
      dy = -dx_raw;
      break;
    default:
    case Cfg::ofl_align_enum::mf_NE:
      dx = dx_raw;
      dy = dy_raw;
      break;
  }

  return true;
}

