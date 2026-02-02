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

#define MF_MOD "BAR"

#include <Arduino.h> //Serial
#include "bar.h"
#include "BarGizmoBMP280.h"
#include "BarGizmoBMP390.h"
#include "BarGizmoMS5611.h"
#include "BarGizmoHP203B.h"
#include "BarGizmoBMP580.h"
#include "BarGizmoDPS310.h"
#include <math.h>

//create global module instance
Bar bar;


int Bar::setup() {
  cfg.printModule(MF_MOD);

  //clear state
  press = 0;
  temp = 0;
  alt = 0;
  ts = 0;
  dt = 0;

  //create gizmo
  delete gizmo;
  switch(config.gizmo) {
    case Cfg::bar_gizmo_enum::mf_NONE :
      gizmo = nullptr;
      break;
    case Cfg::bar_gizmo_enum::mf_BMP280 :
      if(config.i2c_bus) {
        gizmo = new BarGizmoBMP280(config.i2c_bus, config.i2c_adr, config.sample_rate);
      }
      break;
    case Cfg::bar_gizmo_enum::mf_BMP388 :
    case Cfg::bar_gizmo_enum::mf_BMP390 :
      if(config.i2c_bus) {
        gizmo = new BarGizmoBMP390(config.i2c_bus, config.i2c_adr, config.sample_rate);
      }
      break;
    case Cfg::bar_gizmo_enum::mf_MS5611 :
      if(config.i2c_bus) {
        gizmo = new BarGizmoMS5611(config.i2c_bus, config.i2c_adr, config.sample_rate);
      }
      break;
    case Cfg::bar_gizmo_enum::mf_HP203B :
      if(config.i2c_bus) {
        gizmo = new BarGizmoHP203B(config.i2c_bus, config.i2c_adr, config.sample_rate);
      }
      break;
    case Cfg::bar_gizmo_enum::mf_BMP580 :
      if(config.i2c_bus) {
        //gizmo = new BarGizmoBMP580(config.i2c_bus, config.i2c_adr, config.sample_rate);
        gizmo = BarGizmoBMP580::create(&config, this);
      }
      break;
    case Cfg::bar_gizmo_enum::mf_DPS310 :
      if(config.i2c_bus) {
        gizmo = BarGizmoDPS310::create(config.i2c_bus, config.i2c_adr, config.sample_rate);
      }
      break;
  }

  //check gizmo
  if(!installed() && config.gizmo != Cfg::bar_gizmo_enum::mf_NONE) {
    Serial.println("\n" MF_MOD ": ERROR check pin/bus config\n");
    return -1001;
  }

  return 0;
}

bool Bar::update() {
  runtimeTrace.start();
  bool updated = (gizmo != nullptr);
  updated = updated && gizmo->update(&press, &temp);

  if(updated) {
    float P = press;
    //float T = temp;
    //alt = 153.84348f * (1 - pow(P / 101325.0f, 0.19029496f)) * (T + 273.15f); //hypsometric formula - reduces to barometric with T=15C
    alt = 44330.0f * (1 - pow(P / 101325.0f, 0.19029496f)); //barometric formula  0.19029496 = 1/5.255
    //alt = (101325.0f - P) / 12.0f; //linearisation of barometric formula at sealevel
    uint32_t now = micros();
    dt = (now - ts) / 1000000.0;
    ts = now;

    topic.publish(this);
  }

  runtimeTrace.stop(updated);
  return updated;
}
