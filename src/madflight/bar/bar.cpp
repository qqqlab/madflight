#define MF_MOD "BAR"

#include <Arduino.h> //Serial
#include "bar.h"
#include "BarGizmoBMP280.h"
#include "BarGizmoBMP390.h"
#include "BarGizmoMS5611.h"
#include <math.h>

//create global module instance
Bar bar;

int Bar::setup() {
  Cfg::printModule(MF_MOD);

  _samplePeriod = 1000000 / config.sampleRate;

  //clear state
  press = 0;
  temp = 0;
  alt = 0;
  ts = micros();
  dt = 0;  

  //create gizmo
  delete gizmo;
  switch(config.gizmo) {
    case Cfg::bar_gizmo_enum::mf_NONE :
      gizmo = nullptr;
      break;
    case Cfg::bar_gizmo_enum::mf_BMP280 :
      if(config.i2c_bus) {
        gizmo = new BarGizmoBMP280(config.i2c_bus, config.i2c_adr, config.sampleRate);
      }
      break;
    case Cfg::bar_gizmo_enum::mf_BMP388 :
    case Cfg::bar_gizmo_enum::mf_BMP390 :
      if(config.i2c_bus) {
        gizmo = new BarGizmoBMP390(config.i2c_bus, config.i2c_adr, config.sampleRate);
      }
      break;
    case Cfg::bar_gizmo_enum::mf_MS5611 :
      if(config.i2c_bus) {
        gizmo = new BarGizmoMS5611(config.i2c_bus, config.i2c_adr, config.sampleRate);
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
  if(!gizmo) return false;
  if (micros() - ts >= _samplePeriod) {
    uint32_t tsnew = micros();
    dt = (tsnew - ts) / 1000000.0;
    gizmo->update(&press, &temp);
    float P = press;
    //float T = temp;
    //alt = 153.84348f * (1 - pow(P/101325.0f, 0.19029496f)) * (T + 273.15f); //hypsometric formula - reduces to barometric with T=15C
    alt = 44330.0f * (1 - pow(P/101325.0f, 0.19029496f)); //barometric formula  0.19029496 = 1/5.255
    //alt = (101325.0f - P) / 12.0f; //linearisation of barometric formula at sealevel
    ts = tsnew;
    return true;
  }
  return false;
}
