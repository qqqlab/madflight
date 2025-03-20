#define MF_MOD "MAG"

#include <Arduino.h> //Serial
#include "mag.h"
#include "MagGizmoQMC5883L.h"

//create global module instance
Mag mag;

int Mag::setup() {
  Cfg::printModule(MF_MOD);

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
