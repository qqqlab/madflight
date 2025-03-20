#define MF_MOD "BAT"

#include <Arduino.h> //Serial
#include "bat.h"
#include "BatGizmoADC.h"
#include "BatGizmoINA226.h"
#include "BatGizmoINA228.h"

//create global module instance
Bat bat;

int Bat::setup() {
  Cfg::printModule(MF_MOD);

  //create gizmo
  delete gizmo;
  switch(config.gizmo) {
    case Cfg::bat_gizmo_enum::mf_NONE :
      gizmo = nullptr;
      break;
    case Cfg::bat_gizmo_enum::mf_ADC :
      gizmo = new BatGizmoADC(this);
      break;
    case Cfg::bat_gizmo_enum::mf_INA226 :
      if(config.i2c_bus) {
        gizmo = new BatGizmoINA226((BatState*)this, config.i2c_bus, config.i2c_adr, config.rshunt);
      }
      break;
    case Cfg::bat_gizmo_enum::mf_INA228 :
      if(config.i2c_bus) {
        gizmo = new BatGizmoINA228((BatState*)this, config.i2c_bus, config.i2c_adr, config.rshunt);
      }
      break;
  }

  //check gizmo
  if(!installed() && config.gizmo != Cfg::bat_gizmo_enum::mf_NONE) {
    Serial.println("\n" MF_MOD ": ERROR check pin/bus config\n");
    return -1001;
  }

  return 0;
}

bool Bat::update() {
  if(!gizmo) return false;
  return gizmo->update();
}

