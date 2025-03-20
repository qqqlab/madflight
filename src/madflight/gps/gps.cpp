#define MF_MOD "GPS"

#include <Arduino.h> //Serial
#include "gps.h"

//the gizmos
#include "GpsGizmoUblox.h"
//#include "GpsGizmoNmea.h" //TODO

//create global module instance
Gps gps;

int Gps::setup() {
  Cfg::printModule(MF_MOD);

  //create gizmo
  delete gizmo;
  switch(config.gizmo) {
    case Cfg::gps_gizmo_enum::mf_NONE :
      gizmo = nullptr;
      break;
    case Cfg::gps_gizmo_enum::mf_UBLOX :
      gizmo = new GpsGizmoUblox((GpsState*)this, config.ser_bus, config.baud);
      break;
  }

  //check gizmo
  if(!installed() && config.gizmo != Cfg::gps_gizmo_enum::mf_NONE) {
    Serial.println("\n" MF_MOD ": ERROR check pin/bus config\n");
    return -1001;
  }

  return 0;
}

bool Gps::update() {
  if(!gizmo) return false;
  return gizmo->update();
}

