//=================================================================================================
// VEH - Vehicle info
//=================================================================================================

#pragma once

#include "veh_interface.h"
#include "../bb/bb_interface.h"

void Vehicle::setFlightmode(uint8_t flightmode) {
  if(_flightmode != flightmode) {
    _flightmode = flightmode;
    bb.log_mode(_flightmode, flightmode_name());
  }
}

uint8_t Vehicle::getFlightmode() {
  return _flightmode;
}

uint8_t Vehicle::flightmode_ap_id() {
  if(_flightmode<6) return flightmode_ap_ids[_flightmode];
  return 0xff;
}

const char* Vehicle::flightmode_name() {
  if(_flightmode<6) return flightmode_names[_flightmode];
  return flightmode_name_unknown;
}

Vehicle veh;
