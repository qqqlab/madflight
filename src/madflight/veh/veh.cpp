//=================================================================================================
// VEH - Vehicle info
//=================================================================================================

#include "veh.h"
#include "../bbx/bbx_interface.h"

//global module class instance
Veh veh;

void Veh::setFlightmode(uint8_t flightmode) {
  if(_flightmode != flightmode) {
    _flightmode = flightmode;
    bbx.log_mode(_flightmode, flightmode_name());
  }
}

uint8_t Veh::getFlightmode() {
  return _flightmode;
}

uint8_t Veh::flightmode_ap_id() {
  if(_flightmode<6) return flightmode_ap_ids[_flightmode];
  return 0xff;
}

const char* Veh::flightmode_name() {
  if(_flightmode<6) return flightmode_names[_flightmode];
  return flightmode_name_unknown;
}
