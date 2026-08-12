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

//=================================================================================================
// VEH - Vehicle info
//=================================================================================================

#include "veh.h"
#include "../bbx/bbx.h"

//global module class instance
Veh veh;

//returns true if flightmode changed
bool Veh::set_flightmode(FlightMode fm) {
  if(_flightmode == fm) return false;
  FlightMode pid_fm = pid.set_flightmode(fm); //set pid controller flight mode, which might not support all flight modes
  if(_flightmode == pid_fm) return false;
  _flightmode = pid_fm;
  topic.publish(this);
  bbx.log_mode();
  return true;
}

FlightMode Veh::get_flightmode() {
  return _flightmode;
}

uint8_t Veh::flightmode_ap_id() {
  if((int)_flightmode >= MF_FLIGHTMODE_COUNT || (int)_flightmode < 0) return 255;
  return flightmode_map[(int)_flightmode].ap_fm;
}

const char* Veh::flightmode_name() {
  if((int)_flightmode >= MF_FLIGHTMODE_COUNT || (int)_flightmode < 0) return flightmode_name_unknown;
  return flightmode_map[(int)_flightmode].name;
}
