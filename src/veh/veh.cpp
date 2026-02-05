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

//global module class instance and topic
Veh veh;
MsgTopic<VehState> veh_topic = MsgTopic<VehState>("veh");

//returns true if flightmode changed
bool Veh::setFlightmode(uint8_t flightmode) {
  if(_flightmode == flightmode) return false;
  _flightmode = flightmode;
  veh_topic.publish(this);
  bbx.log_mode();
  return true;
}

uint8_t Veh::getFlightmode() {
  return _flightmode;
}

uint8_t Veh::flightmode_ap_id() {
  if(_flightmode<6) return flightmode_ap_ids[_flightmode];
  return _flightmode;
}

const char* Veh::flightmode_name() {
  if(_flightmode<6) return flightmode_names[_flightmode];
  return flightmode_name_unknown;
}
