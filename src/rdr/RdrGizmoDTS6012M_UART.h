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

/* Gizmo for DTS6012M

Max Range: 20 meter
Min Range: 0 mm, reported as 100 mm (sensor has approx 100mm offset, i.e. target at 400mm is reported as 500mm)
Resolution: 1 mm
Rate: 100 measurements / second (max 250)
Returns -1 mm on bad measurement (sensor blocked)

*/

#pragma once

#include "rdr.h"
#include "../hal/hal.h"
#include "DTS6012M_UART/DTS6012M_UART.h"

class RdrGizmoDTS6012M_UART: public RdrGizmo {
private:
  RdrState *state;
  DTS6012M_UART *dts6012m_uart = nullptr;

  RdrGizmoDTS6012M_UART() {} //private constructor

public:
  static RdrGizmoDTS6012M_UART* create(RdrConfig *config, RdrState *state) {
      //get serial bus
      if(config->rdr_baud == 0) config->rdr_baud = 921600; //baud rate is fixed (i.e. need this baud rate to change baud rate?)
      MF_Serial* ser_bus = hal_get_ser_bus(config->rdr_ser_bus, config->rdr_baud);
      if(!ser_bus) return nullptr;

      //setup gizmo
      auto gizmo = new RdrGizmoDTS6012M_UART();
      gizmo->state = state;
      gizmo->dts6012m_uart = new DTS6012M_UART();
      if(!gizmo->dts6012m_uart->begin(ser_bus, config->rdr_baud)) {
        Serial.println("RDR: ERROR: DTS6012M_UART init failed.");
        delete gizmo->dts6012m_uart;
        delete gizmo;
        return nullptr;
      }
      return gizmo;
    }

  bool update() override {
    if(!dts6012m_uart->update()) return false;
    //sensor reports in [mm], or returns -1 on fail
    int16_t dist = dts6012m_uart->getDistance();
    if(dist == -1) {
      state->dist = -1; //no data
    }else if(dist < -1 || dist > 20000) {
      state->dist = -2; //invalid data
    }else{
      state->dist = (float)dist * 0.001f; //dist in [m]
    }
    return true;
  }
};