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
#include "DTS6012M/DTS6012M_UART.h"

class RdrGizmoDTS6012M: public RdrGizmo {
private:
  int *dist = nullptr;
  DTS6012M_UART sensor;

  RdrGizmoDTS6012M() {} //private constructor

public:
  static RdrGizmoDTS6012M* create(int* dist, int ser_bus_id, int baud) {
      //get serial bus
      if(baud == 0) baud = 921600; //baud rate is fixed (i.e. need this baud rate to change baud rate?)
      MF_Serial* ser_bus = hal_get_ser_bus(ser_bus_id, baud);
      if(!ser_bus) return nullptr;

      //setup gizmo
      auto gizmo = new RdrGizmoDTS6012M();
      gizmo->dist = dist;
      if(!gizmo->sensor.begin(ser_bus, baud)) {
        Serial.println("RDR: ERROR: DTS6012M init failed.");
      }
      return gizmo;
    }

  bool update() override {
    if(!sensor.update()) return false;
    *dist = sensor.getDistance(); // in mm
    return true;
  }
};