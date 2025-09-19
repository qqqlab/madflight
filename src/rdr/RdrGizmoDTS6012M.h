#pragma once

#include "rdr.h"
#include "../hal/MF_Serial.h"
#include "DTS6012M/DTS6012M_UART.h"

class RdrGizmoDTS6012M: public RdrGizmo {
private:
  int *dist = nullptr;
  DTS6012M_UART sensor;
  

  RdrGizmoDTS6012M() {} //private constructor

public:
  static RdrGizmoDTS6012M* create(int* dist, int ser_bus_id, int baud) {
      //get serial bus
      if(baud == 0) baud = 115200;
      MF_Serial* ser_bus = hal_get_ser_bus(ser_bus_id, baud);
      if(!ser_bus) return nullptr;

      //setup gizmo
      auto gizmo = new RdrGizmoDTS6012M();
      gizmo->sensor.begin(ser_bus, baud);
      gizmo->dist = dist;
      return gizmo;
    }

  bool update() override {
    if(!sensor.update()) return false;
    *dist = sensor.getDistance(); // in mm
    return true;
  }
};