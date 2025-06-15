// HKL-LD2411S driver
// range: 6m
// reporting rate: 100ms
// default baud rate 256000
// reported data: AA AA tt xx xx 55 55
// where xx xx is a LE int16 with distance in cm
// and tt is 00=no data, 01="campaign target", 02="mircomotion target"

#pragma once

#include "rdr.h"
#include "../hal/MF_Serial.h"
#include "../hal/hal.h"

class RdrGizmoLD2411S: public RdrGizmo {
private:
  int *dist = nullptr;
  MF_Serial* ser_bus = nullptr;
  
  int pos = 0;
  union{
    uint8_t b[2];
    int16_t val;
  } data;

  RdrGizmoLD2411S() {} //private constructor

public:
  static RdrGizmoLD2411S* create(int* dist, int ser_bus_id, int baud) {
      //get serial bus
      if(baud == 0) baud = 256000;
      MF_Serial* ser_bus = hal_get_ser_bus(ser_bus_id, baud);
      if(!ser_bus) return nullptr;

      //setup gizmo
      auto gizmo = new RdrGizmoLD2411S();
      gizmo->ser_bus = ser_bus;
      gizmo->dist = dist;
      gizmo->pos = 0;
      return gizmo;
    }

  bool update() override {
    bool rv = false;
    int n = ser_bus->available();
    for(int i=0;i<n;i++) {
      uint8_t b;
      ser_bus->read(&b,1);
      switch(pos) {
        case 0:
          if(b==0xAA) pos++; else pos=0;
          break;
        case 1:
          if(b==0xAA) pos++; else pos=0;
          break;
        case 2:
          //00=no data, 01="campaign target", 02="mircomotion target"
          pos++;
          break;
        case 3:
        case 4:
          data.b[pos-3] = b;
          pos++;
          break;
        case 5:
          if(b==0x55) pos++; else pos=0;
          break;
        case 6:
          if(b==0x55) {
            *dist = data.val * 10;
            rv = true;
          }
          pos = 0;
          break;
        default:
          pos = 0;
      }
      //Serial.printf("%02X ",b);
    }
    return rv;
  }
};