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

// HKL-LD2413 driver
// range: 10m
// reporting rate: 50-1000ms, 160ms default
// default baud rate 115200
// reported data: F4 F3 F2 F1 04 00 xx xx xx xx F8 F7 F6 F5 
// where xx xx xx xx is a LE float with distance in mm

#pragma once

#include "rdr.h"
#include "../hal/MF_Serial.h"
#include "../hal/hal.h"

class RdrGizmoLD2413: public RdrGizmo {
private:
  int *dist = nullptr;
  MF_Serial* ser_bus = nullptr;
  
  int pos = 0;
  union{
    uint8_t b[4];
    float f;
  } data;

  RdrGizmoLD2413() {} //private constructor

public:
  static RdrGizmoLD2413* create(int* dist, int ser_bus_id, int baud) {
      //get serial bus
      if(baud == 0) baud = 115200;
      MF_Serial* ser_bus = hal_get_ser_bus(ser_bus_id, baud);
      if(!ser_bus) return nullptr;

      //setup gizmo
      auto gizmo = new RdrGizmoLD2413();
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
          if(b==0xF4) pos++; else pos=0;
          break;
        case 1:
          if(b==0xF3) pos++; else pos=0;
          break;
        case 2:
          if(b==0xF2) pos++; else pos=0;
          break;
        case 3:
          if(b==0xF1) pos++; else pos=0;
          break;
        case 4:
          if(b==0x04) pos++; else pos=0;
          break;
        case 5:
          if(b==0x00) pos++; else pos=0;
          break;
        case 6:
        case 7:
        case 8:
        case 9:
          data.b[pos-6] = b;
          pos++;
          break;
        case 10:
          if(b==0xF8) pos++; else pos=0;
          break;
        case 11:
          if(b==0xF7) pos++; else pos=0;
          break;
        case 12:
          if(b==0xF6) pos++; else pos=0;
          break;
        case 13:
          if(b==0xF5) {
            *dist = data.f;
            //Serial.println(data.f);
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