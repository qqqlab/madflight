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

// Ainstein US-D1 Radar Altimeter / uLandings AG-50 driver
// range: 50m
// reporting rate: 12ms
// default baud rate 115200
/*
 reported data: FE vv xx xx snr crc
 where:
   vv: version: 00=AG-50, 02=US-D1
   xx xx: LE uint16 with distance in cm
   snr: SNR on US-D!, 00 on AG-50
   crc: The Checksum Byte = Version_ID + Altitude_H + Altitude_L + SNR

*/
#pragma once

#include "rdr.h"
#include "../hal/MF_Serial.h"
#include "../hal/hal.h"

class RdrGizmoUSD1: public RdrGizmo {
private:
  RdrState *state;
  MF_Serial* ser_bus = nullptr;
  
  int pos = 0;
  uint8_t crc;
  union{
    uint8_t b[2];
    uint16_t val;
  } data;

  RdrGizmoUSD1() {} //private constructor

public:
  static RdrGizmoUSD1* create(RdrConfig *config, RdrState *state) {
      //get serial bus
      if(config->rdr_baud == 0) config->rdr_baud = 115200;
      MF_Serial* ser_bus = hal_get_ser_bus(config->rdr_ser_bus, config->rdr_baud);
      if(!ser_bus) return nullptr;

      //setup gizmo
      auto gizmo = new RdrGizmoUSD1();
      gizmo->ser_bus = ser_bus;
      gizmo->state = state;
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
          crc = 0;
          if(b==0xFE) pos++; else pos=0;
          break;
        case 1: //version
          crc += b;
          pos++;
          break;
        case 2: //distance
        case 3:
          data.b[pos-2] = b;
          crc += b;
          pos++;
          break;
        case 4: //snr or 0
          crc += b;
          pos++;
          break;
        case 5: //crc
          if(b == crc) {
            state->dist = data.val * 0.01f; //sensor reports distance in [cm]
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