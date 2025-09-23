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

// PMW3901U UART driver

/*
PMW3901U Data Format

byte0: header (0xFE)
byte1: number of data bytes (0x04)
byte2: x-motion high byte
byte3: x-motion low byte
byte4: y-motion high byte
byte5: y-motion low byte
byte6: checksum
byte7: surface quality
byte8: footer (0xAA)
*/

#pragma once

#include "ofl.h"
#include "../hal/hal.h"

class OflGizmoPMW3901U: public OflGizmo {
private:
  OflState *state;
  MF_Serial *ser_bus;
  uint8_t _parserState = 0;

  OflGizmoPMW3901U() {} //private constructor

public:
  ~OflGizmoPMW3901U() {}

  static OflGizmoPMW3901U* create(OflConfig *config, OflState *state) {
      //get serial bus
      if(config->ofl_ser_bus<0) {
        Serial.println("OFL: ERROR PMW3901U ofl_ser_bus not configured");
        return nullptr;
      }
      MF_Serial *ser_bus = hal_get_ser_bus(config->ofl_ser_bus);
      if(!ser_bus) {
        Serial.printf("OFL: ERROR PMW3901U pin_ser%d_tx not configured\n", config->ofl_ser_bus);
        return nullptr;
      }
      int baud = config->ofl_baud;
      if(baud<=0) baud = 115200;
      ser_bus->begin(baud);

      //setup gizmo
      auto gizmo = new OflGizmoPMW3901U();
      gizmo->state = state;
      gizmo->ser_bus = ser_bus;
      gizmo->_parserState = 0;
      state->ts = micros();
      return gizmo;
    }

  bool update() override {
    static int16_t dx = 0;
    static int16_t dy = 0;

    int dx_sum = 0;
    int dy_sum = 0;

    uint32_t now = micros();
    bool got_data = false;
    uint8_t b;
    int n = ser_bus->available();
    for(int i = 0; i < n; i++) { //only process current bytes, not bytes received while processing!
      ser_bus->read(&b, 1);

      //Serial.printf("%02X\n", b);

      switch(_parserState) {
        case 0: //byte0: header (0xFE)
          if (b == 0xFE) {
            _parserState++; 
          }
          break;
        case 1: //byte1: number of data bytes (0x04)
          if (b == 0x04) {
            _parserState++; 
          }else{
            _parserState = 0;
          }
          break;
        case 2: //byte2: x-motion high byte
          dx = b << 8;
          _parserState++;
          break;
        case 3: //byte3: x-motion low byte
          dx |= b;
          _parserState++;
          break;
        case 4: //byte4: y-motion high byte
          dy = b << 8;
          _parserState++;
          break;
        case 5: //byte5: y-motion low byte
          dy |= b;
          _parserState++;
          break;
        case 6: //byte6: checksum - ignored
          _parserState++;
          break;
        case 7: //byte7: surface quality - ignored
          _parserState++;
          break;
        case 8: //byte8: footer (0xAA)
          if (b == 0xAA) {
            dx_sum += dx;
            dy_sum += dy;
            got_data = true;
          }
          _parserState = 0;
          break;
        default: //should not get hereby
          _parserState = 0;
          break;
      }
    }

    if(!got_data) return false;

    state->dx_raw = dx_sum;
    state->dy_raw = dy_sum;
    state->dt = now - state->ts;
    state->ts = now;

    return true;
  }
};