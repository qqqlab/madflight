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

// PMW3901 SPI driver

#pragma once

#include "ofl.h"
#include "../hal/hal.h"
#include "PWM3901/Bitcraze_PMW3901.h"

class OflGizmoPMW3901: public OflGizmo {
private:
  OflState *state;
  Bitcraze_PMW3901 *sensor = nullptr;

  OflGizmoPMW3901() {} //private constructor

public:
  ~OflGizmoPMW3901() {
    delete sensor;
  }

  static OflGizmoPMW3901* create(OflConfig *config, OflState *state) {
      //get spi bus
      if(config->pin_ofl_cs < 0) {
        Serial.println("OFL: ERROR PMW3901 pin_ofl_cs not defined");
        return nullptr;
      }
      SPIClass *spi_bus = hal_get_spi_bus(config->ofl_spi_bus);
      if(!spi_bus) {
        Serial.println("OFL: ERROR PMW3901 spi_bus_id not defined");
        return nullptr;
      }

      //try to start sensor
      Bitcraze_PMW3901 *sensor = new Bitcraze_PMW3901(spi_bus, config->pin_ofl_cs);
      if(!sensor->begin()) {
        Serial.println("OFL: ERROR PMW3901 wai incorrect, check wiring");
        delete sensor;
        sensor = nullptr;
        return nullptr;
      }

      //setup gizmo
      auto gizmo = new OflGizmoPMW3901();
      gizmo->state = state;
      gizmo->sensor = sensor;
      state->ts = micros();
      return gizmo;
    }

  bool update() override {
    uint32_t now = micros();

    // update rate approx 6ms (166Hz) but no status flag/interrupt to know when a measurement took place. Sensor returns 0,0 when busy, but 0,0 could also mean "new measurement, but no movement".
    // --> so, 8000 us after the last measurement we should have at least 1 new measurement
    if(now - state->ts < 8000) return false;

    int16_t dx;
    int16_t dy;
    sensor->readMotionCount(&dx, &dy);

    state->dx = dx;
    state->dy = dy;
    state->dt = now - state->ts;
    state->ts = now;

    return true;
  }
};