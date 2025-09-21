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

//https://uglyduck.vajn.icu/ep/archive/2014/01/Making_a_better_HC_SR04_Echo_Locator.html

// HC-SR04 driver (single instance only)
// range: 2-6m (depending on variant)
// reporting rate: 100ms
// trigger pin high pulse >= 8us --> echo high pulse 

#pragma once

#include "rdr.h"
#include "../hal/hal.h"
#include <Arduino.h> //micros, pinMode, digitalWrite, attachInterrupt

#define SR04_TRIG_US 100      // minimum length of trigger pulse
#define SR04_TIMEOUT_US 60000 // echo timeout 60 ms ~= 10 meter, this is also the appoximate sample rate

static uint8_t _RdrGizmoSR04_irq_pin_echo = 0;
static volatile uint32_t _RdrGizmoSR04_irq_echo_ts1 = 0;
static volatile uint32_t _RdrGizmoSR04_irq_echo_ts2 = 0;

static void _RdrGizmoSR04_irq() {
  if(digitalRead(_RdrGizmoSR04_irq_pin_echo) == HIGH) {
    _RdrGizmoSR04_irq_echo_ts1 = micros();
  }else{
    _RdrGizmoSR04_irq_echo_ts2 = micros();
  }
}


class RdrGizmoSR04: public RdrGizmo {
protected:
  enum class state_enum { TRIG_START, TRIG_END, ECHO_WAIT, INTERVAL_WAIT};
  state_enum state = state_enum::TRIG_START;
  uint8_t pin_trig = 0;
  uint32_t state_ts = 0;
  int *dist = nullptr;

  RdrGizmoSR04() {} //private constructor

public:
  static RdrGizmoSR04* create(int* dist, int pin_trig, int pin_echo) {
      if(!dist || pin_trig < 0 || pin_echo < 0) return nullptr;
      pinMode(pin_trig, OUTPUT);
      digitalWrite(pin_trig, LOW);
      pinMode(pin_echo, INPUT);
      attachInterrupt(digitalPinToInterrupt(pin_echo), _RdrGizmoSR04_irq, CHANGE);

      //setup gizmo
      auto gizmo = new RdrGizmoSR04();
      _RdrGizmoSR04_irq_pin_echo = pin_echo;
      gizmo->pin_trig = pin_trig;
      gizmo->dist = dist;
      return gizmo;
    }

  bool update() override {
    switch(state) {
      case state_enum::TRIG_START:
        //note: send trigger pulse regardless of echo pin state, is this wise???
        digitalWrite(pin_trig, HIGH);
        state_ts = micros();
        state = state_enum::TRIG_END;
        break;
      case state_enum::TRIG_END:
        if(micros() - state_ts > SR04_TRIG_US) {
          _RdrGizmoSR04_irq_echo_ts1 = 0;
          _RdrGizmoSR04_irq_echo_ts2 = 0;
          digitalWrite(pin_trig, LOW);
          state_ts = micros();
          state = state_enum::ECHO_WAIT;
        }
        break;
      case state_enum::ECHO_WAIT:
        if(_RdrGizmoSR04_irq_echo_ts2 && _RdrGizmoSR04_irq_echo_ts1) {
          //got echo
          *dist = (_RdrGizmoSR04_irq_echo_ts2 - _RdrGizmoSR04_irq_echo_ts1) * 171 / 1000; // integer math echo_us -> dist_mm with speed of sound 342 m/s
          state = state_enum::INTERVAL_WAIT;
          //Serial.printf("ECHO %d\n",*dist);
          return true;
        }
        //fall thru
      case state_enum::INTERVAL_WAIT:
        if(micros() - state_ts > SR04_TIMEOUT_US) {
          //if(state == state_enum::ECHO_WAIT) Serial.printf("SR04 TIMEOUT\n");
          state = state_enum::TRIG_START;
        }
        break;
    }
    return false;
  }

};