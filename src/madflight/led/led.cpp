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

#define MF_MOD "LED"

#include <Arduino.h> //Serial
#include "led.h"
#include "../cfg/cfg.h"
#include "LedGizmoSingle.h"

int Led::setup() {
  cfg.printModule(MF_MOD);

  //create gizmo
  delete gizmo;
  if(config.pin >= 0) {
    gizmo = new LedGizmoSingle(config.pin, config.on_value);
  }else{
    gizmo = nullptr;
  }

  return 0;
}

inline void Led::set(bool set_on) {
  if(!gizmo || !enabled) return;
  gizmo->set(set_on);
}

void Led::toggle() {
  if(!gizmo || !enabled) return;
  gizmo->toggle();
}

void Led::on() {
  set(true);
}

void Led::off() {
  set(false);
}

void Led::blink(int times) {
  for(int i=0;i<times;i++) {
    off();
    delay(500);
    on();
    delay(500);
  }
  off();
  delay(2000);
}

//Global module instance
Led led;
