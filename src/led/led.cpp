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

#include "led.h"
#include "../cfg/cfg.h"
#include "LedGizmoSingle.h"
#include "LedGizmoRgb.h"

int Led::setup() {
  cfg.printModule(MF_MOD);
  
  is_on = false;
  last_color = 0xffffff; //white

  //create gizmo
  delete gizmo;
  switch(config.gizmo) {
    case Cfg::led_gizmo_enum::mf_NONE :
      gizmo = nullptr;
      break;
    case Cfg::led_gizmo_enum::mf_HIGH_IS_ON :
      gizmo = new LedGizmoSingle(config.pin, 1);
      break;
    case Cfg::led_gizmo_enum::mf_LOW_IS_ON :
      gizmo = new LedGizmoSingle(config.pin, 0);
      break;
    case Cfg::led_gizmo_enum::mf_RGB :
      gizmo = new LedGizmoRgb(config.pin);
      break;
  }

  return 0;
}

void Led::color(uint32_t rgb) {
  if(!gizmo || !enabled) return;
  if(rgb) {
    last_color = rgb;
    is_on = true;
  }else{
    is_on = false;
  }
  gizmo->color(rgb);
}

inline void Led::set(bool set_on) {
  if(set_on) {
    color(last_color);
  }else{
    color(0);
  }
}

void Led::toggle() {
  set(!is_on);
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
