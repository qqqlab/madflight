#define MF_MOD "LED"

#include <Arduino.h> //Serial
#include "led.h"
#include "../cfg/cfg.h"
#include "LedGizmoSingle.h"

int Led::setup() {
  Cfg::printModule(MF_MOD);

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
