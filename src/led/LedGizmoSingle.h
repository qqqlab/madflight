#pragma once

#include "led.h"

class LedGizmoSingle : public LedGizmo {
  public:
    LedGizmoSingle(int pin, uint8_t led_on_value) {
      this->pin = pin;
      this->led_on_value = led_on_value;
      pinMode(pin, OUTPUT);
      color(0); //switch off
    }

    void color(uint32_t rgb) override {
      if(rgb) {
        digitalWrite( pin, led_on_value);
      }else{
        digitalWrite( pin, !led_on_value);
      }
    }

private:
    int pin = -1;
    uint8_t led_on_value = 0;
};
