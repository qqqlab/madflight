#pragma once

#include "led.h"

class LedGizmoSingle : public LedGizmo {
  public:
    LedGizmoSingle(int pin, uint8_t led_on_value) {
      this->pin = pin;
      this->led_on_value = led_on_value;
      pinMode(pin, OUTPUT);
      set(false);
    }

    void set(bool set_on) override {
      state = set_on;
      digitalWrite( pin, (set_on ? led_on_value : !led_on_value) );
    }

    void toggle() override {
      set(!state);
    }

private:
    int pin = -1;
    uint8_t led_on_value = 0;
    bool state = false;
};
