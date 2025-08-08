#pragma once

#include "led.h"
#include "Adafruit_NeoPixel/Adafruit_NeoPixel.h"

class LedGizmoRgb : public LedGizmo {
  public:
    LedGizmoRgb(int pin) {
      pixels = new Adafruit_NeoPixel(1, pin, NEO_GRB + NEO_KHZ800);
      pixels->begin();
    }

    void color(uint32_t rgb) override {
      pixels->setPixelColor(0, rgb);
      pixels->show();
    }

private:
    Adafruit_NeoPixel *pixels = nullptr;
};
