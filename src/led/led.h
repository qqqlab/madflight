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

#pragma once

#include "../cfg/cfg.h"

struct LedConfig {
  public:
    Cfg::led_gizmo_enum gizmo = Cfg::led_gizmo_enum::mf_NONE; //the gizmo to use
    int pin = -1;
};

class LedGizmo {
  public:
    virtual ~LedGizmo() {}
    virtual void color(uint32_t rgb) = 0; //set color
};

class Led {
  public:
    LedConfig config;

    LedGizmo *gizmo = nullptr;

    int setup();      // Use config to setup gizmo, returns 0 on success, or error code
    bool installed() {return (gizmo != nullptr); } // Returns true if a gizmo was setup

    bool enabled = true; //enable changes to led state
    void color(uint32_t rgb); //set color
    void set(bool set_on);
    void on();
    void off();
    void toggle();
    void blink(int times); //blocking blink

  private:
    uint32_t last_color = 0xffffff; //white
    bool is_on = false;
};

extern Led led;
