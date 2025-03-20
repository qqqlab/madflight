#pragma once

#include "../cfg/cfg.h"

struct LedConfig {
  public:
    int pin = -1;
    uint8_t on_value = 0;
};

class LedGizmo {
  public:
    virtual ~LedGizmo() {}
    virtual void set(bool set_on) = 0;
    virtual void toggle() = 0;
};

class Led {
  public:
    LedConfig config;

    LedGizmo *gizmo = nullptr;

    int setup();      // Use config to setup gizmo, returns 0 on success, or error code
    bool installed() {return (gizmo != nullptr); } // Returns true if a gizmo was setup

    bool enabled = true; //enable changes to led state
    void set(bool set_on);
    void on();
    void off();
    void toggle();
    void blink(int times);
};

extern Led led;
