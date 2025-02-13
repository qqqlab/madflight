#pragma once

class Battery {
  public:
    float i = 0; //Battery current (A)
    float v = 0; //battery voltage (V)
    float w = 0; //battery power (W)
    float mah = 0; //battery usage (mAh)
    float wh = 0; //battery usage (Wh)
    uint32_t interval_us = 10000; //update interval in us
    virtual void setup() = 0;
    virtual bool update() = 0; //returns true if battery was updated
};

extern Battery &bat;
