#pragma once

#include "../hal/MF_I2C.h"
#include "../cfg/cfg.h"

struct BarState {
  public:
    float press = 0;  // Pressure in [Pascal]
    float alt = 0;    // Approximate International Standard Atmosphere (ISA) Altitude in [m]
    float temp = 0;   // Temperature in [Celcius]
    uint32_t ts = 0;  // Sample timestamp in [us]
    float dt = 0;     // Time since last sample in [seconds]
};

struct BarConfig {
  public:
    uint32_t sampleRate = 100; //sample rate [Hz]
    Cfg::bar_gizmo_enum gizmo = Cfg::bar_gizmo_enum::mf_NONE; //the gizmo to use
    MF_I2C *i2c_bus = nullptr; //i2c bus
    uint8_t i2c_adr = 0; //i2c address. 0=default address
};

class BarGizmo {
public:
  virtual ~BarGizmo() {}
  virtual bool update(float *press, float *temp) = 0; //returns true if pressure was updated
};

class Bar : public BarState {
  public:
    BarConfig config;

    BarGizmo *gizmo = nullptr;

    int setup();      // Use config to setup gizmo, returns 0 on success, or error code
    bool update();    // Returns true if state was updated
    bool installed() {return (gizmo != nullptr); } // Returns true if a gizmo was setup

  private:
    uint32_t _samplePeriod;
};

//Global module instance
extern Bar bar;
