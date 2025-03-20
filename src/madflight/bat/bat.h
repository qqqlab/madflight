#pragma once

#include "../hal/MF_I2C.h"
#include "../cfg/cfg.h"

struct BatState {
  public:
    float i = 0;     // Battery current [A]
    float v = 0;     // Battery voltage [V]
    float w = 0;     // Battery power [W]
    float mah = 0;   // Battery usage [mAh]
    float wh = 0;    // Battery usage [Wh]
    uint32_t ts = 0; // Last update time stamp [us]
};

struct BatConfig {
  public:
    uint32_t sampleRate = 100; //sample rate [Hz]
    Cfg::bat_gizmo_enum gizmo = Cfg::bat_gizmo_enum::mf_NONE; //the gizmo to use
    MF_I2C *i2c_bus = nullptr; //i2c bus
    uint8_t i2c_adr = 0; //i2c address. 0=default address
    int32_t adc_pin_v = -1;
    int32_t adc_pin_i = -1;
    float adc_cal_v = 1.0; //voltage conversion factor: set to [Actual measured Volt] / [bat_v ADC reading when cal_v=1.0]
    float adc_cal_i = 1.0; //current conversion factor: set to [Actual measured Amperes] / [bat_i ADC reading when cal_i=1.0]
    float rshunt = 1.0; //shunt resistor value [Ohm]
};

class BatGizmo {
  public:
    virtual ~BatGizmo() {}
    virtual bool update() = 0; //returns true if new sample was taken
};

class Bat : public BatState {
  public:
    BatConfig config;

    BatGizmo *gizmo = nullptr;

    int setup();      // Use config to setup gizmo, returns 0 on success, or error code
    bool update();    // Returns true if state was updated
    bool installed() {return (gizmo != nullptr); } // Returns true if a gizmo was setup
};

//Global module instance
extern Bat bat;
