#pragma once

#include "../hal/MF_I2C.h"
#include "../cfg/cfg.h"

struct MagState {
  public:
    float x = 0; //"North" magnetic flux [uT]
    float y = 0; //"East" magnetic flux [uT]
    float z = 0; //"Down" magnetic flux [uT]
};

struct MagConfig {
  public:
    uint32_t sampleRate = 100; //sample rate [Hz]
    Cfg::mag_gizmo_enum gizmo = Cfg::mag_gizmo_enum::mf_NONE; //the gizmo to use
    MF_I2C *i2c_bus = nullptr; //i2c bus
    uint8_t i2c_adr = 0; //i2c address. 0=default address
};

class MagGizmo {
public:
  virtual ~MagGizmo() {};
  virtual bool read_uT(float *x, float *y, float *z) = 0; //returns true if new sample was retrieved
};

class Mag : public MagState {
  public:
    MagConfig config;

    MagGizmo *gizmo = nullptr;

    int setup();      // Use config to setup gizmo, returns 0 on success, or error code
    bool update();    // Returns true if state was updated
    bool installed() {return (gizmo != nullptr); } // Returns true if a gizmo was setup

  protected:
    uint32_t mag_time = 0; //last sample time in [us]
    uint32_t _samplePeriod = 0; //gizmo sample period in [us]

    bool _installed = false;
};

//Global module instance
extern Mag mag;
