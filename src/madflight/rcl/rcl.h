#pragma once

#include "../hal/MF_Serial.h"
#include "../cfg/cfg.h"

#define RCL_MAX_CH 20 //maximum number of channels

struct RclConfig {
  public:
    Cfg::rcl_gizmo_enum gizmo = Cfg::rcl_gizmo_enum::mf_NONE; //the gizmo to use
    MF_Serial *ser_bus = nullptr; //Serial bus
    int baud = 0; //baud rate. 0=autobaud
    int ppm_pin = -1; //pin for PPM receiver
};

class RclGizmo {
  public:
    virtual ~RclGizmo() {}
    virtual bool update() = 0; //returns true if channel pwm data was updated
    //virtual bool connected() = 0;
    //virtual void calibrate() = 0; //interactive calibration
    //virtual bool telem_statustext(uint8_t severity, char *text) {(void)severity; (void)text; return true;} //send MAVLink status text
};

class Rcl {
  public:
    RclConfig config;

    RclGizmo *gizmo = nullptr;

    int setup();   // Use config to setup gizmo, returns 0 on success, or error code
    bool update(); // Returns true if channel pwm data was updated
    bool installed() {return (gizmo != nullptr); } // Returns true if a gizmo was setup

    bool connected();
    void calibrate(); //interactive calibration
    bool telem_statustext(uint8_t severity, char *text); //send MAVLink status text

    uint16_t pwm[RCL_MAX_CH] = {}; //pwm channel data. values: 988-2012

    float throttle = 0; //throttle stick value 0.0 (zero throttle/stick back) to 1.0 (full throttle/stick forward)
    float roll = 0; //roll stick value -1.0 (left) to 1.0 (right)
    float pitch = 0; //pitch stick value -1.0 (pitch up/stick back) to 1.0 (pitch down/stick forward)
    float yaw = 0; //yaw stick value -1.0 (left) to 1.0 (right)
    float vspeed = 0; //vertical speed stick value -1.0 (descent/stick back) to 1.0 (ascent/stick forward)
    bool arm = false; //arm switch state
    uint8_t flightmode = 0; //flightmode 0 to 5

  private:
    float _ChannelNormalize(int val, int min, int center, int max, int deadband);
    void _setupStick(int stickno, int ch, int left_pull, int mid, int right_push);
    
    uint32_t update_time = 0;

    enum stick_enum {THR,ROL,PIT,YAW,ARM,FLT};

    struct stick_t{
      uint8_t ch; //stick/switch channel - 0 based
      uint16_t min; //stick/switch min pwm
      uint16_t mid; //stick center pwm
      uint16_t max; //stick/switch max pwm
      int8_t inv; //stick inverted: -1 inverted, 1 normal
    } st[6];

    uint16_t st_flt_spacing;
};

extern Rcl rcl;
