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

#include "../hal/MF_Serial.h"
#include "../cfg/cfg.h"

#define RCL_MAX_CH 20 //maximum number of channels

struct RclConfig {
  public:
    Cfg::rcl_gizmo_enum gizmo = Cfg::rcl_gizmo_enum::mf_NONE; //the gizmo to use
    int ser_bus_id = -1; //Serial bus id
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

    uint16_t pwm[RCL_MAX_CH + 1] = {}; //pwm channel data. regular range: 988-2012, pwm[RCL_MAX_CH] is used for non assigned sticks

    float throttle = 0; //throttle stick value 0.0 (zero throttle/stick back) to 1.0 (full throttle/stick forward)
    float roll = 0; //roll stick value -1.0 (left) to 1.0 (right)
    float pitch = 0; //pitch stick value -1.0 (pitch up/stick back) to 1.0 (pitch down/stick forward)
    float yaw = 0; //yaw stick value -1.0 (left) to 1.0 (right)
    float vspeed = 0; //vertical speed stick value -1.0 (descent/stick back) to 1.0 (ascent/stick forward)
    bool armed = false; //armed state (triggered by arm switch or stick commands)
    uint8_t flightmode = 0; //flightmode 0 to 5

  private:
    int _getCh(int ch); //normalize 1-based parameter channel to 0-RCL_MAX_CH, where RCL_MAX_CH is used for invalid channels
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
    bool _arm_sw_prev = true; //default to true, to require arm switch first off, then on to enter armed state!
    uint32_t _arm_ts = 0;
};

extern Rcl rcl;
