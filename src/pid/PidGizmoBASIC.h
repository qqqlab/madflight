/*==========================================================================================
MIT License

Copyright (c) 2026 https://madflight.com

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

// Basic PID controller

#pragma once

#include "pid.h"
#include "../cfg/cfg.h"
#include "../tbx/tbx.h"

class PidGizmoBASIC : public PidGizmo {
public:
    PidGizmoBASIC() {setup();}
    const char* name() override {return "BASIC";};
    void setup() override;
    void load_param() override;
    FlightMode set_flightmode(FlightMode fm) override; //set flightmode, returns actual fm set which could be differnt than requested
    void controller() override;

private:
    FlightMode flightmode = FlightMode::mf_RATE;
    
    enum class Mode {
        RCL_RATE,        // control axis by rate setpoint
        RCL_ANGLE,       // control axis by angle setpoint
        RCL_PASSTHRU,    // passthru stick setpoint
        RCL_YAW_HYBRID,  // keep yaw angle when stick centered, RCL_RATE otherwise - use only for axis 2 (yaw)
    };

    //Controller mode for 4 axis (roll,pitch,yaw,throttle), flightmode is mapped to mode[4]
    Mode mode[4] = {
        Mode::RCL_RATE, //roll
        Mode::RCL_RATE, //pitch
        Mode::RCL_RATE, //yaw
        Mode::RCL_PASSTHRU //throttle
    };

    //Parameters are private and are loaded from cfg.pid_xxx with load_config()

    //Controller parameters
    float param_angl_mult = 0;

    struct axis_s {
        //parameters for roll/pitch/yaw/throttle
        float kp = 0;
        float ki = 0;
        float kd = 0;
        float rate_limit = 0;
        float angle_limit = 0;
        float i_limit = 0;
        MF_Filter *d_filter = nullptr;
        //intermediate state
        float error_prev = 0;
    } param[4];

    float pid_freq = 0;
    float pid_dt = 0;

    //pointers to external modules
    float *stick = nullptr;         //RCL roll/pitch/yaw/throttle/vspeed stick inputs
    float *gyro = nullptr;          //AHR gx,gy,gz gyro
    float *ahrs_angle = nullptr;    //AHR roll/pitch/yaw euler angles
    PidStatePID_s *state = nullptr; //PID roll/pitch/yaw/throttle output state

    void zeroIntegrators();
    void control_axis(int axis);
};
