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

#pragma once

#include "pid.h"
#include "../cfg/cfg.h"
#include "../tbx/tbx.h"

class PidGizmoEXPERIMENTAL : public PidGizmo {
public:
    PidGizmoEXPERIMENTAL() {setup();}
    const char* name() override {return "EXPERIMENTAL";};
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
        RCL_YAWHYBRID,   // yaw angle/rate hybrid: keep yaw angle when stick centered, control yaw rate when stick off-center
        RCL_THR_ALTHOLD, // altitude hold throttle controller
    };

    Mode mode[4] = {
        Mode::RCL_RATE, //roll
        Mode::RCL_RATE, //pitch
        Mode::RCL_RATE, //yaw
        Mode::RCL_PASSTHRU //throttle
    };

    //axis names
    enum Axis_enum {
        AXIS_ROLL,     //RCL,AHR,PID
        AXIS_PITCH,    //RCL,AHR,PID
        AXIS_YAW,      //RCL,AHR,PID
        AXIS_THROTTLE, //RCL,AHR,PID
        AXIS_VSPEED,   //RCL only
    };

    //yaw_hybrid
    float state_yawhybrid_angle_setpoint = 0; //RCL_YAW_HYBRID: yaw angle setpoint when yaw stick centered

    //althold
    float state_althold_altitude_setpoint = 0; //RCL_THR_ALTHOLD: altitude setpoint when vertical-speed stick centered
    float state_althold_hover_throttle = 0.55;

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
    float param_althold_stick_limit = 0;

    float pid_freq = 0;
    float pid_dt = 0;

    //pointers to external variables
    float *stick = nullptr; //roll/pitch/yaw/throttle/vspeed stick inputs
    float *gyro = nullptr; //gx,gy,gz gyro
    float *ahrs_angle = nullptr; //roll/pitch/yaw euler angles
    PidStatePID_s *state = nullptr; //roll/pitch/yaw/throttle output state

    void zeroIntegrators();
    void control_axis(int axis);
};
