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

class PidGizmoMF2 : public PidGizmo {
public:
    PidGizmoMF2() {setup();}
    const char* name() override {return "MF2";};
    void setup() override;
    void load_param() override;
    bool has_flightmode(FlightMode fm) override;
    void controller() override;

private:
    enum class Mode {RCL_RATE, RCL_ANGLE, RCL_PASSTHRU};

    void zeroIntegrators();
    void control_mode(Mode mode[3]);
    void control_axis(int axis, float rate_setpoint);
    FlightMode get_default_flightmode();

    //Parameters are private and are loaded from cfg.pid_xxx with load_config()

    //Controller parameters
    float i_limit = 0;
    float pid_angl_mult = 0;

    struct axis_s {
        //parameters
        float kp = 0;
        float ki = 0;
        float kd = 0;
        float rate_limit = 0;
        float angle_limit = 0;
        MF_Filter *d_filter = nullptr;
        //intermediate state
        float error_prev = 0;
    } param[3];

    float pid_freq = 0;
    float pid_dt = 0;

    //pointers to external variables
    float *stick = nullptr; //roll/pitch/yaw stick inputs
    float *gyro = nullptr; //gx,gy,gz gyro
    float *ahrs_angle = nullptr; //roll/pitch/yaw euler angles
    PidStatePID_s *state = nullptr; //roll/pitch/yaw output state
};
