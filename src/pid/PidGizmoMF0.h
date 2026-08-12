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

class PidGizmoMF0 : public PidGizmo {
public:
    PidGizmoMF0() {setup();}
    const char* name() override {return "MF0";};
    void setup() override;
    void load_param() override;
    FlightMode set_flightmode(FlightMode fm) override; //set flightmode, returns actual fm set which could be differnt than requested
    void controller() override;

private:
    void control_Angle(bool zero_integrators);
    void control_Rate(bool zero_integrators);

    FlightMode flightmode = FlightMode::mf_RATE;

    //Yaw to keep in ANGLE mode when yaw stick is centered
    float yaw_desired = 0;

    //Parameters are private and are loaded from cfg.pid_xxx with load_config()

    //Controller parameters
    float maxRoll;
    float maxPitch;
    float maxRollRate;
    float maxPitchRate;
    float maxYawRate;
    float i_limit;

    //PID Rate Mode 
    float Kp_ro_pi_rate;
    float Ki_ro_pi_rate;
    float Kd_ro_pi_rate;
    float Kp_yaw_rate;
    float Ki_yaw_rate;
    float Kd_yaw_rate;

    //PID Angle Mode 
    float Kp_ro_pi_angle;
    float Ki_ro_pi_angle;
    float Kd_ro_pi_angle;
    float Kp_yaw_angle;
    float Kd_yaw_angle;
};
