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

#include "PidGizmoMF2.h"
#include "../rcl/rcl.h"
#include "../veh/veh.h"
#include "../ahr/ahr.h"
#include "../veh/veh.h"
#include "../cfg/cfg.h"
#include "Arduino.h" //constrain

enum flag_enum {
  FLAG_RATE = 1,
  FLAG_ANGLE_YAWCENTER = 2,
  FLAG_ANGLE_YAW = 3,
};

void PidGizmoMF2::load_param() {
    //Controller Parameters
    i_limit              = 0.01 * ifneg(cfg.pid_i_limit, 10);  // Integrator saturation level in % of output
    pid_angl_mult        = ifneg(cfg.pid_angl_mult, 5);   // Multiplicator to convert Angle Error to Rate Error

    //roll
    param[0].kp          = 0.032029 * 0.001 * ifneg(cfg.pid_kp0, 45);
    param[0].ki          = 0.244381 * 0.001 * ifneg(cfg.pid_ki0, 80);
    param[0].kd          = 0.000529 * 0.001 * ifneg(cfg.pid_kd0, 30);
    param[0].rate_limit  = ifneg(cfg.pid_rol_rate_lim, 300);  // Max roll rate in deg/sec for rate mode 
    param[0].angle_limit = ifneg(cfg.pid_rol_angl_lim, 30);   // Max roll angle in deg for angle mode - DO NOT INCREASE OVER 70 OR YOU WILL CRASH DUE TO GIMBAL-LOCKS

    //pitch (-1 means use same values as roll for pitch but do not update the cfg parameters)
    param[1].kp          = (cfg.pid_kp1 < 0          ? param[0].kp          : 0.032029 * 0.001 * cfg.pid_kp1);
    param[1].ki          = (cfg.pid_ki1 < 0          ? param[0].ki          : 0.244381 * 0.001 * cfg.pid_ki1);
    param[1].kd          = (cfg.pid_kd1 < 0          ? param[0].kd          : 0.000529 * 0.001 * cfg.pid_kd1);
    param[1].rate_limit  = (cfg.pid_pit_rate_lim < 0 ? param[0].rate_limit  : cfg.pid_pit_rate_lim);
    param[1].angle_limit = (cfg.pid_pit_angl_lim < 0 ? param[0].angle_limit : cfg.pid_pit_angl_lim);

    //yaw
    param[2].kp          = 0.032029 * 0.001 * ifneg(cfg.pid_kp2, 45);
    param[2].ki          = 0.244381 * 0.001 * ifneg(cfg.pid_ki2, 80);
    param[2].kd          = 0.000529 * 0.001 * ifneg(cfg.pid_kd2, 0);
    param[2].rate_limit  = ifneg(cfg.pid_yaw_rate_lim, 160); // Max yaw rate in deg/sec for angle and rate mode
    param[2].angle_limit = -1; //unused

    //D-term filter
    if(cfg.pid_filt0_freq < 0) {
      //set default filter
      cfg.pid_filt0_type = MF_FilterType::mf_PT1;
      cfg.pid_filt0_freq = 30;
      cfg.pid_filt0_q    = -1; //unused for PT1
    }
    MF_Filter::setup(param[0].d_filter, cfg.pid_filt0_type, pid_freq, cfg.pid_filt0_freq,  cfg.pid_filt0_q);
    MF_Filter::setup(param[1].d_filter, cfg.pid_filt0_type, pid_freq, cfg.pid_filt0_freq,  cfg.pid_filt0_q);
    MF_Filter::setup(param[2].d_filter, cfg.pid_filt0_type, pid_freq, cfg.pid_filt0_freq,  cfg.pid_filt0_q);
}

//returns angle in range -180 to 180
static float degreeModulus(float v) {
  if(v >= 180) {
    return fmod(v + 180, 360) - 180;
  }else if(v < -180.0) {
    return fmod(v - 180, 360) + 180;
  }
  return v;
}

void PidGizmoMF2::setup() {
  if(!has_flightmode(veh.getFlightmode())) veh.setFlightmode(FlightMode::mf_RATE);
  stick = &rcl.roll;
  gyro = &ahr.gx;
  ahrs_angle = &ahr.roll;
  state = (PidStatePID_s*) &pid.roll;
  pid_freq = imu.getSampleRate();
  pid_dt = 1.0f / pid_freq;
  load_param();
  zeroIntegrators();
  for(uint8_t axis = 0; axis < 3; axis++) {
    param[axis].error_prev = 0;
  }
}

bool PidGizmoMF2::has_flightmode(FlightMode fm) {
  switch(fm) {
    case FlightMode::mf_RATE:
    case FlightMode::mf_ANGLE: 
      return true;
  }
  return false;
}

void PidGizmoMF2::controller() {
  if(rcl.throttle == 0) zeroIntegrators();
  switch( veh.getFlightmode() ) {
    case FlightMode::mf_ANGLE: 
      control_angle(); //Stabilize on pitch/roll angle setpoint, stabilize yaw on rate setpoint
      break;
    default:
      control_rate(); //Stabilize on rate setpoint
  }
}

void PidGizmoMF2::zeroIntegrators() {
  for(uint8_t axis = 0; axis < 3; axis++) {
    state[axis].i = 0;
  }
}

void PidGizmoMF2::control_angle() {
  // roll + pitch: use angle controller
  for(uint8_t axis = 0; axis <= 1; axis++) {
    float angle_setpoint = stick[axis] * param[axis].angle_limit;
    float angle_error = angle_setpoint - ahrs_angle[axis];
    float setpoint = pid_angl_mult * angle_error;
    control_axis(axis, setpoint);
  }
  // yaw: use rate controller
  control_rate_axis(2);
}  

void PidGizmoMF2::control_rate() {
  //roll + pitch + yaw: use rate controller
  for(uint8_t axis = 0; axis < 3; axis++) {
    control_rate_axis(axis);
  }
}

void PidGizmoMF2::control_rate_axis(int axis) {
    float setpoint = stick[axis] * param[axis].rate_limit;
    control_axis(axis, setpoint);
}

void PidGizmoMF2::control_axis(int axis, float rate_setpoint) {
  //Stabilize on rate from gyro
  //P
  float error = rate_setpoint - gyro[axis];
  state[axis].p = param[axis].kp * error;  
  //I
  float iterm = state[axis].i + param[axis].ki * error * pid_dt;
  iterm = constrain(iterm, -i_limit, i_limit);
  state[axis].i = iterm;  
  //D
  float dterm_unfiltered = param[axis].kd * (error - param[axis].error_prev) / pid_dt;
  float dterm_filtered = param[axis].d_filter->apply(dterm_unfiltered);
  param[axis].error_prev = error;
  state[axis].d = dterm_filtered;
  //Sum
  state[axis].sum = state[axis].p + state[axis].i + state[axis].d;

  //for debugging
  state[axis].setpoint = rate_setpoint;
  state[axis].a = dterm_unfiltered;
}
