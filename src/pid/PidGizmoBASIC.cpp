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

#include "PidGizmoBASIC.h"
#include "../rcl/rcl.h"
#include "../ahr/ahr.h"
#include "../veh/veh.h"
#include "../cfg/cfg.h"
#include "Arduino.h" //constrain

void PidGizmoBASIC::load_param() {
    //Controller Parameters
    float i_limit        = 0.01 * ifneg(cfg.pid_i_limit, 10);  // Integrator saturation level in % of output
    param_angl_mult      = ifneg(cfg.pid_angl_mult, 5);   // Multiplicator to convert Angle Error to Rate Error

    //roll axis
    param[0].kp          = 0.032029 * 0.001 * ifneg(cfg.pid_kp0, 45);
    param[0].ki          = 0.244381 * 0.001 * ifneg(cfg.pid_ki0, 80);
    param[0].kd          = 0.000529 * 0.001 * ifneg(cfg.pid_kd0, 30);
    param[0].rate_limit  = ifneg(cfg.pid_rol_rate_lim, 300);  // Max roll rate in deg/sec for rate mode 
    param[0].angle_limit = ifneg(cfg.pid_rol_angl_lim, 30);   // Max roll angle in deg for angle mode - DO NOT INCREASE OVER 70 OR YOU WILL CRASH DUE TO GIMBAL-LOCKS
    param[0].i_limit     = i_limit;

    //pitch axis (-1 means use same values as roll for pitch but do not update the cfg parameters)
    param[1].kp          = (cfg.pid_kp1 < 0          ? param[0].kp          : 0.032029 * 0.001 * cfg.pid_kp1);
    param[1].ki          = (cfg.pid_ki1 < 0          ? param[0].ki          : 0.244381 * 0.001 * cfg.pid_ki1);
    param[1].kd          = (cfg.pid_kd1 < 0          ? param[0].kd          : 0.000529 * 0.001 * cfg.pid_kd1);
    param[1].rate_limit  = (cfg.pid_pit_rate_lim < 0 ? param[0].rate_limit  : cfg.pid_pit_rate_lim);
    param[1].angle_limit = (cfg.pid_pit_angl_lim < 0 ? param[0].angle_limit : cfg.pid_pit_angl_lim);
    param[1].i_limit     = i_limit;

    //yaw axis
    param[2].kp          = 0.032029 * 0.001 * ifneg(cfg.pid_kp2, 45);
    param[2].ki          = 0.244381 * 0.001 * ifneg(cfg.pid_ki2, 80);
    param[2].kd          = 0.000529 * 0.001 * ifneg(cfg.pid_kd2, 0);
    param[2].rate_limit  = ifneg(cfg.pid_yaw_rate_lim, 160); // Max yaw rate in deg/sec for angle and rate mode
    param[2].angle_limit = -1; //unused
    param[2].i_limit     = i_limit;

    //throttle axis is always passthru in this controller

    //D-term filter
    if(cfg.pid_filt0_freq < 0) {
      //set default filter
      cfg.pid_filt0_type = MF_FilterType::mf_PT1;
      cfg.pid_filt0_freq = 30;
      cfg.pid_filt0_q    = -1; //unused for PT1
    }
    MF_Filter::setup(param[0].d_filter, cfg.pid_filt0_type, pid_freq, cfg.pid_filt0_freq, cfg.pid_filt0_q);
    MF_Filter::setup(param[1].d_filter, cfg.pid_filt0_type, pid_freq, cfg.pid_filt0_freq, cfg.pid_filt0_q);
    MF_Filter::setup(param[2].d_filter, cfg.pid_filt0_type, pid_freq, cfg.pid_filt0_freq, cfg.pid_filt0_q);
    MF_Filter::setup(param[3].d_filter, cfg.pid_filt0_type, pid_freq, cfg.pid_filt0_freq, cfg.pid_filt0_q);
}

void PidGizmoBASIC::setup() {
  if(veh.mav_type == VEH_TYPE_PLANE) {
    set_flightmode(FlightMode::mf_PLANE_MANUAL);
  }else{
    set_flightmode(FlightMode::mf_RATE);
  }
  stick = &rcl.roll;
  gyro = &ahr.gx;
  ahrs_angle = &ahr.roll;
  state = (PidStatePID_s*) &pid.roll;
  pid_freq = imu.getSampleRate();
  pid_dt = 1.0f / pid_freq;
  load_param();
  zeroIntegrators();
  for(uint8_t axis = 0; axis < 4; axis++) {
    param[axis].error_prev = 0;
  }
}

//set controller mode for each axis (roll/pitch/yaw)
FlightMode PidGizmoBASIC::set_flightmode(FlightMode fm) {
  switch(fm) {

    case FlightMode::mf_RATE: {
      //Stabilize rate for roll/pitch/yaw
      mode[0] = Mode::RCL_RATE; //roll
      mode[1] = Mode::RCL_RATE; //pitch
      mode[2] = Mode::RCL_RATE; //yaw
      mode[3] = Mode::RCL_PASSTHRU; //throttle
      flightmode = fm;
      break;
    }

    case FlightMode::mf_ANGLE: {
      //Stabilize angle for roll/pitch, stabilize rate for yaw
      mode[0] = Mode::RCL_ANGLE; //roll
      mode[1] = Mode::RCL_ANGLE; //pitch
      mode[2] = Mode::RCL_RATE; //yaw (control yaw by rate only - will drift when stick centered)
      mode[3] = Mode::RCL_PASSTHRU; //throttle
      flightmode = fm;
      break;
    }

    case FlightMode::mf_PLANE_FBWA: {
      //Stabilize angle for roll/pitch, passthru for yaw
      mode[0] = Mode::RCL_ANGLE; //roll
      mode[1] = Mode::RCL_ANGLE; //pitch
      mode[2] = Mode::RCL_PASSTHRU; //yaw
      mode[3] = Mode::RCL_PASSTHRU; //throttle
      flightmode = fm;
      break;
    }

    case FlightMode::mf_PLANE_ROLL: {
      //Stabilize angle for roll, passthru for pitch/yaw
      mode[0] = Mode::RCL_ANGLE; //roll
      mode[1] = Mode::RCL_PASSTHRU; //pitch
      mode[2] = Mode::RCL_PASSTHRU; //yaw
      mode[3] = Mode::RCL_PASSTHRU; //throttle
      flightmode = fm;
      break;
    }

    case FlightMode::mf_PLANE_MANUAL: {
      // Passthru roll/pitch/yaw
      mode[0] = Mode::RCL_PASSTHRU; //roll
      mode[1] = Mode::RCL_PASSTHRU; //pitch
      mode[2] = Mode::RCL_PASSTHRU; //yaw
      mode[3] = Mode::RCL_PASSTHRU; //throttle
      flightmode = fm;
      break;
    }

    default: 
      //requested flightmode not supported - keep existing mode
      break;
  }
  return flightmode;
}

void PidGizmoBASIC::zeroIntegrators() {
  for(uint8_t axis = 0; axis < 4; axis++) {
    state[axis].i = 0;
  }
}

void PidGizmoBASIC::controller() {
  if(rcl.throttle == 0) zeroIntegrators();

  for(int axis = 0; axis < 4; axis++) {
    switch(mode[axis]) { 
      case Mode::RCL_RATE: {
        //control roll/pitch/yaw axis by rate setpoint
        float rate_setpoint = stick[axis] * param[axis].rate_limit;
        state[axis].setpoint = rate_setpoint;
        state[axis].actual = gyro[axis];
        control_axis(axis);
        break;
      }
      case Mode::RCL_ANGLE: {
        //control  roll/pitch axis by angle setpoint
        float angle_setpoint = stick[axis] * param[axis].angle_limit;
        float angle_error = angle_setpoint - ahrs_angle[axis];
        float rate_setpoint = param_angl_mult * angle_error;
        state[axis].setpoint = rate_setpoint;
        state[axis].actual = gyro[axis];
        control_axis(axis);
        break;
      }
      case Mode::RCL_PASSTHRU: {
        //passthru stick setpoint
        state[axis].p = 0;
        state[axis].i = 0;
        state[axis].d = 0;
        state[axis].a = 0;
        state[axis].sum = stick[axis];
        state[axis].setpoint = 0;
        state[axis].actual = 0;
        break;
      }
    }

    //for debugging
    state[axis].b = (float)mode[axis];
  }
}

void PidGizmoBASIC::control_axis(int axis) {
  //P
  float error = state[axis].setpoint - state[axis].actual;
  state[axis].p = param[axis].kp * error;  
  //I
  float iterm = state[axis].i + param[axis].ki * error * pid_dt;
  iterm = constrain(iterm, -param[axis].i_limit, param[axis].i_limit);
  state[axis].i = iterm;  
  //D
  float dterm_unfiltered = param[axis].kd * (error - param[axis].error_prev) / pid_dt;
  float dterm_filtered = param[axis].d_filter->apply(dterm_unfiltered);
  param[axis].error_prev = error;
  state[axis].d = dterm_filtered;
  //Sum
  state[axis].sum = state[axis].p + state[axis].i + state[axis].d;

  //for debugging
  state[axis].a = dterm_unfiltered;
}
