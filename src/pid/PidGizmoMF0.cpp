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

#include "PidGizmoMF0.h"
#include "../rcl/rcl.h"
#include "../ahr/ahr.h"
#include "../cfg/cfg.h"
#include "Arduino.h" //constrain

void PidGizmoMF0::load_param() {
    //Controller parameters
    maxRoll        = ifneg(cfg.pid_rol_angl_lim, 30);  // Max roll angle in deg for angle mode - DO NOT INCREASE OVER 70 OR YOU WILL CRASH DUE TO GIMBAL-LOCKS
    maxPitch       = ifneg(cfg.pid_pit_angl_lim, 30);  // Max pitch angle in deg for angle mode - DO NOT INCREASE OVER 70 OR YOU WILL CRASH DUE TO GIMBAL-LOCKS
    maxRollRate    = ifneg(cfg.pid_rol_rate_lim, 50);  // Max roll rate in deg/sec for rate mode 
    maxPitchRate   = ifneg(cfg.pid_pit_rate_lim, 60);  // Max pitch rate in deg/sec for rate mode
    maxYawRate     = ifneg(cfg.pid_yaw_rate_lim, 160); // Max yaw rate in deg/sec for angle and rate mode
    i_limit        = ifneg(cfg.pid_i_limit,      25);  // Integrator saturation level, mostly for safety

    //PID Rate Mode - Scaled by .01 to bring PID output within -1 to 1 range
    Kp_ro_pi_rate  = 0.01 * ifneg(cfg.pid_kp0, 0.15);    // Roll/Pitch rate P-gain
    Ki_ro_pi_rate  = 0.01 * ifneg(cfg.pid_ki0, 0.2);     // Roll/Pitch rate I-gain
    Kd_ro_pi_rate  = 0.01 * ifneg(cfg.pid_kd0, 0.002);   // Roll/Pitch rate D-gain (be careful when increasing too high, motors will begin to overheat!)
    Kp_yaw_rate    = 0.01 * ifneg(cfg.pid_kp1, 0.3);     // Yaw rate P-gain
    Ki_yaw_rate    = 0.01 * ifneg(cfg.pid_ki1, 0.05);    // Yaw rate I-gain
    Kd_yaw_rate    = 0.01 * ifneg(cfg.pid_kd1, 0.00015); // Yaw rate D-gain (be careful when increasing too high, motors will begin to overheat!)

    //PID Angle Mode 
    Kp_ro_pi_angle = 0.01 * ifneg(cfg.pid_kp2, 0.4);  // Roll/Pitch P-gain
    Ki_ro_pi_angle = 0.01 * ifneg(cfg.pid_ki2, 0.1);  // Roll/Pitch I-gain
    Kd_ro_pi_angle = 0.01 * ifneg(cfg.pid_kd2, 0.05); // Roll/Pitch D-gain
    Kp_yaw_angle   = 0.01 * ifneg(cfg.pid_kp3, 0.6);  // Yaw P-gain
    //cfg.pid_ki3 unused
    Kd_yaw_angle   = 0.01 * ifneg(cfg.pid_kd3, 0.1);  // Yaw D-gain
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

void PidGizmoMF0::setup() {
  set_flightmode(FlightMode::mf_RATE);
  yaw_desired = ahr.yaw; //set desired yaw to current yaw, the yaw angle controller will hold this value
  load_param();
}

FlightMode PidGizmoMF0::set_flightmode(FlightMode fm) {
  switch(fm) {
    case FlightMode::mf_RATE:
    case FlightMode::mf_ANGLE: 
      flightmode = fm;
      break;
    default:
      //keep current flight mode, i.e. do nothing
      break;
  }
  return flightmode;
}

void PidGizmoMF0::controller() {
  switch(flightmode) {
    case FlightMode::mf_ANGLE: 
      control_Angle(rcl.throttle == 0); //Stabilize on pitch/roll angle setpoint, stabilize yaw on rate setpoint
      break;
    default: //RATE 
      control_Rate(rcl.throttle == 0); //Stabilize on rate setpoint
  }
}

void PidGizmoMF0::control_Angle(bool zero_integrators) {
  //DESCRIPTION: Computes control commands based on angle error
  /*
   * Basic PID control to stablize on angle setpoint based on desired states roll_des, pitch_des, and yaw_des. Error
   * is simply the desired state minus the actual state (ex. roll_des - ahr.roll). Two safety features
   * are implimented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent 
   * excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
   * they've maxed out throttle... saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
   * if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I 
   * terms will always start from 0 on takeoff. This function updates the variables PIDroll.PID, PIDpitch.PID, and PIDyaw.PID which
   * can be thought of as 1-D stablized signals. They are mixed to the configuration of the vehicle in out_Mixer().
   */ 

  //inputs: roll_des, pitch_des, yawRate_des
  //outputs: PIDroll.PID, PIDpitch.PID, PIDyaw.PID

  //desired values
  float roll_des = rcl.roll * maxRoll; //Between -maxRoll and +maxRoll
  float pitch_des = rcl.pitch * maxPitch; //Between -maxPitch and +maxPitch
  float yawRate_des = rcl.yaw * maxYawRate; //Between -maxYawRate roll_PIDand +maxYawRate

  //state vars
  static float integral_roll, integral_pitch, error_yawRate_prev, integral_yawRate;

  //Zero the integrators (used to don't let integrator build if throttle is too low, or to re-start the controller)
  if(zero_integrators) {
    integral_roll = 0;
    integral_pitch = 0;
    integral_yawRate = 0;
  }

  //Roll PID
  float error_roll = roll_des - ahr.roll;
  integral_roll += error_roll * imu.dt;
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_roll = ahr.gx;
  pid.roll.p = Kp_ro_pi_angle * error_roll;
  pid.roll.i = Ki_ro_pi_angle * integral_roll;
  pid.roll.d = -Kd_ro_pi_angle * derivative_roll;
  pid.roll.sum = pid.roll.p + pid.roll.i + pid.roll.d;

  //Pitch PID
  float error_pitch = pitch_des - ahr.pitch;
  integral_pitch += error_pitch * imu.dt;
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_pitch = ahr.gy;
  pid.pitch.p = Kp_ro_pi_angle * error_pitch;
  pid.pitch.i = Ki_ro_pi_angle * integral_pitch;
  pid.pitch.d = -Kd_ro_pi_angle * derivative_pitch;
  pid.pitch.sum = pid.pitch.p + pid.pitch.i + pid.pitch.d;

  //Yaw PID
  if(-0.02 < rcl.yaw && rcl.yaw < 0.02) {
    //on reset, set desired yaw to current yaw
    if(zero_integrators) yaw_desired = ahr.yaw; 

    //Yaw stick centered: hold yaw_desired
    float error_yaw = degreeModulus(yaw_desired - ahr.yaw);
    float desired_yawRate = error_yaw / 0.5; //set desired yawRate such that it gets us to desired yaw in 0.5 second
    float derivative_yaw = desired_yawRate - ahr.gz;
    pid.yaw.p = Kp_yaw_angle * error_yaw;
    pid.yaw.i = Kd_yaw_angle * derivative_yaw;
    pid.yaw.d = 0;
    pid.yaw.sum = pid.yaw.p + pid.yaw.i + pid.yaw.d;

    //update yaw rate controller
    error_yawRate_prev = 0;
  }else{
    //Yaw stick not centered: stablize on rate from GyroZ
    float error_yawRate = yawRate_des - ahr.gz;
    integral_yawRate += error_yawRate * imu.dt;
    integral_yawRate = constrain(integral_yawRate, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
    float derivative_yawRate = (error_yawRate - error_yawRate_prev) / imu.dt;
    pid.yaw.p = Kp_yaw_rate * error_yawRate;
    pid.yaw.i = Ki_yaw_rate * integral_yawRate;
    pid.yaw.d = Kd_yaw_rate * derivative_yawRate;
    pid.yaw.sum = pid.yaw.p + pid.yaw.i + pid.yaw.d;

    //Update derivative variables
    error_yawRate_prev = error_yawRate;

    //update yaw controller: 
    yaw_desired = ahr.yaw; //set desired yaw to current yaw, the yaw angle controller will hold this value
  }
}

void PidGizmoMF0::control_Rate(bool zero_integrators) {
  //Computes control commands based on state error (rate)
  //See explanation for control_Angle(). Everything is the same here except the error is now: desired rate - raw gyro reading.

  //inputs: roll_des, pitch_des, yawRate_des
  //outputs: PIDroll.PID, PIDpitch.PID, PIDyaw.PID

  //desired values
  float rollRate_des = rcl.roll * maxRollRate; //Between -maxRoll and +maxRoll
  float pitchRate_des = rcl.pitch * maxPitchRate; //Between -maxPitch and +maxPitch
  float yawRate_des = rcl.yaw * maxYawRate; //Between -maxYawRate and +maxYawRate 
  
  //state vars
  static float integral_roll, error_roll_prev;
  static float integral_pitch, error_pitch_prev;
  static float integral_yaw, error_yaw_prev;

  //Zero the integrators (used to don't let integrator build if throttle is too low, or to re-start the controller)
  if(zero_integrators) {
    integral_roll = 0;
    integral_pitch = 0;
    integral_yaw = 0;
  }

  //Roll
  float error_roll = rollRate_des - ahr.gx;
  integral_roll += error_roll * imu.dt;
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_roll = (error_roll - error_roll_prev) / imu.dt;
  pid.roll.p = Kp_ro_pi_rate * error_roll;
  pid.roll.i = Ki_ro_pi_rate * integral_roll;
  pid.roll.d = Kd_ro_pi_rate * derivative_roll;
  pid.roll.sum = pid.roll.p + pid.roll.i + pid.roll.d;

  //Pitch
  float error_pitch = pitchRate_des - ahr.gy;
  integral_pitch += error_pitch * imu.dt;
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_pitch = (error_pitch - error_pitch_prev) / imu.dt;
  pid.pitch.p = Kp_ro_pi_rate * error_pitch;
  pid.pitch.i = Ki_ro_pi_rate * integral_pitch;
  pid.pitch.d = Kd_ro_pi_rate * derivative_pitch;
  pid.pitch.sum = pid.pitch.p + pid.pitch.i + pid.pitch.d;

  //Yaw, stablize on rate from GyroZ
  float error_yaw = yawRate_des - ahr.gz;
  integral_yaw += error_yaw * imu.dt;
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_yaw = (error_yaw - error_yaw_prev) / imu.dt; 
  pid.yaw.p = Kp_yaw_rate * error_yaw;
  pid.yaw.i = Ki_yaw_rate * integral_yaw;
  pid.yaw.d = Kd_yaw_rate * derivative_yaw;
  pid.yaw.sum = pid.yaw.p + pid.yaw.i + pid.yaw.d;

  //Update derivative variables
  error_roll_prev = error_roll;
  error_pitch_prev = error_pitch;
  error_yaw_prev = error_yaw;
}
