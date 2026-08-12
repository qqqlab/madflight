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

#include "PidGizmoMF1.h"
#include "../rcl/rcl.h"
#include "../ahr/ahr.h"
#include "../cfg/cfg.h"
#include "Arduino.h" //constrain

enum flag_enum {
  FLAG_RATE = 1,
  FLAG_ANGLE_YAWCENTER = 2,
  FLAG_ANGLE_YAW = 3,
};

void PidGizmoMF1::load_param() {
    //Controller parameters
    maxRoll        =        ifneg(cfg.pid_rol_angl_lim, 30);  // Max roll angle in deg for angle mode - DO NOT INCREASE OVER 70 OR YOU WILL CRASH DUE TO GIMBAL-LOCKS
    maxPitch       =        ifneg(cfg.pid_pit_angl_lim, 30);  // Max pitch angle in deg for angle mode - DO NOT INCREASE OVER 70 OR YOU WILL CRASH DUE TO GIMBAL-LOCKS
    maxRollRate    =        ifneg(cfg.pid_rol_rate_lim, 50);  // Max roll rate in deg/sec for rate mode 
    maxPitchRate   =        ifneg(cfg.pid_pit_rate_lim, 60);  // Max pitch rate in deg/sec for rate mode
    maxYawRate     =        ifneg(cfg.pid_yaw_rate_lim, 160); // Max yaw rate in deg/sec for angle and rate mode
    i_limit        = 0.01 * ifneg(cfg.pid_i_limit,      10);  // Integrator saturation level in % of output
    pid_angl_mult  =        ifneg(cfg.pid_angl_mult,    5);   // Multiplicator to convert Angle Error to Rate Error

    //PID Rate Mode - Scaled by .01 to bring PID output within -1 to 1 range
    Kp_ro_pi_rate  = 0.01 * ifneg(cfg.pid_kp0, 0.15);    // Roll/Pitch rate P-gain
    Ki_ro_pi_rate  = 0.01 * ifneg(cfg.pid_ki0, 0.2);     // Roll/Pitch rate I-gain
    Kd_ro_pi_rate  = 0.01 * ifneg(cfg.pid_kd0, 0.002);   // Roll/Pitch rate D-gain (be careful when increasing too high, motors will begin to overheat!)
    Kp_yaw_rate    = 0.01 * ifneg(cfg.pid_kp1, 0.3);     // Yaw rate P-gain
    Ki_yaw_rate    = 0.01 * ifneg(cfg.pid_ki1, 0.05);    // Yaw rate I-gain
    Kd_yaw_rate    = 0.01 * ifneg(cfg.pid_kd1, 0.00015); // Yaw rate D-gain (be careful when increasing too high, motors will begin to overheat!)

    //PID Angle Mode 
    Kp_ro_pi_angle = 0.01 * ifneg(cfg.pid_kp2, 0.4);  // Roll/Pitch P-gain
    Ki_ro_pi_angle = 0.01 * ifneg(cfg.pid_ki2, 1);  // Roll/Pitch I-gain
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

void PidGizmoMF1::setup() {
  set_flightmode(FlightMode::mf_RATE);
  yaw_angle_desired = ahr.yaw; //set desired yaw to current yaw, the yaw angle controller will hold this value
  load_param();
  zeroIntegrators();
  error_roll_prev = 0;
  error_pitch_prev = 0;
  error_yaw_prev = 0;
}

FlightMode PidGizmoMF1::set_flightmode(FlightMode fm) {
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

void PidGizmoMF1::controller() {
  if(rcl.throttle == 0) zeroIntegrators();
  switch(flightmode) {
    case FlightMode::mf_ANGLE: 
      control_Angle(); //Stabilize on pitch/roll angle setpoint, stabilize yaw on rate setpoint
      break;
    default:
      control_Rate(); //Stabilize on rate setpoint
  }
}

void PidGizmoMF1::zeroIntegrators() {
  integral_roll = 0;
  integral_pitch = 0;
  integral_yaw_rate = 0;
  integral_yaw_angle = 0;
}

void PidGizmoMF1::control_Angle() {
  //Roll Angle PID
  float roll_des = rcl.roll * maxRoll; //Desired roll angle, between -maxRoll and +maxRoll
  float error_roll = roll_des - ahr.roll;
  integral_roll += Ki_ro_pi_angle * error_roll * imu.dt;
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_roll = ahr.gx;
  pid.roll.p = Kp_ro_pi_angle * error_roll;
  pid.roll.i = integral_roll;
  pid.roll.d = -Kd_ro_pi_angle * derivative_roll;
  pid.roll.sum = pid.roll.p + pid.roll.i + pid.roll.d;

  //Pitch Angle PID
  float pitch_des = rcl.pitch * maxPitch; //Desired pitch angle, between -maxPitch and +maxPitch
  float error_pitch = pitch_des - ahr.pitch;
  integral_pitch += Ki_ro_pi_angle * error_pitch * imu.dt;
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_pitch = ahr.gy;
  pid.pitch.p = Kp_ro_pi_angle * error_pitch;
  pid.pitch.i = integral_pitch;
  pid.pitch.d = -Kd_ro_pi_angle * derivative_pitch;
  pid.pitch.sum = pid.pitch.p + pid.pitch.i + pid.pitch.d;

  //Yaw Angle PID
  if(-0.02 < rcl.yaw && rcl.yaw < 0.02) {
    //stick centered - control yaw angle
    float error_yaw = degreeModulus(yaw_angle_desired - ahr.yaw);
    float yawRate_des = pid_angl_mult * error_yaw; //set desired yaw rate based on yaw angle error
    //do not update desired yaw angle (keep position when got out of yaw-rate mode)
    integral_yaw_rate = 0; //reset yaw rate integrator
    control_YawRate(yawRate_des, &integral_yaw_angle); //use angle integrator
    
    //debug
    pid.yaw.a = FLAG_ANGLE_YAWCENTER;
  }else{
    //stick off-center - control yaw rate
    float yawRate_des = rcl.yaw * maxYawRate; //Desired yaw rate, between -maxYawRate and +maxYawRate
    yaw_angle_desired = ahr.yaw; //set desired yaw to current yaw, the yaw angle controller will hold this value
    integral_yaw_angle = 0; //reset angle integrator
    control_YawRate(yawRate_des, &integral_yaw_rate); //use rate integrator

    //debug
    pid.yaw.a = FLAG_ANGLE_YAW;
  }
}

void PidGizmoMF1::control_Rate() {
  //Roll Rate - Stabilize on rate from GyroX
  float rollRate_des = rcl.roll * maxRollRate; //Desired Roll Rate, between -maxRoll and +maxRoll
  float error_roll = rollRate_des - ahr.gx;
  integral_roll += Ki_ro_pi_rate * error_roll * imu.dt;
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_roll = (error_roll - error_roll_prev) / imu.dt;
  pid.roll.p = Kp_ro_pi_rate * error_roll;
  pid.roll.i = integral_roll;
  pid.roll.d = Kd_ro_pi_rate * derivative_roll;
  pid.roll.sum = pid.roll.p + pid.roll.i + pid.roll.d;
  error_roll_prev = error_roll; //Update derivative variable

  //Pitch Rate - Stabilize on rate from GyroY
  float pitchRate_des = rcl.pitch * maxPitchRate; //Desired Pitch Rate, between -maxPitch and +maxPitch
  float error_pitch = pitchRate_des - ahr.gy;
  integral_pitch += Ki_ro_pi_rate * error_pitch * imu.dt;
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_pitch = (error_pitch - error_pitch_prev) / imu.dt;
  pid.pitch.p = Kp_ro_pi_rate * error_pitch;
  pid.pitch.i = integral_pitch;
  pid.pitch.d = Kd_ro_pi_rate * derivative_pitch;
  pid.pitch.sum = pid.pitch.p + pid.pitch.i + pid.pitch.d;
  error_pitch_prev = error_pitch; //Update derivative variable

  //Yaw Rate
  float yawRate_des = rcl.yaw * maxYawRate; 
  control_YawRate(yawRate_des, &integral_yaw_rate);

  //debug
  pid.yaw.a = FLAG_RATE;
}

//Yaw Rate - Stabilize on rate from GyroZ, yawRate_des = Desired Yaw Rate, between-maxYawRate and +maxYawRate 
void PidGizmoMF1::control_YawRate(float yawRate_des, float* integral_yaw) {
  float error_yaw = yawRate_des - ahr.gz;
  *integral_yaw += Ki_yaw_rate * error_yaw * imu.dt;
  *integral_yaw = constrain(*integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  float derivative_yaw = (error_yaw - error_yaw_prev) / imu.dt; 
  pid.yaw.p = Kp_yaw_rate * error_yaw;
  pid.yaw.i = *integral_yaw;
  pid.yaw.d = Kd_yaw_rate * derivative_yaw;
  pid.yaw.sum = pid.yaw.p + pid.yaw.i + pid.yaw.d;
  error_yaw_prev = error_yaw;  //Update derivative variable
}
