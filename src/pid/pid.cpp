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

#include "pid.h"
#include <math.h> //for fmod()

Pid pid;

float PIDController::control(float desired, float actual, float dt) {
  float err = desired - actual;
  err_i += err * dt;
  //Saturate integrator to prevent unsafe buildup
  if(err_i < -klimit_i) err_i = -klimit_i;
  if(err_i > klimit_i) err_i = klimit_i;
  float err_d = (err - err_prev) / dt;
  output = kscale * (kp * err + ki * err_i + kd * err_d);
  err_prev = err;
  return output;
}

// With user provided actual_derivative (velocity)
float PIDController::controlActualDerivative(float desired, float actual, float dt, float actual_derivative) {
  float err = desired - actual;
  err_i += err * dt;
  //Saturate integrator to prevent unsafe buildup
  if(err_i < -klimit_i) err_i = -klimit_i;
  if(err_i > klimit_i) err_i = klimit_i;
  float err_d = -actual_derivative; //negate actual_derivative -> PIDController output opposes the change in actual value
  output = kscale * (kp * err + ki * err_i + kd * err_d);
  err_prev = err;
  return output;
}

// Control a 360 degree value
float PIDController::controlDegrees(float desired, float actual, float dt) {
  float err = degreeModulus(desired - actual);
  err_i += err * dt;
  //Saturate integrator to prevent unsafe buildup
  if(err_i < -klimit_i) err_i = -klimit_i;
  if(err_i > klimit_i) err_i = klimit_i;
  float err_d = (err - err_prev) / dt;
  output = kscale * (kp * err + ki * err_i + kd * err_d);
  err_prev = err;
  return output;
}

// Control a 360 degree value, with user provided actual_derivative (velocity)
float PIDController::controlDegreesActualDerivative(float desired, float actual, float dt, float actual_derivative) {
  float err = degreeModulus(desired - actual);
  err_i += err * dt;
  //Saturate integrator to prevent unsafe buildup
  if(err_i < -klimit_i) err_i = -klimit_i;
  if(err_i > klimit_i) err_i = klimit_i;
  float err_d = -actual_derivative; //negate actual_derivative -> PIDController output opposes the change in actual value
  output = kscale * (kp * err + ki * err_i + kd * err_d);
  err_prev = err;
  return output;
}

void PIDController::reset() {
  err_i = 0;
  err_prev = 0;
}

//returns angle in range -180 to 180
float PIDController::degreeModulus(float v) {
  if(v >= 180) {
    return fmod(v + 180, 360) - 180;
  }else if(v < -180.0) {
    return fmod(v - 180, 360) + 180;
  }
  return v;
}