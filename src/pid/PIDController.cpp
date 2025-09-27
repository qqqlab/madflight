/*==========================================================================================
MIT License

Copyright (c) 2025 https://madflight.com

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

#include "PIDController.h"
#include <math.h> //for fmod()

#ifndef constrain
  #define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
#endif

PIDController::PIDController(float kp, float ki, float kd, float imax, float kscale) {
    begin(kp, ki, kd, imax, kscale);
}

void PIDController::begin(float kp, float ki, float kd, float imax, float kscale) {
    this->kp = kp * kscale;
    this->ki = ki * kscale;
    this->kd = kd * kscale;
    this->imax = imax;
}

// (private) Standard PID
float PIDController::controlErr(float err, float dt) {
  out_p = kp * err;
  out_i += ki * err * dt;
  out_i = constrain(out_i, -imax, imax);
  out_d = kd * (err - err_prev) / dt;
  err_prev = err;
  output = out_p + out_i + out_d;
  return output;
}

// (private) With user provided actual_derivative (velocity)
float PIDController::controlErrActualDerivative(float err, float dt, float actual_derivative) {
  out_p = kp * err;
  out_i += ki * err * dt;
  //Saturate integrator to prevent unsafe buildup
  if(out_i < -imax) out_i = -imax;
  if(out_i > imax) out_i = imax;
  out_d = -kd * actual_derivative / dt; //negate actual_derivative -> PIDController output opposes the change in actual value
  output = out_p + out_i + out_d;
  return output;
}

// Standard PID
float PIDController::control(float desired, float actual, float dt) {
  return controlErr(desired - actual, dt);
}

// Control a 360 degree value
float PIDController::controlDegrees(float desired, float actual, float dt) {
  return controlErr(degreeModulus(desired - actual) - actual, dt);
}

// With user provided actual_derivative (velocity)
float PIDController::controlActualDerivative(float desired, float actual, float dt, float actual_derivative) {
  return controlErrActualDerivative(desired - actual, dt, actual_derivative);
}

// Control a 360 degree value, with user provided actual_derivative (velocity)
float PIDController::controlDegreesActualDerivative(float desired, float actual, float dt, float actual_derivative) {
  return controlErrActualDerivative(degreeModulus(desired - actual), dt, actual_derivative);
}

void PIDController::reset() {
  out_i = 0;
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
