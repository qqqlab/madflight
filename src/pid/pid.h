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

#pragma once

class Pid {
  public:
    //PIDController outputs for CLI
    float roll = 0;
    float pitch = 0;
    float yaw = 0;
};
extern Pid pid;

class PIDController {
  public:
    float output = 0; //PIDController output value

    float err_i = 0; //integral error
    float err_prev = 0; //previous error

    float kp = 0; //Kp
    float ki = 0; //Ki
    float kd = 0; //Kd
    float kscale = 1; //scale factor
    float klimit_i = 0; //err_i is limited to -limit_i to +limit_i

    PIDController(float kp, float ki, float kd, float kscale, float klimit_i) {
      this->kp = kp;
      this->ki = ki;
      this->kd = kd;
      this->kscale = kscale;
      this->klimit_i = klimit_i;
    }

    float control(float desired, float actual, float dt);
    float controlActualDerivative(float desired, float actual, float dt, float actual_derivative);

    // Control a 360 degree value (wraps around correctly)
    float controlDegrees(float desired, float actual, float dt);
    float controlDegreesActualDerivative(float desired, float actual, float dt, float actual_derivative);

    void reset(); //reset err_i and err_prev

    static float degreeModulus(float v); //returns angle in range -180 to 180
};


