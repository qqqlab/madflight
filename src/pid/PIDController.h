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
#pragma once

class PIDController {
  public:
    float out = 0; //PIDController output value (= out_p + out_i + out_d)

    float out_p = 0; //proporional output
    float out_i = 0; //integral output, limited to -imax to +imax
    float out_d = 0; //derivative output

    float kp = 0; //Kp
    float ki = 0; //Ki
    float kd = 0; //Kd
    float imax = 0; //integral(ki*err*dt) is limited to -imax to +imax

  private:
    float err_prev = 0; //previous error

  public:
    PIDController() {}
    PIDController(float kp, float ki, float kd, float imax, float kscale = 1.0f);
    void begin(float kp, float ki, float kd, float imax, float kscale = 1.0f);

    float control(float desired, float actual, float dt);
    float controlActualDerivative(float desired, float actual, float dt, float actual_derivative);

    // Control a 360 degree value (wraps around correctly)
    float controlDegrees(float desired, float actual, float dt);
    float controlDegreesActualDerivative(float desired, float actual, float dt, float actual_derivative);

    void reset(); //reset err_i and err_prev

    static float degreeModulus(float v); //returns angle in range -180 to 180

  private:
    float controlErr(float err, float dt);
    float controlErrActualDerivative(float err, float dt, float actual_derivative);
};
