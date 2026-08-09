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

#include "quaternion.h"
#include <math.h>

static float constrain(float x, float min, float max) {
    return (x < min ? min : (x > max ? max : x));
}

static constexpr float rad_to_deg = 180 / M_PI;
static constexpr float deg_to_rad = M_PI / 180;

void quaternion_from_euler_deg(float q[4], const float euler_rpy[3]) {
  float eul[3];
  eul[0] = euler_rpy[0] * deg_to_rad; //roll
  eul[1] = euler_rpy[1] * deg_to_rad; //pitch
  eul[2] = euler_rpy[2] * deg_to_rad; //yaw

  float c0 = cos(eul[0]/2);
  float s0 = sin(eul[0]/2);
  float c1 = cos(eul[1]/2);
  float s1 = sin(eul[1]/2);
  float c2 = cos(eul[2]/2);
  float s2 = sin(eul[2]/2);
  float c0c1 = c0*c1;
  float s0s1 = s0*s1;
  float s0c1 = s0*c1;
  float c0s1 = c0*s1;
  q[0] = c0c1*c2 + s0s1*s2;
  q[1] = s0c1*c2 - c0s1*s2;
  q[2] = c0s1*c2 + s0c1*s2;
  q[3] = c0c1*s2 - s0s1*c2;
}

void quaternion_to_euler_deg(float euler_rpy[3], const float q[4]) {
  euler_rpy[0] = atan2(q[0]*q[1] + q[2]*q[3], 0.5f - q[1]*q[1] - q[2]*q[2]) * rad_to_deg; //degrees - roll right is positive
  euler_rpy[1] = asin(constrain(-2.0f * (q[1]*q[3] - q[0]*q[2]), -1.0, 1.0)) * rad_to_deg; //degrees - pitch up is positive - use constrain() to prevent NaN due to rounding
  euler_rpy[2] = atan2(q[1]*q[2] + q[0]*q[3], 0.5f - q[2]*q[2] - q[3]*q[3]) * rad_to_deg; //degrees - yaw right is positive
}

void quaternion_mult(float result[4], const float p[4], const float q[4]) {
  result[0] = p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3];
  result[1] = p[0] * q[1] + p[1] * q[0] + p[2] * q[3] - p[3] * q[2];
  result[2] = p[0] * q[2] - p[1] * q[3] + p[2] * q[0] + p[3] * q[1];
  result[3] = p[0] * q[3] + p[1] * q[2] - p[2] * q[1] + p[3] * q[0];
}

void quaternion_inverse(float q[4]) {
  //q[0] = q[0];
  q[1] = -q[1];
  q[2] = -q[2];
  q[3] = -q[3];
}

void quaternion_inverse(float result[4], const float q[4]) {
  result[0] = q[0];
  result[1] = -q[1];
  result[2] = -q[2];
  result[3] = -q[3];
}

void quaternion_normalize(float q[4]) {
  float norm2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
  if(norm2 <= 0) {
    quaternion_init(q);
  }else{
    float inv_norm = 1 / sqrt(norm2);
    q[0] *= inv_norm;
    q[1] *= inv_norm;
    q[2] *= inv_norm;
    q[3] *= inv_norm;
  }
}

void quaternion_init(float q[4]) {
  q[0] = 1;
  q[1] = 0;
  q[2] = 0;
  q[3] = 0;
}
