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

#include "maths.h" //fast approximations of sin, cos, etc

static float _constrain(float x, float min, float max) {
    return (x < min ? min : (x > max ? max : x));
}

static constexpr float rad_to_deg = 180.0f / 3.14159265358979323846f;
static constexpr float deg_to_rad = 3.14159265358979323846f / 180.0f;
static constexpr float deg_to_rad_2 = 3.14159265358979323846f / 360.0f;

void quaternion_from_euler_deg(float q[4], const float euler_rpy[3]) {
  float eul_div2[3];
  eul_div2[0] = euler_rpy[0] * deg_to_rad_2; //roll
  eul_div2[1] = euler_rpy[1] * deg_to_rad_2; //pitch
  eul_div2[2] = euler_rpy[2] * deg_to_rad_2; //yaw

  float s0,s1,s2,c0,c1,c2;
  sincos_approx(eul_div2[0], &s0, &c0);
  sincos_approx(eul_div2[1], &s1, &c1);
  sincos_approx(eul_div2[2], &s2, &c2);
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
  euler_rpy[0] = atan2_approx(q[0]*q[1] + q[2]*q[3], 0.5f - q[1]*q[1] - q[2]*q[2]) * rad_to_deg; //degrees - roll right is positive
  euler_rpy[1] = asin_approx(_constrain(-2.0f * (q[1]*q[3] - q[0]*q[2]), -1.0, 1.0)) * rad_to_deg; //degrees - pitch up is positive - use constrain() to prevent NaN due to rounding
  euler_rpy[2] = atan2_approx(q[1]*q[2] + q[0]*q[3], 0.5f - q[2]*q[2] - q[3]*q[3]) * rad_to_deg; //degrees - yaw right is positive
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
    float inv_norm = rsqrt_approx(norm2);
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

void quaternion_rotate(float result[3], const float q[4], const float v[3])
{
    float ww = q[0] * q[0];
    float xx = q[1] * q[1];
    float yy = q[2] * q[2];
    float zz = q[3] * q[3];
    float wx = q[0] * q[1];
    float wy = q[0] * q[2];
    float wz = q[0] * q[3];
    float xy = q[1] * q[2];
    float xz = q[1] * q[3];
    float yz = q[2] * q[3];

    result[0] = 
      + (1 - 2 * (yy + zz)) * v[0]
      + 2 * (xy - wz) * v[1]
      + 2 * (xz + wy) * v[2];
    result[1] = 
      + 2 * (xy + wz) * v[0]
      + (1 - 2 * (xx + zz)) * v[1]
      + 2 * (yz - wx) * v[2];
    result[2] = 
      + 2 * (xz - wy) * v[0]
      + 2 * (yz + wx) * v[1]
      + (1 - 2 * (xx + yy)) * v[2];
}
