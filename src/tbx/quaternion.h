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

#pragma once

//euler angles in Body 3-2-1 sequence: airplane first does yaw (Body-Z) turn during taxiing onto the runway, then pitches (Body-Y) during take-off, and finally rolls (Body-X) in the air
//see https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
void quaternion_from_euler_deg(float q[4], const float euler_rpy[3]);
void quaternion_to_euler_deg(float euler_rpy[3], const float q[4]); //q needs to be normalized before call
void quaternion_mult(float result[4], const float p[4], const float q[4]);
void quaternion_inverse(float result[4], const float q[4]);
void quaternion_inverse(float q[4]);
void quaternion_normalize(float q[4]);
void quaternion_init(float q[4]);
void quaternion_rotate(float result[3], const float q[4], const float v[3]);
