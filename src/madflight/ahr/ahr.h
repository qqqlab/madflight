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

#include <stdint.h>

class Ahr {
  public:
    float gx = 0, gy = 0, gz = 0; //corrected and filtered imu gyro measurements in [deg/sec]
    float ax = 0, ay = 0, az = 0; //corrected and filtered imu accel measurements in [g]
    float mx = 0, my = 0, mz = 0; //corrected and filtered external magnetometer or internal imu mag measurements in [uT]
    float q[4] = {1,0,0,0};  //quaternion NED reference frame
    float roll = 0;          //roll in degrees: -180 to 180, roll right is positive
    float pitch = 0;         //pitch in degrees: -90 to 90, pitch up is positive
    float yaw = 0;           //yaw in degrees: -180 to 180, yaw right is positive
    float B_gyr = 1.0; //gyr filter constant
    float B_acc = 1.0; //acc filter constant
    float B_mag = 1.0; //mag filter constant
    uint32_t ts = 0; //IMU sample timestamp

    static constexpr float rad_to_deg = 57.2957795132f;
    static constexpr float deg_to_rad = 0.0174532925199f;

    virtual void setup(float gyrLpFreq, float accLpFreq, float magLpFreq) = 0;
    virtual void setInitalOrientation() {}
    void update(); //get imu+mag data, filter it, and call fusionUpdate() to update q

    float getAccelUp(); //get acceleration in earth-frame up direction in [m/s^2]

  protected:
    virtual void fusionUpdate() = 0;
    void computeAngles();
    void setFromMag(float *q);
};

extern Ahr &ahr;
