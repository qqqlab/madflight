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
#include "../cfg/cfg.h"
#include "../imu/imu.h"
#include "../mag/mag.h"

struct AhrState {
  public:
    float gx = 0, gy = 0, gz = 0; // corrected and filtered imu gyro measurements in [deg/sec]
    float ax = 0, ay = 0, az = 0; // corrected and filtered imu accel measurements in [g]
    float mx = 0, my = 0, mz = 0; // corrected and filtered external magnetometer or internal imu mag measurements in [uT]
    float q[4] = {1,0,0,0};       // quaternion NED reference frame
    float roll = 0;               // roll in degrees: -180 to 180, roll right is positive
    float pitch = 0;              // pitch in degrees: -90 to 90, pitch up is positive
    float yaw = 0;                // yaw in degrees: -180 to 180, yaw right is positive
    uint32_t ts = 0;              // IMU sample timestamp [us]
    float dt = 0;              // time difference with ts from last update() call [s]
};

struct AhrConfig {
  public:
    Cfg::ahr_gizmo_enum gizmo = Cfg::ahr_gizmo_enum::mf_MAHONY; //the gizmo to use
    float gyrLpFreq = 1e10; //gyro low pass filter freq [Hz] (default to no filtering)
    float accLpFreq = 1e10; //accelerometer low pass filter freq [Hz] (default to no filtering)
    float magLpFreq = 1e10; //magnetometer low pass filter freq [Hz] (default to no filtering)
    Imu* pimu = nullptr; //pointer to Imu to use
    Mag* pmag = nullptr; //pointer to Mag to use
    float* gyr_offset = nullptr; //gyro offset[3] [deg/sec]
    float* acc_offset = nullptr; //acc offset[3] [G]
    float* mag_offset = nullptr; //mag offset[3] [adc_lsb]
    float* mag_scale = nullptr; //mag scale[3] [uT/adc_lsb]
};

class AhrGizmo {
  public:
    virtual ~AhrGizmo() {}
    virtual bool update() = 0; //returns true if new sample was taken
    virtual void setInitalOrientation(float *qnew) = 0; //set orientation to qnew
};

class Ahr : public AhrState {
  public:
    AhrConfig config;

    AhrGizmo *gizmo = nullptr;

    int setup();      // Use config to setup gizmo, returns 0 on success, or error code
    bool update(); //get imu+mag data, filter it, and call fusionUpdate() to update q. Returns true if state was updated
    bool installed() {return (gizmo != nullptr); } // Returns true if a gizmo was setup

    static constexpr float rad_to_deg = 57.2957795132f;
    static constexpr float deg_to_rad = 0.0174532925199f;

    void setInitalOrientation();
    float getAccelUp(); //get acceleration in earth-frame up direction in [m/s^2]

  protected:
    float B_gyr = 1.0;            // gyr filter constant
    float B_acc = 1.0;            // acc filter constant
    float B_mag = 1.0;            // mag filter constant

    void fusionUpdate();
    void computeAngles();
    void getQFromMag(float *q);

  private:
    RuntimeTrace runtimeTrace = RuntimeTrace("_AHR");
};

//Global module instance
extern Ahr ahr;
