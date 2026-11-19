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
#include "../tbx/tbx.h" //RuntimeTrace, MsgBroker
#include "../tbx/MF_Filter.h"

struct __attribute__((aligned(4))) AhrState {
  public:
    float gx = 0, gy = 0, gz = 0; // corrected and filtered imu gyro measurements in [deg/sec]
    float ax = 0, ay = 0, az = 0; // corrected and filtered imu accel measurements in [g]
    float mx = 0, my = 0, mz = 0; // corrected and filtered external magnetometer or internal imu mag measurements in [uT]
    float q[4] = {1,0,0,0};       // quaternion NED reference frame
    float roll = 0;               // roll in degrees: -180 to 180, roll right is positive
    float pitch = 0;              // pitch in degrees: -90 to 90, pitch up is positive
    float yaw = 0;                // yaw in degrees: -180 to 180, yaw right is positive
    uint32_t ts = 0;              // IMU sample timestamp [us]
    float dt = 0;                 // time difference with ts from last update() call [s]

//struct __attribute__((aligned(4))) InsFusionState {
//  uint32_t ts = 0;          // IMU timestamp (low 32 bit) of this solution [us]
  uint8_t mode = 0;         // nav_suite_mode_t: 0 none, 1 attitude only, 2 coasting, 3 full
  bool ready = false;       // INS position/velocity solution usable
  bool gnss_used = false;   // a GNSS fix was fused in the last epoch

  float vel_ned[3] = {0};   // INS velocity NED [m/s]
  float vel_up = 0;         // INS climb rate = -vel_ned[2] [m/s]
  bool  vel_up_valid = false;
  float baro_v_up = 0;      // baro_alt vertical-channel climb rate [m/s]
  bool  baro_v_up_valid = false;
  float baro_h = 0;         // baro_alt filter height relative to start [m] (same filter as baro_v_up)
  float baro_h_isa = 0;     // baro_alt filter height as ISA (barometric) altitude [m] - datum independent, never GNSS-corrected
  bool  baro_isa_valid = false;

  float pos_ned[3] = {0};   // local NED position relative to start [m]
  float height = 0;         // best local height above start, positive up [m]
  bool  height_valid = false;

//  float q[4] = {1, 0, 0, 0}; // body->NED quaternion (w,x,y,z)
//  float roll = 0, pitch = 0, yaw = 0; // [deg]
  bool  att_valid = false;

  float acc_bias[3] = {0};  // estimated accelerometer bias, body FRD [m/s^2]

  // GNSS-aided absolute position (valid only in FULL mode, i.e. while GNSS is being fused)
  bool   pos_valid = false;
  double lat_deg = 0, lon_deg = 0;   // WGS84
  float  h_ell = 0;                  // ellipsoid height [m]
  bool   h_ell_valid = false;        // also valid during short GNSS outages (local height + estimated offset)
  float  h_msl = 0;                  // ellipsoid height + geoid undulation [m] (needs gps.have_undulation, else = h_ell)
};

struct AhrConfig {
  public:
    Cfg::ahr_gizmo_enum gizmo = Cfg::ahr_gizmo_enum::mf_MAHONY; //the gizmo to use
    float imu_gyr_lp = 1e10; //gyro low pass filter freq [Hz] (default to no filtering)
    float imu_acc_lp = 1e10; //accelerometer low pass filter freq [Hz] (default to no filtering)
    float mag_lp = 1e10; //magnetometer low pass filter freq [Hz] (default to no filtering)
    Imu* pimu = nullptr; //pointer to Imu to use
    Mag* pmag = nullptr; //pointer to Mag to use
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
    MsgTopic<AhrState> topic = MsgTopic<AhrState>("ahr");

    int setup();      // Use config to setup gizmo, returns 0 on success, or error code
    bool update(); //get imu+mag data, filter it, and call fusionUpdate() to update q. Returns true if state was updated
    bool installed() {return (gizmo != nullptr); } // Returns true if a gizmo was setup

    static constexpr float rad_to_deg = 57.2957795132f;
    static constexpr float deg_to_rad = 0.0174532925199f;

    void setInitalOrientation();
    float getAccelUp(); //get acceleration in earth-frame up direction in [m/s^2]
    void get_acc_ned(float acc[3]); //get acceleration in NED earth-frame [m/s^2]

  protected:
    MF_Filter *gyr_filter[3] = {new MF_FilterLowPT1(), new MF_FilterLowPT1(), new MF_FilterLowPT1()};
    MF_Filter *acc_filter[3] = {new MF_FilterLowPT1(), new MF_FilterLowPT1(), new MF_FilterLowPT1()};
    MF_Filter *mag_filter[3] = {new MF_FilterLowPT1(), new MF_FilterLowPT1(), new MF_FilterLowPT1()};

    void fusionUpdate();
    void getQFromMag(float *q);

  private:
    RuntimeTrace runtimeTrace = RuntimeTrace("AHR");
};

//Global module instance
extern Ahr ahr;
