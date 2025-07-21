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

#define MF_MOD "AHR"

#include <Arduino.h> //Serial
#include <math.h>
#include "ahr.h"
#include "AhrGizmoMahony.h"
#include "AhrGizmoMadgwick.h"
#include "AhrGizmoVqf.h"
#include "AhrGizmoImu.h"

#include "../mag/mag.h"
#include "../imu/imu.h"
#include "../cfg/cfg.h"
#include "../tbx/common.h" //lowpass_to_beta

//create global module instance
Ahr ahr;

int Ahr::setup() {
  cfg.printModule(MF_MOD);

  B_gyr = lowpass_to_beta(config.gyrLpFreq, config.pimu->getSampleRate());
  B_acc = lowpass_to_beta(config.accLpFreq, config.pimu->getSampleRate());
  B_mag = lowpass_to_beta(config.magLpFreq, config.pimu->getSampleRate());

  //create gizmo
  delete gizmo;
  switch(config.gizmo) {
    case Cfg::ahr_gizmo_enum::mf_MAHONY :
      gizmo = new AhrGizmoMahony(this, false);
      break;
    case Cfg::ahr_gizmo_enum::mf_MAHONY_BF :
      gizmo = new AhrGizmoMahony(this, true);
      break;
    case Cfg::ahr_gizmo_enum::mf_MADGWICK :
      gizmo = new AhrGizmoMadgwick(this);
      break;
    case Cfg::ahr_gizmo_enum::mf_VQF :
      gizmo = new AhrGizmoVqf(this);
      break;
    case Cfg::ahr_gizmo_enum::mf_IMU :
      gizmo = new AhrGizmoImu(this);
      if (!config.pimu->hasSensorFusion()) {
        Serial.printf("Cannot use mf_IMU for AHR with an IMU that does not support sensor fusion!\n");
        return -1;
      }
      break;
    default:
      gizmo = new AhrGizmoMahony(this, false);
      break;
  }

  return 0;
}

bool Ahr::update() {
  // get sensor data from imu and mag
  // correct the sensor data with the calibration values
  // use simple first-order low-pass filter to get rid of high frequency noise
  // store filtered data in ax ay az gx gy gz mx my mz
  // call the sensor fusion algorithm to update q
  // compute euler angles from q

  //Low-pass filtered, corrected accelerometer data
  ax += B_acc * ((config.pimu->ax - config.acc_offset[0]) - ax);
  ay += B_acc * ((config.pimu->ay - config.acc_offset[1]) - ay);
  az += B_acc * ((config.pimu->az - config.acc_offset[2]) - az);

  //Low-pass filtered, corrected gyro data
  gx += B_gyr * ((config.pimu->gx - config.gyr_offset[0]) - gx);
  gy += B_gyr * ((config.pimu->gy - config.gyr_offset[1]) - gy);
  gz += B_gyr * ((config.pimu->gz - config.gyr_offset[2]) - gz);

  //Magnetometer (External chip, or internal in IMU chip) 
  float _mx, _my, _mz;
  //If no external mag, then use internal mag
  if(!config.pmag || (config.pmag->x == 0 && config.pmag->y == 0 && config.pmag->z == 0)) {
    _mx = config.pimu->mx;
    _my = config.pimu->my;
    _mz = config.pimu->mz;
  }else{
    _mx = config.pmag->x;
    _my = config.pmag->y;
    _mz = config.pmag->z;
  }
  //update the mag values
  if( ! (_mx == 0 && _my == 0 && _mz == 0) ) {
    //Correct the mag values with the calibration values
    _mx = (_mx - config.mag_offset[0]) * config.mag_scale[0];
    _my = (_my - config.mag_offset[1]) * config.mag_scale[1];
    _mz = (_mz - config.mag_offset[2]) * config.mag_scale[2];
    //Low-pass filtered magnetometer data
    mx += B_mag * (_mx - mx);
    my += B_mag * (_my - my);
    mz += B_mag * (_mz - mz);
  }else{
    mx = 0;
    my = 0;
    mz = 0;
  }

  //update dt and ts for use by gizmo
  dt = (config.pimu->ts - ts) * 1e-6;
  ts = config.pimu->ts;

  //call gizmo to update q
  gizmo->update();

  // Apply q_bias to correct the IMU quaternion if using sensor fusion
  if (config.pimu->hasSensorFusion()) {
    // Apply q_bias to correct the IMU quaternion
    float qb0 = cfg.imu_cal_q0_bias;
    float qb1 = cfg.imu_cal_q1_bias;
    float qb2 = cfg.imu_cal_q2_bias;
    float qb3 = cfg.imu_cal_q3_bias;

    float q0 = q[0];
    float q1 = q[1];
    float q2 = q[2];
    float q3 = q[3];

    // q = q_bias * q
    q[0] = qb0*q0 - qb1*q1 - qb2*q2 - qb3*q3;
    q[1] = qb0*q1 + qb1*q0 + qb2*q3 - qb3*q2;
    q[2] = qb0*q2 - qb1*q3 + qb2*q0 + qb3*q1;
    q[3] = qb0*q3 + qb1*q2 - qb2*q1 + qb3*q0;
  }

  //update euler angles
  computeAngles();
  return true;
}

void Ahr::getQFromMag(float *q) {
  //warm up mag by getting 100 samples (imu should be running already)
  for(int i=0;i<100;i++) {
    uint32_t start = micros();
    config.pmag->update();
    while(micros() - start < 1000000 / config.pimu->getSampleRate()); //wait until next sample time
  }

  //update mx and my from mag or imu
  update();

  //calculate yaw angle
  if(mx == 0 && my == 0 && mz == 0) {
    yaw = 0;
    pitch = 0;
    roll = 0;
    q[0] = 1;
    q[1] = 0;
    q[2] = 0;
    q[3] = 0;
    Serial.println("AHR: No Magnetometer, yaw:0.00");
  }else{
    float yaw_rad = -atan2(my, mx);
    yaw = yaw_rad * rad_to_deg;
    pitch = 0;
    roll = 0;
    q[0] = cos(yaw_rad/2);
    q[1] = 0;
    q[2] = 0;
    q[3] = sin(yaw_rad/2);
    Serial.printf("AHR: Estimated yaw:%+.2f\n", yaw);
  }
  
  //set dt,ts
  dt = 0;
  ts = micros();
}

//compute euler angles from q
void Ahr::computeAngles() {
  // standard ZYX (yaw-pitch-roll) from quaternion
  roll  = atan2(2.0f * (q[0]*q[1] + q[2]*q[3]), 1.0f - 2.0f * (q[1]*q[1] + q[2]*q[2])) * rad_to_deg;
  pitch = asin (constrain(2.0f * (q[0]*q[2] - q[3]*q[1]), -1.0f, 1.0f)) * rad_to_deg;
  yaw = atan2(2.0f * (q[0]*q[3] + q[1]*q[2]), 1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3])) * rad_to_deg;
}

//get acceleration in earth-frame up direction in [m/s^2]
float Ahr::getAccelUp() {
  return 9.80665 * ((2*q[1]*q[3] - 2*q[0]*q[2])*ax + (2*q[2]*q[3] + 2*q[0]*q[1])*ay + (q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3])*az - 1.0);
}

void Ahr::setInitalOrientation() {
  if(!gizmo) return;
  float qnew[4];
  getQFromMag(qnew);
  gizmo->setInitalOrientation(qnew);
}