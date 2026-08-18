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

#include "../madflight_modules.h"
#include "AhrGizmoMahony.h"
#include "AhrGizmoMadgwick.h"
#include "AhrGizmoVqf.h"
#include "../tbx/quaternion.h"

//create global module instance
Ahr ahr;

int Ahr::setup() {
  cfg.printModule(MF_MOD);

  for(int axis = 0; axis < 3; axis++) {
    MF_Filter::setup(gyr_filter[axis], MF_FilterType::mf_PT1, config.pimu->getSampleRate(), config.imu_gyr_lp, 0);
    MF_Filter::setup(acc_filter[axis], MF_FilterType::mf_PT1, config.pimu->getSampleRate(), config.imu_acc_lp, 0);
    MF_Filter::setup(mag_filter[axis], MF_FilterType::mf_PT1, config.pimu->getSampleRate(), config.mag_lp,     0);
  }

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
    default:
      gizmo = new AhrGizmoMahony(this, false);
      break;
  }

  return 0;
}

bool Ahr::update() {
  runtimeTrace.start();

  // get sensor data from imu and mag
  // correct the sensor data with the calibration values
  // use simple first-order low-pass filter to get rid of high frequency noise
  // store filtered data in ax ay az gx gy gz mx my mz
  // call the sensor fusion algorithm to update q
  // compute euler angles from q

  //Low-pass filtered, corrected accelerometer data
  ax = acc_filter[0]->apply(config.pimu->ax);
  ay = acc_filter[1]->apply(config.pimu->ay);
  az = acc_filter[2]->apply(config.pimu->az);

  //Low-pass filtered, corrected gyro data
  gx = gyr_filter[0]->apply(config.pimu->gx);
  gy = gyr_filter[1]->apply(config.pimu->gy);
  gz = gyr_filter[2]->apply(config.pimu->gz);

  //Magnetometer (External chip, or internal in IMU chip) 
  if(config.pmag) {
    float _mx = config.pmag->mx;
    float _my = config.pmag->my;
    float _mz = config.pmag->mz;
    //update the mag values
    if( ! (_mx == 0 && _my == 0 && _mz == 0) ) {
      //Low-pass filtered magnetometer data
      mx = mag_filter[0]->apply(_mx);
      my = mag_filter[1]->apply(_my);
      mz = mag_filter[2]->apply(_mz);
    }
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

  //update euler angles
  quaternion_to_euler_deg(&roll, q);

  topic.publish(this);

  runtimeTrace.stop(true);
  return true;
}

void Ahr::getQFromMag(float *q) {
  //warm up mag by getting 100 samples within max 1 second (imu + mag should be running already)
  uint32_t ts = millis();
  do {
    portYIELD();
  } while( mag.topic.get_generation() < 100 && millis() - ts < 1000 );

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

//get acceleration in earth-frame up direction in [m/s^2]
float Ahr::getAccelUp() {
  return 9.80665 * ((2*q[1]*q[3] - 2*q[0]*q[2])*ax + (2*q[2]*q[3] + 2*q[0]*q[1])*ay + (q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3])*az - 1.0);
}

//get acceleration in NED earth-frame [m/s^2]
void Ahr::get_acc_ned(float acc[3]) {
  quaternion_rotate(acc, q, &ax);
  acc[0] *= 9.80665;
  acc[1] *= 9.80665;
  acc[2]  = 9.80665 * (acc[2] - 1); //remove gravity
}

void Ahr::setInitalOrientation() {
  if(!gizmo) return;
  float qnew[4];
  getQFromMag(qnew);
  gizmo->setInitalOrientation(qnew);
}