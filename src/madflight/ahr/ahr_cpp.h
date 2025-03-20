//This file is kept as header to allow #AHR_xxx defines to be added in user code (Arduino IDE has no easy way to add -D compiler options)

#ifndef MF_ALLOW_INCLUDE_CCP_H
  #error "Only include this file from madflight.h"
#endif

#define AHR_USE_MAHONY 1 //default when AHR_USE is not defined
#define AHR_USE_MAHONY_BF 2 //betaflight flavored: only use accel if near 1G
#define AHR_USE_MADGWICK 3
#define AHR_USE_VQF 4

#include "ahr.h" //AHR class definition
#include "../mag/mag.h"
#include "../imu/imu.h"
#include "../cfg/cfg.h"
#include "../tbx/common.h" //lowpass_to_beta

void Ahr::update() {
  // get sensor data from imu and mag
  // correct the sensor data with the calibration values
  // use simple first-order low-pass filter to get rid of high frequency noise
  // store filtered data in ax ay az gx gy gz mx my mz
  // call the sensor fusion algorithm to update q
  // compute euler angles from q
  
  //Low-pass filtered, corrected accelerometer data
  ax += B_acc * ((imu.ax - cfg.imu_cal_ax) - ax);
  ay += B_acc * ((imu.ay - cfg.imu_cal_ay) - ay);
  az += B_acc * ((imu.az - cfg.imu_cal_az) - az);

  //Low-pass filtered, corrected gyro data
  gx += B_gyr * ((imu.gx - cfg.imu_cal_gx) - gx);
  gy += B_gyr * ((imu.gy - cfg.imu_cal_gy) - gy);
  gz += B_gyr * ((imu.gz - cfg.imu_cal_gz) - gz);

  //External Magnetometer 
  float _mx = mag.x;
  float _my = mag.y;
  float _mz = mag.z;
  //If no external mag, then use internal mag
  if(_mx == 0 && _my == 0 && _mz == 0) {
    _mx = imu.mx;
    _my = imu.my;
    _mz = imu.mz;
  }
  //update the mag values
  if( ! (_mx == 0 && _my == 0 && _mz == 0) ) {
    //Correct the mag values with the calibration values
    _mx = (_mx - cfg.mag_cal_x) * cfg.mag_cal_sx;
    _my = (_my - cfg.mag_cal_y) * cfg.mag_cal_sy;
    _mz = (_mz - cfg.mag_cal_z) * cfg.mag_cal_sz;
    //Low-pass filtered magnetometer data
    mx += B_mag * (_mx - mx);
    my += B_mag * (_my - my);
    mz += B_mag * (_mz - mz);
  }else{
    mx = 0;
    my = 0;
    mz = 0;
  }
  
  //call fusionUpdate to update q
  fusionUpdate();
  
  //update euler angles
  computeAngles();
  
  ts = imu.ts;
}

void Ahr::setFromMag(float *q) {
  //warm up mag by getting 100 samples (imu should be running already)
  for(int i=0;i<100;i++) {
    uint32_t start = micros();
    mag.update();
    while(micros() - start < 1000000 / imu.getSampleRate()); //wait until next sample time
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
}

//compute euler angles from q
void Ahr::computeAngles() {
  roll = atan2(q[0]*q[1] + q[2]*q[3], 0.5f - q[1]*q[1] - q[2]*q[2]) * rad_to_deg; //degrees - roll right is positive
  pitch = asin(constrain(-2.0f * (q[1]*q[3] - q[0]*q[2]), -1.0, 1.0)) * rad_to_deg; //degrees - pitch up is positive - use constrain() to prevent NaN due to rounding
  yaw = atan2(q[1]*q[2] + q[0]*q[3], 0.5f - q[2]*q[2] - q[3]*q[3]) * rad_to_deg; //degrees - yaw right is positive
}

//get acceleration in earth-frame up direction in [m/s^2]
float Ahr::getAccelUp() {
  return 9.80665 * ((2*q[1]*q[3] - 2*q[0]*q[2])*ax + (2*q[2]*q[3] + 2*q[0]*q[1])*ay + (q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3])*az - 1.0);
}


//=================================================================================================
// Mahony or undefined
//=================================================================================================
#if AHR_USE == AHR_USE_MAHONY || AHR_USE == AHR_USE_MAHONY_BF || !defined AHR_USE

#include "mahony/mahony.h"
class AhrMahony : public Ahr {
public:
  void setup(float gyrLpFreq, float accLpFreq, float magLpFreq) {
    #if AHR_USE == AHR_USE_MAHONY_BF
      Serial.println("AHR: MAHONY_BF");
    #else
      Serial.println("AHR: MAHONY");
    #endif

    B_gyr = lowpass_to_beta(gyrLpFreq, imu.getSampleRate());
    B_acc = lowpass_to_beta(accLpFreq, imu.getSampleRate());
    B_mag = lowpass_to_beta(magLpFreq, imu.getSampleRate());
  }

  //estimate yaw based on mag only (assumes vehicle is horizontal)  
  void setInitalOrientation() {
    float qnew[4];
    setFromMag(qnew);
    q0 = qnew[0];
    q1 = qnew[1];
    q2 = qnew[2];
    q3 = qnew[3];
  }

protected:
  void fusionUpdate() {
    ahrs_Mahony(gx*deg_to_rad, gy*deg_to_rad, gz*deg_to_rad, ax, ay, az, mx, my, mz, imu.dt);
    q[0] = q0;
    q[1] = q1;
    q[2] = q2;
    q[3] = q3;
  }
};
AhrMahony ahr_instance;

//=================================================================================================
// Madgwick
//=================================================================================================
#elif AHR_USE == AHR_USE_MADGWICK

#include "madgwick/madgwick.h"
class AhrMadgwick : public Ahr {
public:
  void setup(float gyrLpFreq, float accLpFreq, float magLpFreq) {
    Serial.println("AHR: MADGWICK");
    
    B_gyr = lowpass_to_beta(gyrLpFreq, imu.getSampleRate());
    B_acc = lowpass_to_beta(accLpFreq, imu.getSampleRate());
    B_mag = lowpass_to_beta(magLpFreq, imu.getSampleRate());
  }

  //estimate yaw based on mag only (assumes vehicle is horizontal)  
  void setInitalOrientation() {
    float qnew[4];
    setFromMag(qnew);
    q0 = qnew[0];
    q1 = qnew[1];
    q2 = qnew[2];
    q3 = qnew[3];
  }

protected:
  void fusionUpdate() {
    ahrs_Madgwick(gx*deg_to_rad, gy*deg_to_rad, gz*deg_to_rad, ax, ay, az, mx, my, mz, imu.dt);
    q[0] = q0;
    q[1] = q1;
    q[2] = q2;
    q[3] = q3;
  }
};
AhrMadgwick ahr_instance;

//=================================================================================================
// VQF - see https://github.com/dlaidig/vqf
//=================================================================================================
#elif AHR_USE == AHR_USE_VQF

#include "vqf/vqf.h"
class AhrVqf : public Ahr {
public:
  void setup(float gyrLpFreq, float accLpFreq, float magLpFreq) {
    Serial.println("AHR: VQF");

    //do not use filter for VQF, feed the data directly into the filter
    (void)gyrLpFreq;
    (void)accLpFreq;
    (void)magLpFreq;
    B_gyr = 1.0;
    B_acc = 1.0;
    B_mag = 1.0;

    initVqf(1.0/imu.getSampleRate(), 1.0/imu.getSampleRate(), 1.0/imu.getSampleRate());
  }

  //TODO (?)
  void setInitalOrientation() {}

protected:
  void fusionUpdate() {
    //convert to ENU from NED: xE = yE, yN = xE, zU = -zD 
    float gyr[3] = {gy*deg_to_rad, gx*deg_to_rad, -gz*deg_to_rad}; //gyr in rad/sec
    float acc[3] = {ay*9.81f, ax*9.81f, -az*9.81f}; //acc in m/s/s -  acc[] horizontal: [0, 0, -9.81], nose down:[0, 9.81, 0], right down: [9.81, 0, 0]
    float mag[3] = {mx, -my, mz}; //mag no unit - mag[] horizontal pointing north: [15, 0, 40], east: [0,15,40]

    //get fused q_enu in ENU frame
    float q_enu[4];
    if (mx==0 && my==0 && mz==0) {
      updateGyr(gyr);
      updateAcc(acc);
      getQuat6D(q_enu);
    }else{
      updateGyr(gyr);
      updateAcc(acc);
      updateMag(mag);
      getQuat9D(q_enu);
    }
    
    //convert q_enu to NED reference frame
    float q_trans[4] = {0,M_SQRT2/2,M_SQRT2/2,0}; //ENU to NED
    quatMultiply(q_enu, q_trans, q);

  }

  static void quatMultiply(const vqf_real_t q1[4], const vqf_real_t q2[4], vqf_real_t out[4])
  {
      vqf_real_t w = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
      vqf_real_t x = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
      vqf_real_t y = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
      vqf_real_t z = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
      out[0] = w; out[1] = x; out[2] = y; out[3] = z;
  }

};

AhrVqf ahr_instance;
//=================================================================================================
// Invalid value
//=================================================================================================
#else
  #error "invalid AHR_USE value"
#endif


//global Ahr class instance
Ahr &ahr = ahr_instance;
