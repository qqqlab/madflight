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

#define MF_MOD "MAG"

#include <Arduino.h> //Serial
#include "mag.h"
#include "MagGizmoQMC5883L.h"
#include "MagGizmoQMC6309.h"
#include "MagGizmoRM3100.h"
#include "MagGizmoQMC5883P.h"
#include "MagGizmoMMC5603.h"
#include "MagGizmoBMM150.h"

//create global module instance
Mag mag;

int Mag::setup() {
  cfg.printModule(MF_MOD);

  _samplePeriod = 1000000 / config.sample_rate;

  //clear state
   mx = 0; //"North" magnetic flux [uT]
   my = 0; //"East" magnetic flux [uT]
   mz = 0; //"Down" magnetic flux [uT]
   ts = 0; //last sample time in [us]

  //create gizmo
  delete gizmo;
  switch(config.gizmo) {
    case Cfg::mag_gizmo_enum::mf_NONE :
      gizmo = nullptr;
      break;
    case Cfg::mag_gizmo_enum::mf_QMC5883L :
    case Cfg::mag_gizmo_enum::mf_QMC5883 :
      if(config.i2c_bus) {
        gizmo = new MagGizmoQMC5883L(config.i2c_bus, config.i2c_adr);
      }
      break;
    case Cfg::mag_gizmo_enum::mf_QMC6309 :
      if(config.i2c_bus) {
        gizmo = new MagGizmoQMC6309(config.i2c_bus); //i2c address is always 0x7C
      }
      break;
    case Cfg::mag_gizmo_enum::mf_RM3100 :
      gizmo = MagGizmoRM3100::create(config.i2c_bus);
      break;
    case Cfg::mag_gizmo_enum::mf_QMC5883P :
      if(config.i2c_bus) {
        gizmo = new MagGizmoQMC5883P(config.i2c_bus); //i2c address is always 0x2C
      }
      break;
    case Cfg::mag_gizmo_enum::mf_MMC5603 :
      gizmo = MagGizmoMMC5603::create(&config, (MagState*)this); //i2c address is always 0x30
      break;
    case Cfg::mag_gizmo_enum::mf_BMM150 :
      gizmo = MagGizmoBMM150::create(&config, (MagState*)this);
      break;
  }

  //check gizmo
  if(!gizmo && config.gizmo != Cfg::mag_gizmo_enum::mf_NONE) {
    cfg.printModule(MF_MOD, CfgClass::printModuleMode::CFG_ERROR);
    return -1001;
  }

  return 0;
}

bool Mag::update() {
  runtimeTrace.start();
  bool updated = (gizmo != nullptr);
  updated = updated && schedule.interval(_samplePeriod); //wait for next sample interval

  float nwu[3]; //uncalibrated NWU values
  updated = updated && gizmo->update_nwu(&nwu[0], &nwu[1], &nwu[2]);

  //Correct the NWU mag values with the calibration values, and convert to NED (N=+N, E=-W, D=-U)
  float _mx = +(nwu[0] - config.mag_cal_x[0]) * config.mag_cal_sx[0];
  float _my = -(nwu[1] - config.mag_cal_x[1]) * config.mag_cal_sx[1];
  float _mz = -(nwu[2] - config.mag_cal_x[2]) * config.mag_cal_sx[2];

  if(updated) {
    //handle rotation for different mounting positions
    switch(*config.mag_align) {
      case Cfg::mag_align_enum::mf_CW0 :
        mx = +_mx;
        my = +_my; 
        mz = +_mz;
        break;
      case Cfg::mag_align_enum::mf_CW90 :
        mx = -_my;
        my = +_mx; 
        mz = +_mz;
        break;
      case Cfg::mag_align_enum::mf_CW180 :
        mx = -_mx;
        my = -_my;
        mz = +_mz;
        break;
      case Cfg::mag_align_enum::mf_CW270 :
        mx = +_my;
        my = -_mx;
        mz = +_mz;
        break;
      case Cfg::mag_align_enum::mf_CW0FLIP :
        mx = +_mx;
        my = -_my; 
        mz = -_mz;
        break;
      case Cfg::mag_align_enum::mf_CW90FLIP :
        mx = +_my;
        my = +_mx;
        mz = -_mz;
        break;
      case Cfg::mag_align_enum::mf_CW180FLIP :
        mx = -_mx;
        my = +_my;
        mz = -_mz;
        break;
      case Cfg::mag_align_enum::mf_CW270FLIP :
        mx = -_my;
        my = -_mx;
        mz = -_mz;
        break;
    }

    ts = micros();

    topic.publish(this);
  }

  runtimeTrace.stop(updated);
  return updated;
}

void Mag::convert_to_raw(float m[3]) {
  //unrotate
  float b[3];
  switch(*config.mag_align) {
    case Cfg::mag_align_enum::mf_CW0 :
      b[0] = +m[0]; //mx = +_mx;
      b[1] = +m[1]; //my = +_my; 
      b[2] = +m[2]; //mz = +_mz;
      break;
    case Cfg::mag_align_enum::mf_CW90 :
      b[1] = -m[0]; //mx = -_my;
      b[0] = +m[1]; //my = +_mx; 
      b[2] = +m[2]; //mz = +_mz;
      break;
    case Cfg::mag_align_enum::mf_CW180 :
      b[0] = -m[0]; //mx = -_mx;
      b[1] = -m[1]; //my = -_my;
      b[2] = +m[2]; //mz = +_mz;
      break;
    case Cfg::mag_align_enum::mf_CW270 :
      b[1] = +m[0]; //mx = +_my;
      b[0] = -m[1]; //my = -_mx;
      b[2] = +m[2]; //mz = +_mz;
      break;
    case Cfg::mag_align_enum::mf_CW0FLIP :
      b[0] = +m[0]; //mx = +_mx;
      b[1] = -m[1]; //my = -_my; 
      b[2] = -m[2]; //mz = -_mz;
      break;
    case Cfg::mag_align_enum::mf_CW90FLIP :
      b[1] = +m[0]; //mx = +_my;
      b[0] = +m[1]; //my = +_mx;
      b[2] = -m[2]; //mz = -_mz;
      break;
    case Cfg::mag_align_enum::mf_CW180FLIP :
      b[0] = -m[0]; //mx = -_mx;
      b[1] = +m[1]; //my = +_my;
      b[2] = -m[2]; //mz = -_mz;
      break;
    case Cfg::mag_align_enum::mf_CW270FLIP :
      b[1] = -m[0]; //mx = -_my;
      b[0] = -m[1]; //my = -_mx;
      b[2] = -m[2]; //mz = -_mz;
      break;
  }

  //Undo: Correct the NWU mag values with the calibration values, and convert to NED (N=+N, E=-W, D=-U)
  m[0] = (+b[0] / config.mag_cal_sx[0] + config.mag_cal_x[0]); //float _mx = +(raw[0] - config.mag_cal_x[0]) * config.mag_cal_sx[0];
  m[1] = (-b[1] / config.mag_cal_sx[1] + config.mag_cal_x[1]); //float _my = -(raw[1] - config.mag_cal_x[1]) * config.mag_cal_sx[1];
  m[2] = (-b[2] / config.mag_cal_sx[2] + config.mag_cal_x[2]); //float _mz = -(raw[2] - config.mag_cal_x[2]) * config.mag_cal_sx[2];
}


void Mag::cli_calibrate() {
  float bias[3], scale[3];

  Serial.printf("Magnetometer %s calibration. Rotate the IMU about all axes until complete.\n", mag.name());
  if ( _calibrate(bias, scale) ) {
    Serial.println("Calibration Successful!");
    Serial.printf("set mag_cal_x  %+f #config was %+f\n", bias[0], config.mag_cal_x[0]);
    Serial.printf("set mag_cal_y  %+f #config was %+f\n", bias[1], config.mag_cal_x[1]);
    Serial.printf("set mag_cal_z  %+f #config was %+f\n", bias[2], config.mag_cal_x[2]);
    Serial.printf("set mag_cal_sx %+f #config was %+f\n", scale[0], config.mag_cal_sx[0]);
    Serial.printf("set mag_cal_sy %+f #config was %+f\n", scale[1], config.mag_cal_sx[1]);
    Serial.printf("set mag_cal_sz %+f #config was %+f\n", scale[2], config.mag_cal_sx[2]);
    Serial.println("Note: type 'save' to save these values to flash");
    Serial.println(" ");
    Serial.println("If you are having trouble with your attitude estimate at a new flying location, repeat this process as needed.");
  } else {
    Serial.println("ERROR: No magnetometer");
  }

  //save new calibration values
  for(int axis = 0; axis < 3; axis++) {
    config.mag_cal_x[axis] = bias[axis];
    config.mag_cal_sx[axis] = scale[axis];
  }
}

// finds bias and scale factor calibration for the magnetometer, the sensor should be rotated in a figure 8 motion until complete
// Note: Earth's field ranges between approximately 25 and 65 uT. (Europe & USA: 45-55 uT, inclination 50-70 degrees)
bool Mag::_calibrate(float bias[3], float scale[3]) {
  const int maxCounts = 1000; //sample for at least 10 seconds @ 100Hz
  const float count_reduction_factor = 0.7; // reduce counter when a min/max changed by at least 10% of current min-max range

  float mlast[3] = {0};
  int counter;
  float m_max[3];
  float m_min[3];

  //exit if no mag present
  if(!mag.installed()) return false;

  //start subscription
  auto mag_sub = MsgSubscription<MagState>("calmag", &mag.topic);
  MagState state;
  float *m = &state.mx;

  // get starting sample
  uint32_t ts = millis();
  while(millis() - ts < 1000) {
    if(mag_sub.pull_next(&state)) {
      convert_to_raw(m);
      break;
    }
  }

  //save starting data
  for(int axis = 0; axis < 3; axis++) {
    mlast[axis] = m[axis];
    m_max[axis] = m[axis];
    m_min[axis] = m[axis];
  }

  // collect data to find max / min in each channel
  // sample counter times, reduce counter when a min/max changed by at least 10% of current min-max range
  uint32_t progress_time = millis() - 1000;
  counter = 0;
  while (counter < maxCounts) {
    if(mag_sub.pull_next(&state)) {
      convert_to_raw(m);
      if ( m[0] != mlast[0] && m[1] != mlast[1] && m[2] != mlast[2] && m[0] != 0  && m[1] != 0 && m[2] != 0) { //value changed and is not 0,0,0
        for(int axis = 0; axis < 3; axis++) {
          float range_limit = 0.1 * (m_max[axis] - m_min[axis]);
          if(m_min[axis] > m[axis]) {
            if(m_min[axis] - m[axis] > range_limit) counter *= count_reduction_factor;
            m_min[axis] = m[axis];
          }
          if(m_max[axis] < m[axis]) {
            if(m[axis] - m_max[axis] > range_limit) counter *= count_reduction_factor;
            m_max[axis] = m[axis];
          }
          mlast[axis] = m[axis];
        }
        counter++;
      }
    }
    
    //print progress
    if (millis() - progress_time > 1000) {
      progress_time = millis();
      Serial.printf("done:%2d%%\txmin:%+.2f\txmax:%+.2f\tymin:%+.2f\tymax:%+.2f\tzmin:%+.2f\tzmax:%+.2f\n", counter * 100 / maxCounts, m_min[0], m_max[0], m_min[1], m_max[1], m_min[2], m_max[2]);
    }
  }

  // find the magnetometer bias and scale
  float avg_scale = 0;
  for(int axis = 0; axis < 3; axis++) { 
    bias[axis] = (m_max[axis] + m_min[axis]) / 2;
    scale[axis] = (m_max[axis] - m_min[axis]) / 2;
    avg_scale += scale[axis];
  }
  for(int axis = 0; axis < 3; axis++) {
    scale[axis] = (avg_scale / 3) / scale[axis];
  }

  return true;
}
