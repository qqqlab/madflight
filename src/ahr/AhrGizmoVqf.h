//=================================================================================================
// VQF - see https://github.com/dlaidig/vqf
//=================================================================================================

//NOTE: only one AhrGizmoVqf instance is allowed

//NOTE: do not use filter for VQF, feed the data directly into the filter, i.e. set gyrLpFreq, accLpFreq, magLpFreq to 1e10

#pragma once

#include "ahr.h"
#include "vqf/vqf.h"

class AhrGizmoVqf : public AhrGizmo {
private:
  AhrConfig *config;
  AhrState *state;

public:
  AhrGizmoVqf(Ahr *ahr) {
    this->config = &(ahr->config);
    this->state = (AhrState*)ahr;

    initVqf(1.0/config->pimu->getSampleRate(), 1.0/config->pimu->getSampleRate(), 1.0/config->pimu->getSampleRate());
  }

  void setInitalOrientation(float *qnew) {
    //TODO (?)
  }

  bool update() {
    //convert to ENU from NED: xE = yE, yN = xE, zU = -zD 
    float gyr[3] = {state->gy * Ahr::deg_to_rad,  state->gx * Ahr::deg_to_rad, -state->gz * Ahr::deg_to_rad}; //gyr in rad/sec
    float acc[3] = {state->ay * 9.81f,            state->ax * 9.81f,           -state->az * 9.81f};           //acc in m/s/s -  acc[] horizontal: [0, 0, -9.81], nose down:[0, 9.81, 0], right down: [9.81, 0, 0]
    float mag[3] = {state->mx,                   -state->my,                    state->mz};                   //mag no unit - mag[] horizontal pointing north: [15, 0, 40], east: [0,15,40]

    //get fused q_enu in ENU frame
    float q_enu[4];
    if (state->mx==0 && state->my==0 && state->mz==0) {
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
    quatMultiply(q_enu, q_trans, state->q);

    return true;
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
