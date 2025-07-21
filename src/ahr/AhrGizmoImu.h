#pragma once

#include "ahr.h"

class AhrGizmoImu : public AhrGizmo {
private:
  AhrConfig *config;
  AhrState *state;

  //NED reference frame
  //gyro in rad/sec
  //acc in g
  //mag any unit of measurement

  // quaternion of sensor frame relative to auxiliary frame
  float q0 = 1.0f; //Initialize quaternion
  float q1 = 0.0f;
  float q2 = 0.0f;
  float q3 = 0.0f;

  public:
  AhrGizmoImu(Ahr *ahr) {
    this->config = &(ahr->config);
    this->state = (AhrState*)ahr;
  }

  void setInitalOrientation(float *qnew) {
    q0 = qnew[0];
    q1 = qnew[1];
    q2 = qnew[2];
    q3 = qnew[3];
  }

  bool update() {
    // just copy the quaternions from the IMU
    state->q[0] = config->pimu->q[0];
    state->q[1] = config->pimu->q[1];
    state->q[2] = config->pimu->q[2];
    state->q[3] = config->pimu->q[3];
    return true;
  }
};
