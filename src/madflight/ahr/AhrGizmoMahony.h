#pragma once

#include "ahr.h"
#include "Mahony/Mahony.h"

class AhrGizmoMahony : public AhrGizmo {
private:
  AhrConfig *config;
  AhrState *state;
  Mahony ahrs;

public:
  AhrGizmoMahony(Ahr *ahr, bool update_1g_only) {
    this->config = &(ahr->config);
    this->state = (AhrState*)ahr;

    //Betaflight flavored version, only update if acc is between 0.9 and 1.1 G
    if(update_1g_only) {
      ahrs.config_alen2_min = 0.81;
      ahrs.config_alen2_max = 1.21;
    }
  }

  void setInitalOrientation(float *qnew) {
    ahrs.q0 = qnew[0];
    ahrs.q1 = qnew[1];
    ahrs.q2 = qnew[2];
    ahrs.q3 = qnew[3];
  }

  bool update() {
    ahrs.update(
      state->gx * Ahr::deg_to_rad, state->gy * Ahr::deg_to_rad, state->gz * Ahr::deg_to_rad,
      state->ax, state->ay, state->az,
      state->mx, state->my, state->mz,
      state->dt
    );
    state->q[0] = ahrs.q0;
    state->q[1] = ahrs.q1;
    state->q[2] = ahrs.q2;
    state->q[3] = ahrs.q3;

    return true;
  }
};
