#pragma once

#include "ahr.h"
#include "Madgwick/Madgwick.h"

class AhrGizmoMadgwick : public AhrGizmo {
private:
  AhrConfig *config;
  AhrState *state;
  Madgwick ahrs;

public:
  AhrGizmoMadgwick(Ahr *ahr) {
    this->config = &(ahr->config);
    this->state = (AhrState*)ahr;
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
