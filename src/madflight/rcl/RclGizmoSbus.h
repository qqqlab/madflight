#pragma once

#include "rcl.h"
#include "../hal/MF_Serial.h"
#include "sbus/SBUS.h" //sBus interface

class RclGizmoSbus : public RclGizmo {
  private:
    SBUS sbus;
    uint16_t* pwm;

  public:
    RclGizmoSbus(MF_Serial *ser_bus, uint16_t *pwm) {
      this->pwm = pwm;
      sbus._bus = ser_bus;
      sbus.begin();
    }

    bool update() {
      uint16_t sbusChannels[16];
      bool sbusFailSafe;
      bool sbusLostFrame;
      if(!sbus.read(sbusChannels, &sbusFailSafe, &sbusLostFrame)) return false;
  
      //sBus scaling below is for Taranis-Plus and X4R-SB
      float scale = 0.615;
      float bias  = 895.0;
      for(int i=0;i<16;i++) {
        pwm[i] = sbusChannels[i] * scale + bias;
      }
      return true;
    }
};
