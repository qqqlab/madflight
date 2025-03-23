#pragma once

#include "rcl.h"
#include "../hal/MF_Serial.h"
#include "sbus/SBUS.h" //sBus interface

class RclGizmoSbus : public RclGizmo {
  private:
    SBUS sbus;
    uint16_t* pwm;

    RclGizmoSbus() {}

  public:
    static RclGizmoSbus* create(int ser_bus_id, uint16_t *pwm, int baud, bool invert) {
      //get serial bus
      if(baud == 0) baud = 100000;
      MF_Serial* ser_bus = hal_get_ser_bus(ser_bus_id, baud, MF_SerialMode::mf_SERIAL_8E2, invert);
      if(!ser_bus) return nullptr;

      //setup sbus gizmo
      auto gizmo = new RclGizmoSbus();
      gizmo->pwm = pwm;
      gizmo->sbus._bus = ser_bus;
      gizmo->sbus.begin();
      return gizmo;
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
      return !sbusFailSafe;
    }
};
