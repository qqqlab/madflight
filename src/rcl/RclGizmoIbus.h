#pragma once

#include "../hal/MF_Serial.h"
#include "../hal/hal.h"
#include "rcl.h"
#include "ibus/IBus.h"

class RclGizmoIbus : public RclGizmo {
  private:
    RclGizmoIbus(MF_Serial *ser_bus) : ser_bus(ser_bus) {}
    IBus ibus;
    MF_Serial *ser_bus;
    uint16_t *pwm;

  public:
    static RclGizmoIbus* create(int ser_bus_id, uint16_t *pwm, int baud) {
      if(baud==0) baud = 115200;
      MF_Serial* ser_bus = hal_get_ser_bus(ser_bus_id, baud);
      if(!ser_bus) return nullptr;

      auto gizmo = new RclGizmoIbus(ser_bus);
      gizmo->ser_bus = ser_bus;
      gizmo->pwm = pwm;
      gizmo->ibus.begin(*ser_bus, IBus::NO_TIMER);
      return gizmo;
    }

    int count = 0;
    bool update() override {
      ibus.process_events();
      if (ibus.has_new_data()) {
        for (uint8_t i=0; i<IBus::MAX_CHANNELS; i++) {
          pwm[i] = ibus.get_channel_value(i);
        }
        count++;
        return true;
      }
      return false;
    }
};