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
        ibus.done_reading_data();
        count++;
      }

      // !!HACK!!
      // FlySky FS-i6 firmware (https://github.com/qba667/FlySkyI6) failsafe
      // doesn't seem to actually change channels 1-6 even when set to
      // failsafe. But channel 9-12 become > 1500 when they are 1500 if unused and
      // we lost radio link. Just hardcode that and hope no one will ever be in a
      // situation where all these are > 1500 except on losing radio link in failsafe!
      // !!HACK!!
      if (pwm[9] > 1500 && pwm[10] > 1500 && pwm[11] >1500 && pwm[12] > 1500) {
        return false;
      }
      return true;
    }
};