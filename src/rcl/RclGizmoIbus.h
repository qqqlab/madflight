#pragma once

#include "../hal/hal.h"
#include "rcl.h"
#include "ibus/IBus.h"

class RclGizmoIbus : public RclGizmo {
  private:
    RclGizmoIbus() {}
    IBus ibus;
    MF_Serial *ser_bus;
    uint16_t *pwm;

  public:
    static RclGizmoIbus* create(int ser_bus_id, uint16_t *pwm, int baud) {
      if(baud==0) baud = 115200;
      MF_Serial* ser_bus = hal_get_ser_bus(ser_bus_id, baud);
      if(!ser_bus) return nullptr;

      auto gizmo = new RclGizmoIbus();
      gizmo->ser_bus = ser_bus;
      gizmo->pwm = pwm;
      gizmo->ibus.serial = ser_bus;
      gizmo->ibus.begin();
      return gizmo;
    }

    bool update() override {
      bool rv = false;
      while(ser_bus->available()) {
        // we have received data on the UART
        if (ibus.update()) {
          // update pwm with decoded rc data
          for (uint8_t i=0; i<IBus::PROTOCOL_CHANNELS;i++) {
            //Serial.print(" decoded RC: "); Serial.printf("%d:%d ",i,ibus.readChannel(i); Serial.println();
            pwm[i] = ibus.readChannel(i);
          }
          rv = true;
        }
      }
      return rv;
    }
};