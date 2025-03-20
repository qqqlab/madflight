#pragma once

#include "rcl.h"
#include "../hal/MF_Serial.h"
#include "dsmrx/DSMRX.h"

class RclGizmoDsm : public RclGizmo {
  private:
    DSM1024 DSM;
    MF_Serial *ser_bus;
    uint16_t *pwm;

  public:
    RclGizmoDsm(MF_Serial *ser_bus, int baud, uint16_t *pwm) {
      this->pwm = pwm;
      this->ser_bus = ser_bus;
      if(baud<=0) baud = 115200;
      ser_bus->begin(baud);
    }

    bool update() {
      uint8_t b;
      while(ser_bus->read(&b, 1)) {
        DSM.handleSerialEvent(b, micros());
      }
      if(!DSM.gotNewFrame()) return false;
      DSM.getChannelValues(pwm);
      return true;
    }
};
