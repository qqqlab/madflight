#pragma once

#include "../hal/MF_Serial.h"
#include "crsf/crsf.h"

class RclGizmoCrsf : public RclGizmo {
  private:
    CRSF *crsf = nullptr;
    MF_Serial *ser_bus = nullptr;

  public:
    RclGizmoCrsf(MF_Serial *ser_bus, uint32_t baud, uint16_t* pwm) {
      if(baud==0) baud = CRSF_BAUD;
      ser_bus->begin(baud);
      this->ser_bus = ser_bus;
      crsf = new CRSF(pwm, ser_bus);
    }

    ~RclGizmoCrsf() override {
      delete crsf;
    }

    bool update() override {
      bool rv = false;
      while(ser_bus->available()) {
        int c = ser_bus->read();
        //print received data
        //if(c == CRSF_ADDRESS_FLIGHT_CONTROLLER) Serial.printf("\nreceived: "); Serial.printf("%02x ",c);
        if(crsf->update(c)) {
          //print decoded rc data
          //Serial.print(" decoded RC: "); for(int i=0;i<16;i++) Serial.printf("%d:%d ",i,rcl_crsf.channel[i]); Serial.println();
          rv = true;
        }
      }
      return rv;
    }
};