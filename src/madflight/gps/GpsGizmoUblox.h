#pragma once

#include "gps.h"
#include "../hal/hal.h"
#include "GPS-uBlox/qqqlab_GPS_UBLOX.h"
//#include "GPS-uBlox/qqqlab_AutoBaud.h"

class GpsGizmoUblox: public GpsGizmo {
  private:
      GpsGizmoUblox() {} //private constructor

  public:
    static GpsGizmoUblox* create(GpsState *state, int ser_bus_id, int baud) {
      //get serial bus
      if(baud == 0) baud = 230400; //baud is initial GPS baud rate to try
      MF_Serial* ser_bus = hal_get_ser_bus(ser_bus_id, baud);
      if(!ser_bus) return nullptr;

      //setup gizmo
      auto gizmo = new GpsGizmoUblox();
      gizmo->gps_ublox.state = state;
      gizmo->gps_ublox.ser_bus = ser_bus;
      gizmo->gps_ublox.rate_ms = 100;   //optional - gps update rate in milliseconds (default 100)
      gizmo->gps_ublox.save_config = 2, //optional - save config  0:Do not save config, 1:Save config, 2:Save only when needed (default 2)
      gizmo->gps_ublox.gnss_mode = 0;   //optonial - GNSS system(s) to use  Bitmask: 1:GPS, 2:SBAS, 4:Galileo, 8:Beidou, 16:IMES, 32:QZSS, 64:GLONASS (default 0=leave as configured)
      return gizmo;
    }

    bool update() override {
      //update GPS (call at least 10 times per second)
      return gps_ublox.update();
    }
    
  protected:
    class GPS_UBLOX : public AP_GPS_UBLOX {
      public:
        MF_Serial *ser_bus;

        //interface
        void I_setBaud(int baud)                      override {ser_bus->begin(baud);}
        inline int I_availableForWrite()              override {return ser_bus->availableForWrite();}
        inline int I_available()                      override {return ser_bus->available();}
        inline int I_read(uint8_t* data, size_t len)  override {return ser_bus->read(data, len);}
        inline int I_write(uint8_t* data, size_t len) override {return ser_bus->write(data, len);}
        inline uint32_t I_millis()                    override {return ::millis();}
        void I_print(const char *str)                 override {Serial.print("GPS: "); Serial.print(str);}
    };

    GPS_UBLOX gps_ublox = {}; //init to 0 is NEEDED!!!
  };
