#pragma once

#include "gps.h"
#include "../hal/MF_Serial.h"
#include "GPS-uBlox/qqqlab_GPS_UBLOX.h"
//#include "GPS-uBlox/qqqlab_AutoBaud.h"

class GpsGizmoUblox: public GpsGizmo {
public:
  GpsGizmoUblox(GpsState *state, MF_Serial *ser_bus, int baud) {
    //baud is initial GPS baud rate to try
    if(baud<=0) baud = 230400;

    //optional auto-baud to speed up gps connection
    //baud = autobaud(PIN_GPS_RX);
    //Serial.printf("Initial GPS baud rate:%d\n", baud);

    //start GPS Serial with initial baud
    ser_bus->begin(baud);

    //start GPS
    gps_ublox.state = state;
    gps_ublox.ser_bus = ser_bus;
    gps_ublox.rate_ms = 100;   //optional - gps update rate in milliseconds (default 100)
    gps_ublox.save_config = 2, //optional - save config  0:Do not save config, 1:Save config, 2:Save only when needed (default 2)
    gps_ublox.gnss_mode = 0;   //optonial - GNSS system(s) to use  Bitmask: 1:GPS, 2:SBAS, 4:Galileo, 8:Beidou, 16:IMES, 32:QZSS, 64:GLONASS (default 0=leave as configured)
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
      void I_print(const char *str)                 override {Serial.print("GPS:  "); Serial.print(str);}
  } gps_ublox;
};
