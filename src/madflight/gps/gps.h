#pragma once

#include "GPS-uBlox/qqqlab_GPS_UBLOX.h"
#include "GPS-uBlox/qqqlab_AutoBaud.h"

class GPS_UBLOX : public AP_GPS_UBLOX {
public:
  //interface
  void I_setBaud(int baud)                      override {gps_Serial->begin(baud);}
  inline int I_availableForWrite()              override {return gps_Serial->availableForWrite();}
  inline int I_available()                      override {return gps_Serial->available();}
  inline int I_read(uint8_t* data, size_t len)  override {return gps_Serial->read(data, len);}
  inline int I_write(uint8_t* data, size_t len) override {return gps_Serial->write(data, len);}
  inline uint32_t I_millis()                    override {return ::millis();}
  void I_print(const char *str)                 override {Serial.print("GPS:  "); Serial.print(str);}
} gps_ublox;

AP_GPS_UBLOX::GPS_State &gps = gps_ublox.state;

void gps_setup() {
  Serial.println("GPS:  GPS_USE_UBLOX");

  //initial GPS baud rate to try
  int baud = 230400;

  //optional auto-baud to speed up gps connection
  //baud = autobaud(HW_PIN_GPS_RX);
  //Serial.printf("Initial GPS baud rate:%d\n", baud);

  //start GPS Serial
  gps_Serial->begin(baud);

  //start GPS
  gps_ublox.rate_ms = 100;   //optional - gps update rate in milliseconds (default 100)
  gps_ublox.save_config = 2, //optional - save config  0:Do not save config, 1:Save config, 2:Save only when needed (default 2)
  gps_ublox.gnss_mode = 0;   //optonial - GNSS system(s) to use  Bitmask: 1:GPS, 2:SBAS, 4:Galileo, 8:Beidou, 16:IMES, 32:QZSS, 64:GLONASS (default 0=leave as configured)
}

void gps_debug() {}

bool gps_loop() {
  //update GPS (call at least 10 times per second)
  return gps_ublox.update();
}


//TODO - Enable NMEA

/*
#ifndef GPS_BAUD
  #define GPS_BAUD 115200
#endif

#include "gps_nmea_pubx_parser.h"

char gps_buffer[255]; //PUBX messages can be longer than gps standard 85 char
GPS gps(gps_buffer, sizeof(gps_buffer));

void gps_setup() {
  gps_Serial.begin(GPS_BAUD); //start gps serial
}

void gps_debug() {
  gps_Serial.begin(GPS_BAUD);
  uint32_t gps_ts = 0;
  while(1) {
    if(millis() - gps_ts > 1000) {
      gps_ts = millis();
      Serial.println("Waiting for GPS data...");
    }    
    while(gps_Serial.available()) {
      gps_ts = millis();
      char c = gps_Serial.read();
      Serial.print(c);
      if (gps.process(c)) {
        Serial.printf("\n---> time:%d fix:%d lat:%d lon:%d alt:%d sep:%d sog:%d cog:%d sats:%d hacc:%d vacc:%d veld:%d", (int)gps.time, (int)gps.fix, (int)gps.lat, (int)gps.lon, (int)gps.alt, (int)gps.sep, (int)gps.sog, (int)gps.cog, (int)gps.sat, (int)gps.hacc, (int)gps.vacc, (int)gps.veld);
      }
    }
  }
}

//returns true if position was updated
bool gps_loop() {
  bool updated = false;
  while(gps_Serial.available()) {
    if(gps.process((char)gps_Serial.read())) {
      updated=true;
    }
  }
  return updated;
}
*/