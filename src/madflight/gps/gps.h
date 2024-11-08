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
