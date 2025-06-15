/*==========================================================================================
GPS.h 

Library for parsing GPS NMEA RMC, GGA messages, and u-blox PUBX00 messages

Based on: MicroNMEA by Steve Marple https://github.com/stevemarple/MicroNMEA

MIT License

Copyright (c) 2023 https://github.com/qqqlab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
===========================================================================================*/

/*------------------------------------------------------------------------------------------
//usage example 

Serial2.begin(GPS_BAUD);
char buffer[255]; //PUBX messages can be longer than gps standard 85 char
GPS gps(buffer, sizeof(buffer));
while(1) {
while(Serial2.available()) {
    char c = Serial2.read();
    Serial.print(c);
    if (gps.process(c)) { 
        Serial.printf("\n---> time:%d fix:%d lat:%d lon:%d alt:%d galt:%d sog:%d cog:%d sats:%d hacc:%d vacc:%d", gps.time, gps.fix, gps.lat, gps.lon, gps.alt, gps.sep, gps.sog, gps.cog, gps.sat, gps.hacc, gps.vacc);
    }
}
------------------------------------------------------------------------------------------*/
#pragma once

#include <limits.h>  //for LONG_MIN
#include <Arduino.h> //for millis()

class GPS {
  public:
    int32_t time; //time in milliseconds since midnight UTC
    int32_t date; //date as DDMMYY
    int32_t lat;  //Latitude in degrees * 10e7
    int32_t lon;  //Longitude in degrees * 10e7
    int32_t alt;  //Altitude in millimeters above user datum ellipsoid (= MSL Altitude + Geoid Separation from GGA message)
    int32_t fix;  //0:no fix, 1:fix 2:2D fix, 3:3D fix
    int32_t hacc; //Horizontal accuracy estimate in millimeters
    int32_t vacc; //Vertical accuracy estimate in millimeters
    int32_t sog;  //Speed over ground in mm/s
    int32_t cog;  //Course over ground in degrees * 1000
    int32_t veld; //Vertical downward speed in mm/s
    int32_t sat;  //number of satellites
    int32_t hdop; //HDOP Horizontal Dilution of Precision
    int32_t vdop; //VDOP Vertical Dilution of Precision
    int32_t tdop; //TDOP Time Dilution of Precision
    int32_t sep; //Geoid separation: difference between ellipsoid and mean sea level in millimetres.
    uint32_t update_millis; //millis timestamp last pos update

    GPS(void* buf, uint16_t len)
    {
      setBuffer(buf, len);
      clear();
    }

    //Send a NMEA sentence to the GNSS receiver. The sentence must start with `$`; the checksum and `\r\n` terminators will be appended automatically.
    static Stream& sendSentence(Stream& s, const char* sentence)
    {
      char checksum[3];
      generateChecksum(sentence, checksum);
      checksum[2] = '\0';
      s.print(sentence);
      s.print('*');
      s.print(checksum);
      s.print("\r\n");
      return s;
    }

    void setBuffer(void* buf, uint16_t len)
    {
      _bufferLen = len;
      _buffer = (char*)buf;
      _ptr = _buffer;
      if (_bufferLen) {
        *_ptr = '\0';
        _buffer[_bufferLen - 1] = '\0';
      }
    }

    // process a character, returns true if position was updated
    bool process(char c)
    {
      if (_buffer == nullptr || _bufferLen == 0) return false;
      
      if (c == 0 || c == '\n' || c == '\r') {
        // Terminate buffer then reset pointer
        *_ptr = 0;
        _ptr = _buffer;
        if (_buffer[0] == '$' && testChecksum(_buffer)) {
            // Valid message
            if (_buffer[1] == 'G' && strncmp(&_buffer[3], "GGA,", 4) == 0) {
                return processGGA(_buffer);
            } else if (_buffer[1] == 'G' && strncmp(&_buffer[3], "RMC,", 4) == 0) {
                return processRMC(_buffer);
            } else if(strncmp(_buffer, "$PUBX,00,", 9) == 0) {
                return processPUBX00(_buffer);
            }
        }
      }
      else {
        *_ptr = c;
        if (_ptr < &_buffer[_bufferLen - 1]) ++_ptr;
      }
      return false;
    }

    // Get received NMEA sentence
    const char* getSentence(void) const {
      return _buffer;
    }

    void clear(void)
    {
        _use_only_pubx00 = false;
        time = 0; //time in milliseconds since midnight UTC
        date = 0; //date as DDMMYY
        lat = LONG_MIN;  //Latitude in degrees * 10e7
        lon = LONG_MIN;  //Longitude in degrees * 10e7
        alt = LONG_MIN;  //Altitude in millimeters above user datum ellipsoid (= MSL Altitude + Geoid Separation from GGA message)
        fix = 0;  //0:no fix, 1:fix 2:2D fix, 3:3D fix
        hacc = LONG_MIN; //Horizontal accuracy estimate in millimeters
        vacc = LONG_MIN; //Vertical accuracy estimate in millimeters
        sog = LONG_MIN;  //Speed over ground in mm/s
        cog = LONG_MIN;  //Course over ground in degrees * 1000
        veld = LONG_MIN; //Vertical downward speed in mm/s
        sat = 0;  //number of satellites
        hdop = LONG_MIN; //HDOP Horizontal Dilution of Precision
        vdop = LONG_MIN; //VDOP Vertical Dilution of Precision
        tdop = LONG_MIN; //TDOP Time Dilution of Precision
        sep = LONG_MIN; //Height above WGS84 Geoid in millimetres.
        update_millis = 0; //millis timestamp last pos update
    }

  protected:
    static inline bool isEndOfFields(char c) {
      return c == '*' || c == '\0' || c == '\r' || c == '\n';
    }

    bool parseLatLngRef(const char* &s, int32_t &v) {
        int32_t degsec = 0, sec = 0;
        parseIntRef(s, degsec);
        if (s[0] != '.') return false;
        if(!parseFloatRef(s, 7, sec)) return false;
        int32_t deg = degsec / 100;
        sec += (degsec - deg * 100) * 10000000;
        sec = (sec + 30) / 60;
        deg = deg * 10000000 + sec;
        if (s[0] == 'S' || s[0] == 'W') {
            deg = -deg;
        }else if (s[0] != 'N' && s[0] != 'E') {
            return false;
        }
        s++; //skip S/N E/W indicator
        if (s[0] != ',') return false;
        s++; //skip ,
        v = deg;
        return true;
    }

    bool parseTimeRef(const char* &s, int32_t &v) {
        int32_t t;
        if (!parseFloatRef(s, 3, t)) return false;
        int32_t h = t / 10000000;
        t -= h * 10000000;
        int32_t m = t / 100000;
        t -= m * 100000;
        t += m * 60000 + h * 3600000;
        v =t;
        return true;
    }

    //move s to next field, set s=nullpointer if no more fields
    bool skipFieldRef(const char* &s)
    {
      if (s == nullptr) return false;
      while (s[0] != ',' && !isEndOfFields(s[0])) s++;
      if (s[0] == ',') {
          s++; //skip delimiter
      }else{
          s = nullptr; //no more fields
      }
      return true;
    }

    //chop an integer of string, returns true if field found or empty
    //sets s=nullptr if whole string was parsed
    bool parseIntRef(const char * &s, int32_t &v) {
      if (s == nullptr) return false;
      bool neg = (s[0]=='-');
      if (neg) s++; //skip over '-'
      v = 0;
      while(s[0]>='0' && s[0]<='9') {
        v = v * 10 + (s[0] - '0');
        s++; //next digit
      }
      if (neg) v = -v;
      
      //check end of string
      if (isEndOfFields(s[0])) {
          s = nullptr; 
          return true;
      }
      
      //check ending delimiter
      if (s[0] == ',') {
        s++; //skip delimiter
        return true;
      } else {
        return false; //not at end of string or at delimiter - something is wrong
      }
    }

    //chop a float as scaled integer of string, returns true if field found or empty
    //sets s=nullptr if whole string was parsed
    //sets v=LONG_MIN if field was empty
    bool parseFloatRef(const char * &s, int scaledigits, int32_t &v) {
      bool empty = true;
      
      //exit if no more fields
      if (s == nullptr) return false;

      //test negative sign
      bool neg = (s[0]=='-');
      if (neg) s++; //skip over '-'

      //before decimal point
      v = 0;
      while(s[0]>='0' && s[0]<='9') {
        empty = false;
        v = v * 10 + (s[0] - '0');
        s++; //next digit
      }

      //after decimal point
      if (s[0] == '.') {
        s++; //skip decimal point
        while(s[0]>='0' && s[0]<='9') {
          empty = false;
          if(scaledigits>0) {
            v = v * 10 + (s[0] - '0');
            scaledigits--;
          }
          s++; //next digit
        }
      }

      //scale any remaining scaledigits
      while(scaledigits>0) {
        v = v * 10;
        scaledigits--;
      }

      //apply negative sign
      if (neg) v = -v;

      //empty check
      if (empty) v = LONG_MIN;

      //check end of string
      if (isEndOfFields(s[0])) {
          s = nullptr; 
          return true;
      }
      
      //check ending delimiter
      if (s[0] == ',') {
        s++; //skip delimiter
        return true;
      } else {
        return false; //not at end of string or at delimiter - something is wrong
      }
    }

    bool processRMC(const char* s)
    {
        //skip message ID
        if (!skipFieldRef(s)) return false;

        //UTC Time
        int32_t tmp_time;
        if (!parseTimeRef(s, tmp_time)) return false;

        //Status
        bool tmp_fix = (*s == 'A' ? 1 : 0);
        s += 2; // Skip validity and comma

        //Latitude + N/S indicator
        int32_t tmp_lat;
        if (!parseLatLngRef(s, tmp_lat)) return false;

        //Longitude + E/W indicator
        int32_t tmp_lon;
        if (!parseLatLngRef(s, tmp_lon)) return false;

        //Speed over ground in knots (1 kt = 514.44 mm/s)
        int32_t tmp_sog;
        if (!parseFloatRef(s, 3, tmp_sog)) return false;
        tmp_sog = (tmp_sog * 514 + 257) / 1000;

        //Course over ground in degrees
        int32_t tmp_cog;
        if (!parseFloatRef(s, 3, tmp_cog)) return false;

        //Date in day, month, year format
        int32_t tmp_date;
        if (!parseIntRef(s, tmp_date)) return false;

        // That's all we care about, save received data
        //only save date/time if using PUBX00 (PUBX00 does not have date field)
        if(_use_only_pubx00) {
            time = tmp_time;
            date = tmp_date;
            return false;
        }else{
            time = tmp_time;
            fix = tmp_fix;
            lat = tmp_lat;
            lon = tmp_lon;
            sog = tmp_sog;
            cog = tmp_cog;
            date = tmp_date;
            update_millis = millis();
            return true;
        }
    }

    bool processGGA(const char *s)
    {

        //skip message ID
        if (!skipFieldRef(s)) return false;

        //UTC Time
        int32_t tmp_time;
        if (!parseTimeRef(s, tmp_time)) return false;

        //Latitude + N/S indicator
        int32_t tmp_lat;
        if (!parseLatLngRef(s, tmp_lat)) return false;

        //Longitude + E/W indicator
        int32_t tmp_lon;
        if (!parseLatLngRef(s, tmp_lon)) return false;

        //Status
        /*
        0 No Fix / Invalid
        1 Standard GPS (2D/3D)
        2 Differential GPS
        6 Estimated (DR) Fix
        */
        int32_t tmp_fix = (*s >= '1' && *s <= '9' ? 1 : 0);
        s += 2; // Skip position fix flag and comma

        //Number of satellites
        int32_t tmp_sat;
        if (!parseFloatRef(s, 0, tmp_sat)) return false;

        //HDOP Horizontal Dilution of Precision
        int32_t tmp_hdop;
        if (!parseFloatRef(s, 3, tmp_hdop)) return false;

        //MSL Altitude in m
        int32_t tmp_altMsl;
        if (!parseFloatRef(s, 3, tmp_altMsl)) return false;

        //skip 'M'
        if (!skipFieldRef(s)) return false;

        //Geoid Separation in m
        int32_t tmp_sep;
        if (!parseFloatRef(s, 3, tmp_sep)) return false;

        //skip 'M'
        if (!skipFieldRef(s)) return false;

        // That's all we care about, save received data
        //only save geoid altitude if using PUBX00 (PUBX00 does not have this field)
        if(_use_only_pubx00) {
            sep = tmp_sep;
        return false;
        }else{
            time = tmp_time;
            lat = tmp_lat;
            lon = tmp_lon;
            fix = tmp_fix;
            sat = tmp_sat;
            hdop = tmp_hdop;
            if(tmp_altMsl != LONG_MIN && tmp_sep != LONG_MIN) {
                alt = tmp_altMsl + tmp_sep; //altitude above geoid
            }else{
                alt = tmp_altMsl;
            }
            sep = tmp_sep;
            update_millis = millis();
            return true;
        }
    }

    bool processPUBX00(const char* s)
    {
        //skip message ID
        if (!skipFieldRef(s)) return false;

        //skip 00
        if (!skipFieldRef(s)) return false;

        //UTC Time
        int32_t tmp_time;
        if (!parseTimeRef(s, tmp_time)) return false;

        //Latitude + N/S indicator
        int32_t tmp_lat;
        if (!parseLatLngRef(s, tmp_lat)) return false;

        //Longitude + E/W indicator
        int32_t tmp_lon;
        if (!parseLatLngRef(s, tmp_lon)) return false;

        //Altitude in meters above user datum ellipsoid (= MSL Altitude + Geoid Separation from GGA message)
        int32_t tmp_alt;
        if (!parseFloatRef(s, 3, tmp_alt)) return false;

        //Navigation Status (2 char)
        /*
        NF No Fix
        DR Dead reckoning only solution
        G2 Stand alone 2D solution
        G3 Stand alone 3D solution
        D2 Differential 2D solution
        D3 Differential 3D solution
        RK Combined GPS + dead reckoning solution
        TT Time only solution  
        */
        int32_t tmp_fix;
        if(s[0] == 'D' && s[1] == 'R') tmp_fix = 1; 
        else if (s[0] == 'G' && s[1] == '2') tmp_fix = 2;
        else if (s[0] == 'G' && s[1] == '3') tmp_fix = 3;
        else if (s[0] == 'D' && s[1] == '2') tmp_fix = 2;
        else if (s[0] == 'D' && s[1] == '3') tmp_fix = 3;
        else if (s[0] == 'R' && s[1] == 'K') tmp_fix = 3;
        else tmp_fix = 0;
        s += 3; // Skip incl comma

        //Horizontal accuracy estimate in meters
        int32_t tmp_hacc;
        if (!parseFloatRef(s, 3, tmp_hacc)) return false;

        //Vertical accuracy estimate in meters
        int32_t tmp_vacc;
        if (!parseFloatRef(s, 3, tmp_vacc)) return false;

        //Speed over ground in km/h (1 km/h = 1/3.6 m/s)
        int32_t tmp_sog;
        if (!parseFloatRef(s, 3, tmp_sog)) return false;
        tmp_sog = (tmp_sog * 10 + 18) / 36;

        //Course over ground in degrees
        int32_t tmp_cog;
        if (!parseFloatRef(s, 3, tmp_cog)) return false;

        //Vertical downward velocity in m/s
        int32_t tmp_veld;
        if (!parseFloatRef(s, 3, tmp_veld)) return false;

        //Age of most recent DGPS corrections in seconds, empty = none available
        if (!skipFieldRef(s)) return false;

        //HDOP Horizontal Dilution of Precision
        int32_t tmp_hdop;
        if (!parseFloatRef(s, 3, tmp_hdop)) return false;

        //VDOP Vertical Dilution of Precision
        int32_t tmp_vdop;
        if (!parseFloatRef(s, 3, tmp_vdop)) return false;

        //TDOP Time Dilution of Precision
        int32_t tmp_tdop;
        if (!parseFloatRef(s, 3, tmp_tdop)) return false;

        //Number of GPS satellites
        int32_t tmp_satGps;
        if (!parseFloatRef(s, 0, tmp_satGps)) return false;

        //Number of GLONASS satellites
        int32_t tmp_satGlo;
        if (!parseFloatRef(s, 0, tmp_satGlo)) return false;

        // That's all we care about, save received data
        time = tmp_time;
        lat = tmp_lat;
        lon = tmp_lon;
        alt = tmp_alt;
        fix = tmp_fix;
        hacc = tmp_hacc;
        vacc = tmp_vacc;
        sog = tmp_sog;
        cog = tmp_cog;
        veld = tmp_veld;
        sat = tmp_satGps + tmp_satGlo;
        hdop = tmp_hdop;
        vdop = tmp_vdop;
        tdop = tmp_tdop;
        update_millis = millis();
        _use_only_pubx00 = true;

        return true;
    }

    bool testChecksum(const char* s)
    {
      char checksum[2];
      const char* p = generateChecksum(s, checksum);
      return *p == '*' && p[1] == checksum[0] && p[2] == checksum[1];
    }

    static const char* generateChecksum(const char* s, char* checksum)
    {
      uint8_t c = 0;
      // Initial $ is omitted from checksum, if present ignore it.
      if (*s == '$')
        ++s;

      while (*s != '\0' && *s != '*')
        c ^= *s++;

      if (checksum) {
        checksum[0] = toHex(c / 16);
        checksum[1] = toHex(c % 16);
      }
      return s;
    }

  private:
    // Sentence buffer and associated pointers
    // static const uint16_t _bufferLen = 83; // 82 + NULL
    // char _buffer[_bufferLen];
    uint16_t _bufferLen;
    char* _buffer;
    char *_ptr;

    bool _use_only_pubx00;

    static char toHex(uint8_t nibble)
    {
      if (nibble >= 10)
        return nibble + 'A' - 10;
      else
        return nibble + '0';
    }
};
