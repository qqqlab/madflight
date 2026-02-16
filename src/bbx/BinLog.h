/*==========================================================================================
MIT License

Copyright (c) 2023-2026 https://madflight.com

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

#pragma once

#include <Arduino.h> //Serial
#include "BinLogWriter.h"

// prototypes for BinLog
namespace BinLogWriter {
  extern State_t state;
  extern uint32_t startMicros;
  bool queueSend(uint8_t *buf, uint8_t len, uint8_t keepfree = 0);
  bool queueSendFMTU(bool sendFMT, uint8_t type, const char *units, const char* multipliers);
  uint8_t typeRegistry_find(const char *name, bool *fmt_write);
  void typeRegistry_FMT_was_sent(char typ);
};

//BinLog uses 32bit microsecond timestamps (good for 1 hour of recording before wrap-around)

//=====================================================================================================================
// BinLog
//=====================================================================================================================

struct UnitStructure {
    const char ID;
    const char *unit;
};

struct MultiplierStructure {
    const char ID;
    const float mult;
};

// all units here should be base units
// This does mean battery capacity is here as "amp*second"
// Please keep the names consistent with Tools/autotest/param_metadata/param.py:33
const struct UnitStructure log_Units[] = {
  
    { '-', "" },              // no units e.g. Pi, or a string
    { '?', "UNKNOWN" },       // Units which haven't been worked out yet....
    { 'A', "A" },             // Ampere
    { 'a', "Ah" },            // Ampere hours
    { 'd', "deg" },           // of the angular variety, -180 to 180
    { 'b', "B" },             // bytes
    { 'B', "B/s" },           // bytes per second
    { 'k', "deg/s" },         // degrees per second. Degrees are NOT SI, but is some situations more user-friendly than radians
    { 'D', "deglatitude" },   // degrees of latitude
    { 'e', "deg/s/s" },       // degrees per second per second. Degrees are NOT SI, but is some situations more user-friendly than radians
    { 'E', "rad/s" },         // radians per second
    { 'G', "Gauss" },         // Gauss is not an SI unit, but 1 tesla = 10000 gauss so a simple replacement is not possible here
    { 'g', "G" },             // MADFLIGHT: G-force
    { 'h', "degheading" },    // 0.? to 359.?
    { 'i', "As"  },           // Ampere second
    { 'J', "J"   },           // Joule (Watt second)
    { 'l', "l" },             // litres
    { 'L', "rad/s/s" },       // radians per second per second
    { 'm', "m" },             // metres
    { 'n', "m/s" },           // metres per second
    // { 'N', "N" },          // Newton
    { 'o', "m/s/s" },         // metres per second per second
    { 'O', "degC" },          // degrees Celsius. Not SI, but Kelvin is too cumbersome for most users
    { '%', "%" },             // percent
    { 'S', "satellites" },    // number of satellites
    { 's', "s" },             // seconds
    { 'T', "uT" },            // MADFLIGHT: micro Tesla
    { 'q', "rpm" },           // rounds per minute. Not SI, but sometimes more intuitive than Hertz
    { 'r', "rad" },           // radians
    { 'U', "deglongitude" },  // degrees of longitude
    { 'u', "ppm" },           // pulses per minute
    { 'v', "V" },             // Volt
    { 'P', "Pa" },            // Pascal
    { 'w', "Ohm" },           // Ohm
    { 'W', "Watt" },          // Watt
    { 'X', "Wh"  },           // Watt hour
    { 'y', "l/s" },           // litres per second
    { 'Y', "us" },            // pulse width modulation in microseconds
    { 'z', "Hz" },            // Hertz
    { '#', "instance" }       // (e.g.)Sensor instance number
};
const uint8_t _num_units = (sizeof(log_Units) / sizeof(log_Units[0]));

// this multiplier information applies to the raw value present in the
// log.  Any adjustment implied by the format field (e.g. the "centi"
// in "centidegrees" is *IGNORED* for the purposes of scaling.
// Essentially "format" simply tells you the C-type, and format-type h
// (int16_t) is equivalent to format-type c (int16_t*100)
// tl;dr a GCS shouldn't/mustn't infer any scaling from the unit name

const struct MultiplierStructure log_Multipliers[] = {
    { '-', 0 },       // no multiplier e.g. a string
//      { '?', 1 },       // multipliers which haven't been worked out yet....
// <leave a gap here, just in case....>
    { '2', 1e2 },
    { '1', 1e1 },
    { '0', 1e0 },
    { 'A', 1e-1 },
    { 'B', 1e-2 },
    { 'C', 1e-3 },
    { 'D', 1e-4 },
    { 'E', 1e-5 },
    { 'F', 1e-6 },
    { 'G', 1e-7 },
    { 'I', 1e-9 },
// <leave a gap here, just in case....>
    { '!', 3.6 }, // (ampere*second => milliampere*hour) and (km/h => m/s)
    { '/', 3600 }, // (ampere*second => ampere*hour)
};
const uint8_t _num_multipliers = (sizeof(log_Multipliers) / sizeof(log_Multipliers[0]));


class BinLog {
public:
  BinLog(const char* name) {
    msg_begin(name);
  }

  ~BinLog() {
    msg_end();
  }

  uint8_t keepFree = 5; //number of queue spots to keep free when sending messages 

private:
  uint8_t buf[128];
  uint8_t buflen = 0;
  uint8_t msgType = 0;
public:
  bool inhibit = false;
  bool isHeader = false;
private:
  bool error = false;
  
  // FMT message
  struct PACKED {
      uint8_t h1 = HEAD_BYTE1;
      uint8_t h2 = HEAD_BYTE2;
      uint8_t type = 0x80;
      uint8_t msg_type;
      uint8_t length;
      char name[4];
      char format[16];
      char labels[64];
  } FMT;
  uint8_t FMT_fmt; //format position 
  uint8_t FMT_lbl; //label position
  bool FMT_write = false;
  char FMT_unit[16];
  char FMT_mult[16];
  uint8_t FMT_msglen;

  void FMT_addField(const char fmt, const uint8_t datalen, const char* label, float mult, const char* unit) {
    //check datalen
    FMT_msglen += datalen;
    if (FMT_msglen >= MAX_MSG_LEN) {
      error = true;
      Serial.printf("BinLog: ERROR data too long for msg %s\n", FMT.name);
      return;
    }
    
    //datatype
    if (FMT_fmt >= 16) {
      error = true;
      Serial.printf("BinLog: ERROR too many fields for msg %s\n", FMT.name);
      return;
    }
    FMT.format[FMT_fmt] = fmt;
 
    //multiplier
    for(int i=0;i<_num_multipliers;i++) {
      if(mult == log_Multipliers[i].mult) {
        FMT_mult[FMT_fmt] = log_Multipliers[i].ID;
        break;
      }
    }
    FMT_mult[FMT_fmt] = mult;
    
    //units
    if(unit) {
      for(int i=0;i<_num_units;i++) {
        if(strcmp(unit, log_Units[i].unit) == 0) {
          FMT_unit[FMT_fmt] = log_Units[i].ID;
          break;
        }
      }
    }

    FMT_fmt++;

    //column labels
    int lbl_len = strlen(label);
    if (FMT_lbl + 1 + lbl_len > 64) {
      FMT.format[0]=0;
      Serial.printf("BB:   ERROR labels too long for msg %s\n", FMT.name);
      return;
    }
    if (FMT_lbl) FMT.labels[FMT_lbl++] = ',';
    strcpy(FMT.labels+FMT_lbl, label);
    FMT_lbl += lbl_len;
  }

private:
  void msg_begin(const char* name) {
    msgType = BinLogWriter::typeRegistry_find(name, &FMT_write); //this sets FMT_write when new message name is encountered
    
    //prepare message header
    buflen = 0;
    buf[buflen++] = HEAD_BYTE1;
    buf[buflen++] = HEAD_BYTE2;
    buf[buflen++] = msgType;

    //prepare FMT header
    if(FMT_write) {
      FMT = {};
      FMT.h1 = HEAD_BYTE1;
      FMT.h2 = HEAD_BYTE2;
      FMT.type = 0x80;
      FMT.msg_type = msgType;
      strncpy(FMT.name, name, 4);
      FMT_fmt = 0;
      FMT_lbl = 0;
      FMT_msglen = 3;
      
      memset(FMT_unit, '-', 16);
      memset(FMT_mult, '-', 16);
    }
    
  }

  void msg_end() {
    //only queue message if BinLogWriter started or when writing file header
    if(error || inhibit || (BinLogWriter::state != BinLogWriter::STARTED && !isHeader) ) return;

    if(FMT_write) {
      //queue FMT
      FMT.length = buflen; //store message length in FMT message
      if(!BinLogWriter::queueSend((uint8_t*)&FMT, sizeof(FMT))) {
        return;
      }
      //queue FMTU
      if(!BinLogWriter::queueSendFMTU(false, msgType, FMT_unit, FMT_mult)) {
        return;
      }
      BinLogWriter::typeRegistry_FMT_was_sent(msgType);
    }

    //queue message
    BinLogWriter::queueSend(buf,buflen, keepFree); //keep a few queue spots free for more important messages (such as FMT)

    inhibit = true;
  }

public:
  bool msg_queueFMT() {
    if(!FMT_write) return true; //FMT message was already sent, nothing to do
    FMT.length = buflen;
    return BinLogWriter::queueSend((uint8_t*)&FMT, sizeof(FMT));
  }

  bool msg_queue() {
    return BinLogWriter::queueSend(buf,buflen);
  }

  //-------------------------------
  // Message fields
  //-------------------------------
public:
  void TimeUS() {
    TimeUS(micros());
  }
  void TimeUS(uint32_t ts) {
    u32("TimeUS", ts - BinLogWriter::startMicros, 1e-6, "s");
  }  
  void u8(const char* label, uint8_t v, float mult = 1, const char* unit = nullptr) {
    if(FMT_write) FMT_addField('B', 1, label, mult, unit); // B   : uint8_t
    buf[buflen++] = v;
  }
  void u8flightmode(const char* label, uint8_t v, float mult = 1, const char* unit = nullptr) {
    if(FMT_write) FMT_addField('M', 1, label, mult, unit); // M   : uint8_t flight mode
    buf[buflen++] = v;
  }  
  void u16(const char* label, uint16_t v, float mult = 1, const char* unit = nullptr) {
    if(FMT_write) FMT_addField('H', 2, label, mult, unit); // H   : uint16_t
    memcpy(buf+buflen, &v, 2);
    buflen+=2;
  }
  void u16x100(const char* label, uint16_t v, float mult = 1, const char* unit = nullptr) { 
    if(FMT_write) FMT_addField('C', 2, label, mult, unit); // C   : uint16_t * 100
    memcpy(buf+buflen, &v, 2);
    buflen+=2;
  }  
  void u32(const char* label, uint32_t v, float mult = 1, const char* unit = nullptr) {
    if(FMT_write) FMT_addField('I', 4, label, mult, unit); // I   : uint32_t
    memcpy(buf+buflen, &v, 4);
    buflen+=4;
  }
  void u32x100(const char* label, uint32_t v, float mult = 1, const char* unit = nullptr) {
    if(FMT_write) FMT_addField('E', 4, label, mult, unit); // E   : uint32_t * 100
    memcpy(buf+buflen, &v, 4);
    buflen+=4;
  }
  void u64(const char* label, uint64_t v, float mult = 1, const char* unit = nullptr) {
    if(FMT_write) FMT_addField('Q', 8, label, mult, unit); // Q   : uint64_t
    memcpy(buf+buflen, &v, 8);
    buflen+=8;
  }  
  void i8(const char* label, int8_t v, float mult = 1, const char* unit = nullptr) {
    if(FMT_write) FMT_addField('b', 1, label, mult, unit); // b   : int8_t
    buf[buflen++] = v;
  }
  void i16(const char* label, int16_t v, float mult = 1, const char* unit = nullptr) {
    if(FMT_write) FMT_addField('h', 2, label, mult, unit); // h   : int16_t
    memcpy(buf+buflen, &v, 2);
    buflen+=2;
  }
  void i16x100(const char* label, int16_t v, float mult = 1, const char* unit = nullptr) {
    if(FMT_write) FMT_addField('c', 2, label, mult, unit); // c   : int16_t * 100
    memcpy(buf+buflen, &v, 2);
    buflen+=2;
  }
  void i32(const char* label, int32_t v, float mult = 1, const char* unit = nullptr) {
    if(FMT_write) FMT_addField('i', 4, label, mult, unit); // i   : int32_t
    memcpy(buf+buflen, &v, 4);
    buflen+=4;
  }
  void i32x100(const char* label, int32_t v, float mult = 1, const char* unit = nullptr) {
    if(FMT_write) FMT_addField('e', 4, label, mult, unit); // e   : int32_t * 100
    memcpy(buf+buflen, &v, 4);
    buflen+=4;
  }
  void i32latlon(const char* label, int32_t v, float mult = 1, const char* unit = nullptr) {
    if(FMT_write) FMT_addField('L', 4, label, mult, unit); // L   : int32_t latitude/longitude
    memcpy(buf+buflen, &v, 4);
    buflen+=4;
  }
  void i64(const char* label, int64_t v, float mult = 1, const char* unit = nullptr) {
    if(FMT_write) FMT_addField('q', 8, label, mult, unit); // q   : int64_t
    memcpy(buf+buflen, &v, 8);
    buflen+=8;
  }   
  void flt(const char* label, float v, float mult = 1, const char* unit = nullptr) {
    if(FMT_write) FMT_addField('f', 4, label, mult, unit); // f   : float
    memcpy(buf+buflen, &v, 4);
    buflen+=4;
  }
  void dbl(const char* label, double v, float mult = 1, const char* unit = nullptr) {
    if(FMT_write) FMT_addField('d', 8, label, mult, unit); // d   : double
    memcpy(buf+buflen, &v, 8);
    buflen+=8;
  }
  void char4(const char* label, const char* v, float mult = 1, const char* unit = nullptr) {
    if(FMT_write) FMT_addField('n', 4, label, mult, unit); // n   : char[4]
    strncpy((char*)(buf+buflen), v, 4);
    buflen+=4;
  }
  void char16(const char* label, const char* v, float mult = 1, const char* unit = nullptr) {
    if(FMT_write) FMT_addField('N', 16, label, mult, unit);  // N   : char[16]
    strncpy((char*)(buf+buflen), v, 16);
    buflen+=16;
  }
  void char64(const char* label, const char* v, float mult = 1, const char* unit = nullptr) {
    if(FMT_write) FMT_addField('Z', 64, label, mult, unit);  // Z   : char[64]
    strncpy((char*)(buf+buflen), v, 64);
    buflen+=64;
  }
  void blob64(const char* label, const int16_t* v, float mult = 1, const char* unit = nullptr) {
    if(FMT_write) FMT_addField('a', 64, label, mult, unit);  // a   : int16_t[32]
    memcpy(buf+buflen, v, 64); 
    buflen+=64;
  }
};
