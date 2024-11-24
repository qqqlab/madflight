/*==========================================================================================
BB: madflight black box data logger in ArduPilot Binary Log data format

MIT License

Copyright (c) 2024 https://madflight.com

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

/* moved to bb/bb.h
#define BB_USE_NONE 1
#define BB_USE_SD 2 //SDCARD with 1-bit SPI interface
#define BB_USE_SDMMC 3 //SDCARD with 1-bit MMC interface (ESP32/ESP32-S3)
#define BB_USE_SDDEBUG 4 //print log to Serial
*/

//BinLog uses 32bit microsecond timestamps (good for 1 hour of recording before wrap-around)

//#define FREERTOS_DEFAULT_STACK_SIZE - defined in hw_xxx.h

#define LOG_TYPE_LEN  64    // max number of message types
#define QUEUE_LENGTH  100    // max number of messages in the queue
#define MAX_MSG_LEN   89    // max message len is FMT with 89 (0x59) bytes

#define HEAD_BYTE1  0xA3
#define HEAD_BYTE2  0x95

//black box file system
class BBFS {
public:
  virtual void setup() = 0; //setup the file system (can be called multiple times)
  
  virtual bool writeOpen() = 0; //create new file for writing (closes previously opened file first)
  virtual void write(const uint8_t *buf, const uint8_t len) = 0; //write to file
  virtual void close() = 0; //close file

  virtual void erase() = 0; //erase all files
  virtual void dir() = 0; //list files
  virtual void bench() = 0; //benchmark read/write
  virtual void info() = 0; //card info
};

//=====================================================================================================================
// Logging to Serial
//=====================================================================================================================
#if BB_USE == BB_USE_SDDEBUG

class BBFS_Debug : public BBFS {
public:
  void setup() override {}
  bool writeOpen() override {return true;}
  
  void write(const uint8_t *buf, const uint8_t len) override {
    Serial.print("BinLog:");
    for(int i=0;i<len;i++) Serial.printf("%02X ", buf[i]);
    Serial.println();
  }
  
  void close() override {}
  void erase() override {}
  void dir() override {}
  void bench() override {}
  void info() override {}
};
BBFS_Debug bbfs;


//=====================================================================================================================
// Logging to SDCARD with SPI interface
//=====================================================================================================================
#elif BB_USE == BB_USE_SD

#if defined ARDUINO_ARCH_RP2040
  #include "BBFS_SD_RP2040.h"
#elif defined ARDUINO_ARCH_ESP32
  #include "BBFS_SD_ESP32.h"
#else
  #error BB_USE_SD not available for this processor
#endif
  
BBFS_SD bbfs;

//=====================================================================================================================
// Logging to SDCARD with MMC interface (ESP32/ESP32-S3)
//=====================================================================================================================
#elif BB_USE == BB_USE_SDMMC

#ifdef ARDUINO_ARCH_ESP32
  #include "BBFS_SD_ESP32.h"
#else
  #error BB_USE_SDMMC not available for this processor
#endif

BBFS_SD bbfs;

//=====================================================================================================================
// No Logging
//=====================================================================================================================
#else

class BBFS_None : public BBFS {
public:
  void setup() override {}
  bool writeOpen() override {return false;}
  void write(const uint8_t *buf, const uint8_t len) override {}
  void close() override {}
  void erase() override {}
  void dir() override {}
  void bench() override {}
  void info() override {}
};
BBFS_None bbfs;
#endif



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

//BinLog class: static methods for file, instance methods for message
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
  bool inhibit = false;
  bool isHeader = false;
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
      Serial.printf("BB: ERROR data too long for msg %s\n", FMT.name);
      return;
    }
    
    //datatype
    if (FMT_fmt >= 16) {
      error = true;
      Serial.printf("BB: ERROR too many fields for msg %s\n", FMT.name);
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
      Serial.printf("BB: ERROR labels too long for msg %s\n", FMT.name);
      return;
    }
    if (FMT_lbl) FMT.labels[FMT_lbl++] = ',';
    strcpy(FMT.labels+FMT_lbl, label);
    FMT_lbl += lbl_len;
  }

private:
  void msg_begin(const char* name) {
    msgType = typeRegistry_find(name); //this sets FMT_write when new message name is encountered
    
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
    //only queue message if BinLog started or when writing file header
    if(error || inhibit || (BinLog::state!=STARTED && !isHeader) ) return;

    if(FMT_write) {
      //queue FMT
      FMT.length = buflen; //store message length in FMT message
      if(!queueSend((uint8_t*)&FMT, sizeof(FMT))) {
        return;
      }
      //queue FMTU
      if(!queueSendFMTU(false, msgType, FMT_unit, FMT_mult)) {
        return;
      }
      typeRegistry_FMT_was_sent(msgType);
    }

    //queue message
    queueSend(buf,buflen, keepFree); //keep a few queue spots free for more important messages (such as FMT)

    inhibit = true;
  }

  bool msg_queueFMT() {
    if(!FMT_write) return true; //FMT message was already sent, nothing to do
    FMT.length = buflen;
    return queueSend((uint8_t*)&FMT, sizeof(FMT));
  }

  bool msg_queue() {
    return queueSend(buf,buflen);
  }

  //-------------------------------
  // Message fields
  //-------------------------------
public:
  void TimeUS() {
    TimeUS(micros());
  }
  void TimeUS(uint32_t ts) {
    u32("TimeUS", ts - BinLog::startMicros, 1e-6, "s");
  }  
  void u8(const char* label, uint8_t v, float mult = 1, const char* unit = NULL) {
    if(FMT_write) FMT_addField('B', 1, label, mult, unit); // B   : uint8_t
    buf[buflen++] = v;
  }
  void u8flightmode(const char* label, uint8_t v, float mult = 1, const char* unit = NULL) {
    if(FMT_write) FMT_addField('M', 1, label, mult, unit); // M   : uint8_t flight mode
    buf[buflen++] = v;
  }  
  void u16(const char* label, uint16_t v, float mult = 1, const char* unit = NULL) {
    if(FMT_write) FMT_addField('H', 2, label, mult, unit); // H   : uint16_t
    memcpy(buf+buflen, &v, 2);
    buflen+=2;
  }
  void u16x100(const char* label, uint16_t v, float mult = 1, const char* unit = NULL) { 
    if(FMT_write) FMT_addField('C', 2, label, mult, unit); // C   : uint16_t * 100
    memcpy(buf+buflen, &v, 2);
    buflen+=2;
  }  
  void u32(const char* label, uint32_t v, float mult = 1, const char* unit = NULL) {
    if(FMT_write) FMT_addField('I', 4, label, mult, unit); // I   : uint32_t
    memcpy(buf+buflen, &v, 4);
    buflen+=4;
  }
  void u32x100(const char* label, uint32_t v, float mult = 1, const char* unit = NULL) {
    if(FMT_write) FMT_addField('E', 4, label, mult, unit); // E   : uint32_t * 100
    memcpy(buf+buflen, &v, 4);
    buflen+=4;
  }
  void u64(const char* label, uint64_t v, float mult = 1, const char* unit = NULL) {
    if(FMT_write) FMT_addField('Q', 8, label, mult, unit); // Q   : uint64_t
    memcpy(buf+buflen, &v, 8);
    buflen+=8;
  }  
  void i8(const char* label, int8_t v, float mult = 1, const char* unit = NULL) {
    if(FMT_write) FMT_addField('b', 1, label, mult, unit); // b   : int8_t
    buf[buflen++] = v;
  }
  void i16(const char* label, int16_t v, float mult = 1, const char* unit = NULL) {
    if(FMT_write) FMT_addField('h', 2, label, mult, unit); // h   : int16_t
    memcpy(buf+buflen, &v, 2);
    buflen+=2;
  }
  void i16x100(const char* label, int16_t v, float mult = 1, const char* unit = NULL) {
    if(FMT_write) FMT_addField('c', 2, label, mult, unit); // c   : int16_t * 100
    memcpy(buf+buflen, &v, 2);
    buflen+=2;
  }
  void i32(const char* label, int32_t v, float mult = 1, const char* unit = NULL) {
    if(FMT_write) FMT_addField('i', 4, label, mult, unit); // i   : int32_t
    memcpy(buf+buflen, &v, 4);
    buflen+=4;
  }
  void i32x100(const char* label, int32_t v, float mult = 1, const char* unit = NULL) {
    if(FMT_write) FMT_addField('e', 4, label, mult, unit); // e   : int32_t * 100
    memcpy(buf+buflen, &v, 4);
    buflen+=4;
  }
  void i32latlon(const char* label, int32_t v, float mult = 1, const char* unit = NULL) {
    if(FMT_write) FMT_addField('L', 4, label, mult, unit); // L   : int32_t latitude/longitude
    memcpy(buf+buflen, &v, 4);
    buflen+=4;
  }
  void i64(const char* label, int64_t v, float mult = 1, const char* unit = NULL) {
    if(FMT_write) FMT_addField('q', 8, label, mult, unit); // q   : int64_t
    memcpy(buf+buflen, &v, 8);
    buflen+=8;
  }   
  void flt(const char* label, float v, float mult = 1, const char* unit = NULL) {
    if(FMT_write) FMT_addField('f', 4, label, mult, unit); // f   : float
    memcpy(buf+buflen, &v, 4);
    buflen+=4;
  }
  void dbl(const char* label, double v, float mult = 1, const char* unit = NULL) {
    if(FMT_write) FMT_addField('d', 8, label, mult, unit); // d   : double
    memcpy(buf+buflen, &v, 8);
    buflen+=8;
  }
  void char4(const char* label, const char* v, float mult = 1, const char* unit = NULL) {
    if(FMT_write) FMT_addField('n', 4, label, mult, unit); // n   : char[4]
    strncpy((char*)(buf+buflen), v, 4);
    buflen+=4;
  }
  void char16(const char* label, const char* v, float mult = 1, const char* unit = NULL) {
    if(FMT_write) FMT_addField('N', 16, label, mult, unit);  // N   : char[16]
    strncpy((char*)(buf+buflen), v, 16);
    buflen+=16;
  }
  void char64(const char* label, const char* v, float mult = 1, const char* unit = NULL) {
    if(FMT_write) FMT_addField('Z', 64, label, mult, unit);  // Z   : char[64]
    strncpy((char*)(buf+buflen), v, 64);
    buflen+=64;
  }
  void blob64(const char* label, const int16_t* v, float mult = 1, const char* unit = NULL) {
    if(FMT_write) FMT_addField('a', 64, label, mult, unit);  // a   : int16_t[32]
    memcpy(buf+buflen, v, 64); 
    buflen+=64;
  }


//static


  //-------------------------------
  // Black Box interface
  //-------------------------------
public:
  enum Command_t { NONE, START, STOP };
  static Command_t command;
  enum State_t { READY, STARTING, STARTED };
  static State_t state;
  static uint32_t startMicros;
  static uint32_t missCnt;

  static void setup() {
    queue = xQueueCreateStatic(QUEUE_LENGTH, sizeof(msg_t), ucQueueStorageArea, &xStaticQueue);
    if(xTaskCreate(bb_task, "BB", FREERTOS_DEFAULT_STACK_SIZE, NULL, uxTaskPriorityGet(NULL), &xHandle) != pdPASS ){
      Serial.println("BB: Task creation failed");
    }
  }

private:
  static void cmd_start() {
    if(state!=READY) return;
    state = STARTING;

    //empty the queue
    xQueueReset(queue);

    //clear type registry, force writing of FMT records
    typeRegistry_len = 0;

    //open bbfs (black box file system)
    if(!bbfs.writeOpen()) return; //bbfs emits error message

    //log file time start now
    startMicros = micros();
    missCnt = 0;

    //write headers (flush often)
    FMT_sendFMT(); //send FMT for FMT
    queueSendFMTU(true,0,"",""); //send FMT for FMTU
    log_header_msg("ArduPlane"); //this sets the vehicle type -> which drives the translaton of flightmode codes to names (among other things probably)
    //log_header_msg("ArduCopter");  //gives problems with plot.ardupilot.org
    log_header_msg(MADFLIGHT_VERSION);
    queueFlush();
    
    //write multipliers
    for(int i=0;i<_num_multipliers;i++) {
      log_header_mult(log_Multipliers[i].ID, log_Multipliers[i].mult);
      queueFlush();
    }
    
    //write units
    for(int i=0;i<_num_units;i++) {
      log_header_unit(log_Units[i].ID, log_Units[i].unit);
      queueFlush();
    }
    
    //write parameters (plot.ardupilot.org needs at least one)
    String name;
    float value = 0;
    int i = 0;
    while(cfg.getNameAndValue(i,&name,&value)) {
      log_header_parm(name.c_str(), value, 0);
      queueFlush();
      i++;
    }

    //allow logging
    state = STARTED;
  }
  
  static void cmd_stop() {
    //stop logging
    state = READY;

    //flush queue to sdcard
    queueFlush();

    //close file
    bbfs.close();
  }
public:
  static void stop() {
    command = STOP;
    //send zero length message to wake up BB task to process command
    uint8_t msg = 0;
    xQueueSend(queue, (void*)&msg, 0);
  }

  static void start() {
    command = START;
    //send zero length message to wake up BB task to process command
    uint8_t msg = 0;
    xQueueSend(queue, (void*)&msg, 0);
  }

  //-------------------------------
  // Message Queue
  //-------------------------------
private:
  struct msg_t
  {
      uint8_t buf[MAX_MSG_LEN]; //NOTE: first byte overwritten with msg len when stuffed in queue
  };

  static QueueHandle_t queue;
  static StaticQueue_t xStaticQueue;
  static uint8_t ucQueueStorageArea[QUEUE_LENGTH * sizeof(msg_t)];


  //append message to queue
  static bool queueSend(uint8_t *buf, uint8_t len, uint8_t keepfree = 0) {
    if(keepfree && uxQueueSpacesAvailable(queue) < keepfree) return false;
    //NOTE: random data will pad the message - improve this?
    buf[0] = len; //overwrite header1 with message length
    bool ok = ( xQueueSend(queue, (void*)buf, 0) == pdPASS );
    if(!ok) missCnt++;
    return ok;
  }

private:

  //flush queue to bb filesystem
  static void queueFlush() {
    msg_t msg;
    while( xQueueReceive(queue, &msg, 0) == pdPASS ) {
      //extract message length and restore header1
      uint8_t len = msg.buf[0];
      msg.buf[0] = HEAD_BYTE1; 
      bbfs.write(msg.buf, len);
    }
  }


  //send FMT-FMT message to queue
  static void FMT_sendFMT() {
    struct PACKED {
      uint8_t h1 = HEAD_BYTE1;
      uint8_t h2 = HEAD_BYTE2;
      uint8_t type = 0x80;
      uint8_t msg_type = 0x80;
      uint8_t length = 0x59;
      char name[4] = "FMT";
      char format[16] = "BBnNZ";
      char labels[64] = "Type,Length,Name,Format,Columns";
    } FMT;
    queueSend((uint8_t*)&FMT, sizeof(FMT));
/*
    const uint8_t msg_fmt[] = {
    0xA3, 0x95, 0x80, 0x80, 0x59, 0x46, 0x4D, 0x54, 0x00, 0x42, 0x42, 0x6E, 0x4E, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x54, 0x79, 0x70, 0x65, 0x2C, 0x4C, 0x65, 0x6E, 0x67, 0x74, 0x68, 0x2C, 0x4E, 0x61, 0x6D, 0x65, 0x2C, 0x46, 0x6F, 0x72, 0x6D, 0x61, 0x74, 0x2C, 0x43, 0x6F, 0x6C, 0x75, 0x6D, 0x6E, 0x73, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    queueSend(msg_fmt, sizeof(msg_fmt));
*/
  }

  //-------------------------------
  // Message type registry, gives out msg type IDs and keeps track if FMT/FMTU headers were written
  //-------------------------------
private:
  static uint32_t typeRegistry[LOG_TYPE_LEN]; //MSB is FMT+FMTU written flag, 31 LSB is message name 
  static uint8_t typeRegistry_len;

  uint8_t typeRegistry_find(const char *name) {
    uint8_t i;
    uint32_t findname = 0;
    strncpy((char*)&findname, name, 4);
    for(i=0;i<typeRegistry_len;i++) {
      if((typeRegistry[i] & 0x7fffffff) == findname) {
        FMT_write = ((typeRegistry[i] & 0x80000000) == 0); //get FMT written flag
        return i + 0x81;
      }
    }
    if(i<LOG_TYPE_LEN) {
      //add new type
      typeRegistry[typeRegistry_len] = findname;
      typeRegistry_len++;
      FMT_write = true;
      return i + 0x81;
    }
    //too many types, return next type id but don't write FMT record
    FMT_write = false;
    return i + 0x81;
  }
  
  static void typeRegistry_FMT_was_sent(char typ){
    uint8_t i = typ - 0x81;
    if(i<LOG_TYPE_LEN) {
      typeRegistry[i] |= 0x80000000; //set FMT written flag
    }
  }
  


  //-------------------------------
  // Messages used in headers (bypass started flag by setting isHeader)
  //-------------------------------
private:
  static void log_header_msg(const char* msg, bool isHeader = true) {
      BinLog bl("MSG");
      bl.isHeader = isHeader;
      bl.TimeUS();
      bl.char64("Message",msg);
  }

  static void log_header_parm(const char* name, float value, float default_value, bool isHeader = true) {
      BinLog bl("PARM");  //PARM parameter value
      bl.isHeader = isHeader;
      bl.TimeUS(); //TimeUS: Time since system startup
      bl.char16("Name", name); //Name: parameter name
      bl.flt("Value", value); //Value: parameter value
      bl.flt("Default", default_value); //Default: default parameter value for this board and config
  }

  static void log_header_mult(char id, float mult) {
      BinLog bl("MULT");  
      bl.isHeader = true;
      bl.TimeUS(); //uint64_t TimeUS: Time since system startup
      bl.i8("Id", id); //char Id: character referenced by FMTU
      bl.flt("Mult", mult); //double Mult: numeric multiplier
  }
  
  static void log_header_unit(char id, const char *label) {
      BinLog bl("UNIT");  
      bl.isHeader = true;
      bl.TimeUS(); //uint64_t TimeUS: Time since system startup
      bl.i8("Id", id); //char Id: character referenced by FMTU
      bl.char16("Label", label); //char[64] Label: Unit - SI where available
  }

  static bool queueSendFMTU(bool sendFMT, uint8_t type, const char *units, const char* multipliers) {
      BinLog bl("FMTU"); //Message defining units and multipliers used for fields of other messages
      bl.isHeader = true;
      bl.TimeUS(); //uint64_t TimeUS: Time since system startup
      bl.u8("FmtType", type); //char FmtType: numeric reference to associated FMT message
      bl.char16("UnitIds", units); //char[16] UnitIds: each character refers to a UNIT message.  The unit at an offset corresponds to the field at the same offset in FMT.Format
      bl.char16("MultIds", multipliers); //char[16] MultIds: each character refers to a MULT message.  The multiplier at an offset corresponds to the field at the same offset in FMT.Format
      
      bool queued = false;
      if(sendFMT) {
        queued = bl.msg_queueFMT(); //send FMT for FMTU
      }else{
        queued = bl.msg_queue(); //send FMTU
      }
      bl.inhibit = true; //don't send anything in destructor
      return queued;
  }


public:
  static void log_msg(const char* msg) {
    log_header_msg(msg, false);
  }

  static void log_parm(const char* name, float value, float default_value) {
    log_header_parm(name, value, default_value, false);
  }

private:
  static TaskHandle_t xHandle;
  static void bb_task(void *pvParameters) {
    msg_t msg;
    for(;;) {
      if( xQueueReceive(queue, &msg, portMAX_DELAY) == pdPASS ) {
        switch(command) {
        case START:
          command = NONE;
          cmd_start();
          break;
        case STOP:
          command = NONE;
          cmd_stop();
          break;
        case NONE:
          //extract message length and restore header1
          uint8_t len = msg.buf[0];
          msg.buf[0] = HEAD_BYTE1;
          bbfs.write(msg.buf, len);
        }
      }
    }
  }
};

//BinLog static var init
uint8_t BinLog::typeRegistry_len = 0;
uint32_t BinLog::startMicros = 0;
uint32_t BinLog::typeRegistry[LOG_TYPE_LEN] = {};
QueueHandle_t BinLog::queue = NULL;
StaticQueue_t BinLog::xStaticQueue = {};
uint8_t BinLog::ucQueueStorageArea[QUEUE_LENGTH * sizeof(msg_t)] = {};
BinLog::State_t BinLog::state = READY;
BinLog::Command_t BinLog::command = NONE;
uint32_t BinLog::missCnt = 0;



    
TaskHandle_t BinLog::xHandle = NULL;


//=====================================================================================================================
// Black Box public interface
//=====================================================================================================================

class BlackBox_SD : public BlackBox {
  public:
  //-------------------------------
  //loggers
  //-------------------------------


    void log_baro() override {
      BinLog bl("BARO");
      bl.TimeUS();                      //uint64_t TimeUS: Time since system startup [us]
      bl.u8("I", 0, 1, "instance");     //uint8_t I: barometer sensor instance number [-]
      bl.flt("Alt", baro.alt, 1, "m");  //float Alt: calculated altitude [m]
                                        //float AltAMSL: altitude AMSL
      bl.flt("Press", baro.press, 1, "Pa");      //float Press: measured atmospheric pressure [Pa]
                                        //int16_t Temp: measured atmospheric temperature
      bl.flt("CRt", baro.vz, 1, "m/s"); //float CRt: derived climb rate from primary barometer
                                        //uint32_t SMS: time last sample was taken
                                        //float Offset: raw adjustment of barometer altitude, zeroed on calibration, possibly set by GCS
                                        //float GndTemp: temperature on ground, specified by parameter or measured while on ground
                                        //uint8_t Health: true if barometer is considered healthy
      //non-standard
      bl.flt("AltRaw", baro.altRaw, 1, "m");
    } 

    void log_bat() override {
      BinLog bl("BAT");
      bl.TimeUS();                      //uint64_t TimeUS: Time since system startup [us]
      bl.u8("I", 0, 1, "instance");     //uint8_t Inst: battery instance number [-]
      bl.flt("Volt", bat.v, 1, "V");    //float Volt: measured voltage [V]
                                        //float VoltR: estimated resting voltage
      bl.flt("Curr", bat.i, 1, "A");    //float Curr: measured current [A]
      bl.flt("CurrTot", bat.mah*1000, 1, "Ah");  //float CurrTot: consumed Ah, current * time [Ah]
      bl.flt("EnrgTot", bat.wh, 1, "Wh");  //float EnrgTot: consumed Wh, energy this battery has expended [Wh]
                                        //int16_t Temp: measured temperature
                                        //float Res: estimated battery resistance
                                        //uint8_t RemPct: remaining percentage
                                        //uint8_t H: health
                                        //uint8_t SH: state of health percentage.  0 if unknown
    } 

//Ardupilot definition: { "GPS",
//"QBBIHBcLLeffffB", "TimeUS,I,Status,GMS,GWk,NSats,HDop,Lat,Lng,Alt,Spd,GCrs,VZ,Yaw,U", 
//"s#-s-S-DUmnhnh-", 
//"F--C-0BGGB000--" , true }
    void log_gps() override {
      BinLog bl("GPS");                 // Information received from GNSS systems attached to the autopilot
      bl.TimeUS();                      //uint64_t TimeUS 1e-6 [s]: Time since system startup [us]
      bl.u8("I", 0, 1, "instance");     //uint8_t I 0 [instance]: GPS instance number
      bl.u8("Status", gps.fix);         //uint8_t Status 0 []: GPS Fix type; 2D fix, 3D fix etc. --madflight 0:no fix, 1:fix 2:2D fix, 3:3D fix)
      //NOTE: gps.time is time in milliseconds since midnight UTC
      bl.u32("GMS", gps.time);          //uint32_t GMS 1e-3 [s]: milliseconds since start of GPS Week [ms]
      bl.u16("GWk", 2288);              //uint16_t GWk 0 []: weeks since 5 Jan 1980 [week]
      bl.u8("NSats", gps.sat, 1, "satellites"); //uint8_t NSats '0':1e0 ['S':satellites]: number of satellites visible [-]
      bl.i16x100("HDop", gps.hdop, 1, "m");     //int16_t*100 HDop '-':0 ['-']: horizontal dilution of precision [-]
      bl.i32latlon("Lat", gps.lat, 1e-7, "deglatitude");     //int32_t Lat 1e-7 ['D':deglatitude]: latitude [deg*10e7]
      bl.i32latlon("Lng", gps.lon, 1e-7, "deglongitude");     //int32_tLng 1e-7 ['U':deglongitude]: longitude [deg*10e7]
      bl.flt("Alt", gps.alt/1000.0, 1, "m");           //i32x100 Alt 'B':1e-2 ['m':m]: altitude [mm]
      bl.flt("Spd", gps.sog/1000.0, 1, "m/s");           //float Spd 1 ['n':m/s]: ground speed [mm/s]
      bl.i32("GCrs",gps.cog, 1, "degheading");     //float GCrs 1 ['h':degheading]: ground course [deg]
      bl.flt("VZ", gps.veld/1000.0, 1, "m/s");           //float VZ 1 ['n':m/s]: vertical speed [mm/s]
      bl.flt("Yaw", 0, 1, "degheading");           //float Yaw 1 ['h':degheading]: vehicle yaw
      bl.u8("U",1);                     //U: boolean value indicating whether this GPS is in use

      //non standard (use short names!!!)
      //bl.u32("D",gps.date);  //date as DDMMYY
    }

/*
    void log_pos(float homeAlt, float OriginAlt) override {
      BinLog bl("POS");
      bl.TimeUS(); //uint64_t TimeUS 1e-6 [s]: Time since system startup [us]
      bl.i32latlon("Lat", gps.lat, 1e-7, "deglatitude");
      bl.i32latlon("Lng", gps.lon, 1e-7, "deglongitude");
      bl.flt("Alt", gps.alt/1000.0, 1, "m");;
      bl.flt("RelHomeAlt", gps.alt/1000.0 - homeAlt, 1, "m");
      bl.flt("RelOriginAlt", gps.alt/1000.0 - OriginAlt, 1, "m");
    }
*/

    //AHRS roll/pitch/yaw plus filtered+corrected IMU data
    void log_ahrs() override {
      BinLog bl("AHRS"); 
      bl.TimeUS();
      bl.i16("ax",ahrs.ax*100, 1e-2, "G"); //G
      bl.i16("ay",ahrs.ay*100, 1e-2, "G"); //G
      bl.i16("az",ahrs.az*100, 1e-2, "G"); //G
      bl.i16("gx",ahrs.gx*10, 1e-1, "deg/s"); //dps
      bl.i16("gy",ahrs.gy*10, 1e-1, "deg/s"); //dps
      bl.i16("gz",ahrs.gz*10, 1e-1, "deg/s"); //dps
      bl.i16("mx",ahrs.mx*100, 1e-2, "uT"); //uT
      bl.i16("my",ahrs.my*100, 1e-2, "uT"); //uT
      bl.i16("mz",ahrs.mz*100, 1e-2, "uT"); //uT
      bl.i16("roll",ahrs.roll*100, 1e-2, "deg"); //deg -180 to 180
      bl.i16("pitch",ahrs.pitch*100, 1e-2, "deg");; //deg -90 to 90
      bl.i16("yaw",ahrs.yaw*100, 1e-2, "deg");; //deg -180 to 180
    }

    void log_att() override {
      BinLog bl("ATT");
      bl.TimeUS();
      bl.i16("DesRoll",0, 1e-2, "deg");
      bl.i16("Roll",ahrs.roll*100, 1e-2, "deg");
      bl.i16("DesPitch",0, 1e-2, "deg");
      bl.i16("Pitch",-ahrs.pitch*100, 1e-2, "deg");
      bl.u16("DesYaw",0, 1e-2, "deg");
      bl.u16("Yaw",ahrs.yaw*100, 1e-2, "deg");
      bl.u16x100("ErrRP",0);
      bl.u16x100("ErrYaw",0);
      bl.u8 ("AEKF",3);
    }

    //raw (unfiltered but corrected) IMU data
    void log_imu() override {
      BinLog bl("IMU");
      bl.keepFree = QUEUE_LENGTH/4; //keep 25% of queue free for other messages
      bl.TimeUS(imu.ts);
      bl.i16("ax",(imu.ax - cfg.IMU_CAL_AX)*100, 1e-2, "G"); //G
      bl.i16("ay",(imu.ay - cfg.IMU_CAL_AY)*100, 1e-2, "G"); //G
      bl.i16("az",(imu.az - cfg.IMU_CAL_AZ)*100, 1e-2, "G"); //G
      bl.i16("gx",(imu.gx - cfg.IMU_CAL_GX)*10, 1e-1, "deg/s"); //dps
      bl.i16("gy",(imu.gy - cfg.IMU_CAL_GY)*10, 1e-1, "deg/s"); //dps
      bl.i16("gz",(imu.gz - cfg.IMU_CAL_GZ)*10, 1e-1, "deg/s"); //dps
      #if MAG_USE != MAG_USE_NONE
        //get from magnetometer
        bl.i16("mx",((mag.x - cfg.MAG_CAL_X) * cfg.MAG_CAL_SX)*100, 1e-2, "uT"); //uT
        bl.i16("my",((mag.y - cfg.MAG_CAL_Y) * cfg.MAG_CAL_SY)*100, 1e-2, "uT"); //uT
        bl.i16("mz",((mag.z - cfg.MAG_CAL_Z) * cfg.MAG_CAL_SZ)*100, 1e-2, "uT"); //uT
      #else
        //get from imu
        if(imu.hasMag()) {
          bl.i16("mx",((imu.mx - cfg.MAG_CAL_X) * cfg.MAG_CAL_SX)*100, 1e-2, "uT"); //uT
          bl.i16("my",((imu.my - cfg.MAG_CAL_Y) * cfg.MAG_CAL_SY)*100, 1e-2, "uT"); //uT
          bl.i16("mz",((imu.mz - cfg.MAG_CAL_Z) * cfg.MAG_CAL_SZ)*100, 1e-2, "uT"); //uT
        }
      #endif
      bl.i16("roll",ahrs.roll*100, 1e-2, "deg"); //deg -180 to 180
      bl.i16("pitch",ahrs.pitch*100, 1e-2, "deg");; //deg -90 to 90
      bl.i16("yaw",ahrs.yaw*100, 1e-2, "deg");; //deg -180 to 180
    }

    void log_mode(uint8_t fm, const char* name) override {
      BinLog bl("MODE");
      bl.TimeUS();
      bl.u8flightmode("Mode",fm);
      bl.u8("ModeNum",fm);
      bl.u8("Rsn",1); //ModeReason
      bl.char16("Name",name); //extenstion to "standard" ArduPilot BinLog
    }

    void log_msg(const char* msg) {
      BinLog::log_msg(msg);
    }

    void log_parm(const char* name, float value, float default_value) {
      BinLog::log_parm(name, value, default_value);
    }

    //system status
    void log_sys() override {
      BinLog bl("SYS");
      bl.TimeUS();
      bl.u32("BBm",BinLog::missCnt);
      bl.u32("IMi",imu.interrupt_cnt);
      bl.i32("IMm",imu.interrupt_cnt - imu.update_cnt);
    }

  //-------------------------------
  // Blackbox interface
  //-------------------------------
public:
    void setup() override {
      BinLog::setup();
      bbfs.setup();
    }

    void start() override {
      BinLog::start();
    }

    void stop() override {
      BinLog::stop();
    }

    void erase() override {
      stop();
      bbfs.erase();
    }

    void dir() override {
      bbfs.dir();
    }

    void bench() override {
      bbfs.bench();
    }

    void info() override {
      bbfs.info();
    }
};

BlackBox_SD bb_instance;
BlackBox &bb = bb_instance;