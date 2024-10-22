/*==========================================================================================
BB: madflight black box data logger in ArduPilot Binary Log data format

MIT License

Copyright (c) 2024 qqqlab - https://github.com/qqqlab

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

#define BB_USE_NONE 1
#define BB_USE_SDCARD 2 //SDCARD with MMC interface
#define BB_USE_DEBUG 3 //print log to Serial


//uncomment to use 32bit timestamp (good for 1 hour of recording before wrap-around)
#define LOG_TIMEUS_U32

#define LOG_TYPE_LEN 64 //max number of message types

#define HEAD_BYTE1  0xA3    // Decimal 163
#define HEAD_BYTE2  0x95    // Decimal 149

//black box file system
class BBFS {
public:
  virtual void setup() = 0; //setup the file system (can be called multiple times)
  
  virtual bool writeOpen() = 0; //create new file for writing (closes previously opened file first)
  virtual void writeChar(uint8_t c) = 0; //write char to file
  virtual void write(const uint8_t *buf, const uint8_t len) = 0;
  virtual void close() = 0; //close file

  virtual void erase() = 0; //erase all files
  virtual void dir() = 0; //list files
  virtual void bench() = 0; //benchmark read/write
};

//=====================================================================================================================
// Logging to Serial
//=====================================================================================================================
#if BB_USE == BB_USE_DEBUG

class BBFS_Debug : public BBFS {
public:
  void setup() override {}
  bool writeOpen() override {return true;}

  void writeChar(uint8_t c) override {
    static int cnt=0;
    Serial.printf("%02X ", c);
    cnt++;
    if(cnt==32) {
      Serial.println();
      cnt=0;
    }
  }

  void write(const uint8_t *buf, const uint8_t len) override {
    for(int i=0;i<len;i++) writeChar(buf[i]);
  }
  
  void close() override {}
  void erase() override {}
  void dir() override {}
};
BBFS_Debug bbfs;


//=====================================================================================================================
// Logging to SDCARD with MMC interface
//=====================================================================================================================
#elif BB_USE == BB_USE_SDCARD

#if !defined(HW_PIN_SDMMC_CLK) || HW_PIN_SDMMC_CLK == -1 || !defined(HW_PIN_SDMMC_CMD) || HW_PIN_SDMMC_CMD == -1 || !defined(HW_PIN_SDMMC_DATA) || HW_PIN_SDMMC_DATA == -1
  #error BB_USE_SDMMC needs HW_PIN_SDMMC_CLK, HW_PIN_SDMMC_CMD, HW_PIN_SDMMC_DATA
#endif

#include "SD_MMC.h"

class BBFS_SDMMC : public BBFS {
private:
  const char* BB_LOG_DIR_NAME = "/log";
  bool setup_done = false;
  uint8_t wbuf[512];
  int wbuf_len = 0;
  String filename;
  fs::FS fs = SD_MMC;
  File file;

public:
  //setup the file system
  void setup() override {
    Serial.printf("BB: BB_USE_SDCARD Interface=MMC ");
    mmc_setup();
  }

  bool writeOpen() override {
    close();

    int attempt = 0;
    while(++attempt<2) {
      if(mmc_setup()) {
        memset(wbuf, 0xff, sizeof(wbuf));
        wbuf_len = 0;

        mmc_logOpen(fs, BB_LOG_DIR_NAME);
        if(file) {
          Serial.printf("BB: start OK - attempt=%d file=%s\n", attempt, filename.c_str());
          return true;
        }
      }

      setup_done = false; //force setup to re-run
    }
    Serial.println("BB: start FAILED");
    return false;
  }

  void writeChar(uint8_t c) override {
    if(!file) return;
    if(wbuf_len < sizeof(wbuf)) {
     wbuf[wbuf_len++] = c;
    }
    if(wbuf_len >= sizeof(wbuf)) {
      file.write(wbuf, sizeof(wbuf));
      file.flush();
      memset(wbuf, 0xff, sizeof(wbuf));
      wbuf_len = 0;
    }
  }

  void write(const uint8_t *buf, const uint8_t len) override {
    for(int i=0;i<len;i++) writeChar(buf[i]);
  }

  void close() override {
    if(wbuf_len>0) {
      file.write(wbuf, wbuf_len);
      wbuf_len = 0;
    }
    file.close();
  }

  void erase() override {
    if(mmc_setup()) {
      mmc_deleteFilesFromDir(fs, BB_LOG_DIR_NAME);
    }
  }

  void dir () override {
    int attempt = 0;
    while(++attempt<2) {
      if(mmc_setup()) {
        if(mmc_listDir(fs, BB_LOG_DIR_NAME, 0)) return;
      }
      setup_done = false; //force setup to re-run
    }
  }


  void bench() override {
      const char* path = "/madfli.ght";
      uint8_t *buf;
      size_t len = 0;
      uint32_t start;
      uint32_t end;
      size_t flen;
      File file;
      
      buf = (uint8_t*)malloc(512);
      if(!buf) {
          Serial.println("BB: bench - malloc failed");
          return;
      }
      
      file = fs.open(path, FILE_WRITE);
      if(!file){
          Serial.println("BB: bench - Failed to open file for writing");
          free(buf);
          return;
      }

      size_t i;
      for(i=0;i<512;i++) buf[i] = i%64<63 ? '0' + i%64 - 1 : '\n';

      start = millis();
      for(i=0; i<2048; i++){
          sprintf((char*)buf,"%u ",i);
          file.write(buf, 512);
      }
      end = millis() - start;
      flen = 2048 * 512;
      Serial.printf("BB: %s %u bytes written in %u ms %f kbps\r\n", path, (int)flen, (int)end, (float)flen/end);
      file.close();
      
      file = fs.open(path);
      if(!file){
          Serial.println("BB: bench - Failed to open file for reading");
          free(buf);
          return;
      }        
      len = file.size();
      flen = len;
      start = millis();
      while(len){
          size_t toRead = len;
          if(toRead > 512){
              toRead = 512;
          }
          file.read(buf, toRead);
          len -= toRead;
      }
      end = millis() - start;
      Serial.printf("BB: %s %u bytes read in %u ms %f kbps\r\n", path, (int)flen, (int)end, (float)flen/end);
      file.close();

      free(buf);
  }


private:

  bool mmc_setup() {
    if(setup_done) return true;
    
    SD_MMC.end(); //force begin() to re-initialize MMC
    
    SD_MMC.setPins(HW_PIN_SDMMC_CLK, HW_PIN_SDMMC_CMD, HW_PIN_SDMMC_DATA);
    if (!SD_MMC.begin("/sdcard", true, true, SDMMC_FREQ_DEFAULT, 5)) {
      Serial.println("Card Mount Failed");
      setup_done = false;
      return setup_done;
    }

    uint8_t cardType = SD_MMC.cardType();
    if(cardType == CARD_NONE){
      Serial.println("No SD_MMC card attached");
      setup_done = false;
      return setup_done;
    }
    Serial.print("CardType=");
    if(cardType == CARD_MMC){
        Serial.print("MMC");
    } else if(cardType == CARD_SD){
        Serial.print("SD");
    } else if(cardType == CARD_SDHC){
        Serial.print("SDHC");
    } else {
        Serial.print("UNKNOWN");
    }

    Serial.printf(" SectorSize=%d", SD_MMC.sectorSize());
    Serial.printf(" CardSize=%lluMB", SD_MMC.cardSize() / (1024 * 1024));
    Serial.printf(" used:%lluMB", SD_MMC.usedBytes()/ (1024 * 1024));
    Serial.printf(" free:%lluMB", (SD_MMC.totalBytes() - SD_MMC.usedBytes()) / (1024 * 1024));
    Serial.println();

    setup_done = true;
    return setup_done;
  }

  bool mmc_listDir(fs::FS &fs, const char * dirname, uint8_t levels){
      Serial.printf("Listing directory: %s\n", dirname);

      File root = fs.open(dirname);
      if(!root){
          Serial.println("Failed to open directory");
          return false;
      }
      if(!root.isDirectory()){
          Serial.println("Not a directory");
          return true;
      }

      File file = root.openNextFile();
      while(file){
          if(file.isDirectory()){
              Serial.print("DIR:");
              Serial.println(file.name());
              if(levels){
                  mmc_listDir(fs, file.path(), levels -1);
              }
          } else {
              Serial.print(file.name());
              Serial.print("   ");
              Serial.println(file.size());
          }
          file = root.openNextFile();
      }

    Serial.printf("SDCARD: used:%lluMB", SD_MMC.usedBytes()/ (1024 * 1024));
    Serial.printf(" free:%lluMB\n", (SD_MMC.totalBytes() - SD_MMC.usedBytes()) / (1024 * 1024));
    return true;
  }

  void mmc_deleteFilesFromDir(fs::FS &fs, const char * dirname){
      File root = fs.open(dirname);
      if(!root) return;
      if(!root.isDirectory()) return;

      File file = root.openNextFile();
      while(file){
          if(!file.isDirectory()){
            fs.remove(file.path());
          }
          file = root.openNextFile();
      }
  }

  void mmc_logOpen(fs::FS &fs, const char * dirname){
      File root = fs.open(dirname);
      if(!root){
         if(!fs.mkdir(dirname)){
            Serial.println("BB: mkdir /log failed");
            return;
        }
        root = fs.open(dirname);
        if(!root) {
            Serial.println("BB: mkdir /log failed");
            return;
        }
      }
      if(!root.isDirectory()){
          Serial.println("BB: /log is not a directory");
          return;
      }

      File dirfile = root.openNextFile();
      int maxnr = 0;
      while(dirfile){
        String fn = String(dirfile.name());
        if(fn.endsWith(".bin")) {
          int nr = fn.toInt();
          if(maxnr<nr) maxnr = nr;
        }
        dirfile = root.openNextFile();
      }
      filename = String(dirname) + '/' + String(maxnr+1) + ".bin";
      Serial.println(filename);
      file = fs.open(filename, FILE_WRITE, true);
  }
};

BBFS_SDMMC bbfs;

//=====================================================================================================================
// No Logging
//=====================================================================================================================
#else

class BBFS_None : public BBFS {
public:
  void setup() override {}
  bool writeOpen() override {return false;}
  void writeChar(uint8_t c) override {}
  void write(const uint8_t *buf, const uint8_t len) override {}
  void close() override {}
  void erase() override {}
  void dir() override {}
  void bench() override {}
};
BBFS_None bbfs;
#endif




class BinLog {
  //-------------------------------
  //locking handling
  //-------------------------------
private:
  static bool locked;
public:
  static bool getLock() {
    if(!started) return false;
    if(locked) return false;
    locked = true;
    return true;
  }

  //-------------------------------
  //FMT handling
  //-------------------------------
private:
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
    bbfs.write((uint8_t*)&FMT, sizeof(FMT));
/*
    const uint8_t msg_fmt[] = {
    0xA3, 0x95, 0x80, 0x80, 0x59, 0x46, 0x4D, 0x54, 0x00, 0x42, 0x42, 0x6E, 0x4E, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x54, 0x79, 0x70, 0x65, 0x2C, 0x4C, 0x65, 0x6E, 0x67, 0x74, 0x68, 0x2C, 0x4E, 0x61, 0x6D, 0x65, 0x2C, 0x46, 0x6F, 0x72, 0x6D, 0x61, 0x74, 0x2C, 0x43, 0x6F, 0x6C, 0x75, 0x6D, 0x6E, 0x73, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    bbfs.write(msg_fmt, sizeof(msg_fmt));
*/
  }

  void FMT_addField(const char fmt, const char* label) {
    if (FMT_fmt >= 16) {
      FMT.format[0]=0;
      Serial.printf("BB: ERROR too many fields for msg %s\n", FMT.name);
      return;
    }
    FMT.format[FMT_fmt++] = fmt;

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

  //-------------------------------
  //message type handling
  //-------------------------------
private:
  static uint32_t msg_name[LOG_TYPE_LEN];
  static uint8_t msg_name_len;

  uint8_t find_msg_type(const char *name) {
    uint8_t i;
    uint32_t findname = 0;
    strncpy((char*)&findname,name,4);
    for(i=0;i<msg_name_len;i++) {
      if(msg_name[i] == findname) {
        FMT_write = false;
        return i+0x81;
      }
    }
    if(i<LOG_TYPE_LEN) {
      msg_name[msg_name_len] = findname;
      msg_name_len++;
      FMT_write = true;
    }
    return i+0x81;
  }
  
  //-------------------------------
  //black box handling
  //-------------------------------
public:
  static bool started;
  
  static void start() {
    if(started) return;

    //force writing of FMT records
    msg_name_len = 0;

    //open bbfs (black box file system)
    if(!bbfs.writeOpen()) return; //bbfs emits error message

    //write headers
    FMT_sendFMT();
    _log_msg("ArduPlane"); //this sets the vehicle type -> which drives the translaton of flightmode codes to names (among other things probably)
    //_log_msg("ArduCopter");  //gives problems with plot.ardupilot.org
    _log_msg(MADFLIGHT_VERSION);
    _log_parm("DUMMY",0,0); //keep plot.ardupilot.org happy
    
    //allow logging
    locked = false;
    started = true;
  }
  
  static void stop() {
    //stop logging
    started = false;

    //close file
    bbfs.close();
  }

  //-------------------------------
  //standard messages
  //-------------------------------
private:
  //bypass locking
  static void _log_msg(const char* msg) {
      BinLog bl("MSG");
      bl.TimeUS(millis());
      bl.char64("Message",msg);
  }

  static void _log_parm(const char* name, float value, float default_value) {
      BinLog bl("PARM");  //PARM parameter value
      bl.TimeUS(millis()); //TimeUS: Time since system startup
      bl.char16("Name", name); //Name: parameter name
      bl.f32("Value", value); //Value: parameter value
      bl.f32("Default", default_value); //Default: default parameter value for this board and config
  }

public:
  //with locking
  static void log_msg(const char* msg) {
    if(!getLock()) return; //sets locked flag
    _log_msg(msg);
  }

  static void log_parm(const char* name, float value, float default_value) {
    if(!getLock()) return; //sets locked flag
    _log_parm(name, value, default_value);
  }
  
  //-------------------------------
  //message handling
  //-------------------------------
private:
  uint8_t buf[128];
  uint8_t buflen = 0;

public:
  
  BinLog(const char* name) {
    msg_begin(name);
  }
  
  ~BinLog() {
    msg_end();
  }
  
private:
  void msg_begin(const char* name) {
    buflen = 0;
    buf[buflen++] = HEAD_BYTE1;
    buf[buflen++] = HEAD_BYTE2;
    buf[buflen++] = find_msg_type(name); //this sets FMT_write when new message name is encountered
    if(FMT_write) {
      FMT = {}; //memset((void*)&FMT, 0, sizeof(FMT));
      FMT.h1 = HEAD_BYTE1;
      FMT.h2 = HEAD_BYTE2;
      FMT.type = 0x80;
      FMT.msg_type = buf[2];
      strncpy(FMT.name, name, 4);
      FMT_fmt = 0;
      FMT_lbl = 0;
    }
  }

  void msg_end() {
    if(FMT_write) {
      FMT.length = buflen;
      bbfs.write((uint8_t*)&FMT, sizeof(FMT));
      FMT_write = false;
    }
    bbfs.write(buf,buflen);
    buflen = 0;
    locked = false;
  }

public:
  void TimeUS(uint64_t v) {
    #ifdef LOG_TIMEUS_U32
      u32("TimeUS",v);
    #else
      u64("TimeUS",v);
    #endif
  }

  //add fields of specific type to message  
  void u8(const char* label, uint8_t v) {
    if(FMT_write) FMT_addField('B', label); // B   : uint8_t
    buf[buflen++] = v;
  }
  void u8flightmode(const char* label, uint8_t v) {
    if(FMT_write) FMT_addField('M', label); // M   : uint8_t flight mode
    buf[buflen++] = v;
  }  
  void u16(const char* label, uint16_t v) {
    if(FMT_write) FMT_addField('H', label); // H   : uint16_t
    *(uint16_t*)(buf+buflen) = v;
    buflen+=2;
  }
  void u16x100(const char* label, uint16_t v) { 
    if(FMT_write) FMT_addField('C', label); // C   : uint16_t * 100
    *(uint16_t*)(buf+buflen) = v;
    buflen+=2;
  }  
  void u32(const char* label, uint32_t v) {
    if(FMT_write) FMT_addField('I', label); // I   : uint32_t
    *(uint32_t*)(buf+buflen) = v;
    buflen+=4;
  }
  void u32x100(const char* label, uint32_t v) {
    if(FMT_write) FMT_addField('E', label); // E   : uint32_t * 100
    *(uint32_t*)(buf+buflen) = v;
    buflen+=4;
  }
  void u64(const char* label, uint64_t v) {
    if(FMT_write) FMT_addField('Q', label);  // Q   : uint64_t
    *(uint64_t*)(buf+buflen) = v;
    buflen+=8;
  }  
  void i8(const char* label, int8_t v) {
    if(FMT_write) FMT_addField('b', label); // b   : int8_t
    buf[buflen++] = v;
  }
  void i16(const char* label, int16_t v) {
    if(FMT_write) FMT_addField('h', label); // h   : int16_t
    *(int16_t*)(buf+buflen) = v;
    buflen+=2;
  }
  void i16x100(const char* label, int16_t v) {
    if(FMT_write) FMT_addField('c', label); // c   : int16_t * 100
    *(int16_t*)(buf+buflen) = v;
    buflen+=2;
  }
  void i32(const char* label, int32_t v) {
    if(FMT_write) FMT_addField('i', label); // i   : int32_t
    *(int32_t*)(buf+buflen) = v;
    buflen+=4;
  }
  void i32x100(const char* label, int32_t v) {
    if(FMT_write) FMT_addField('e', label); // e   : int32_t * 100
    *(int32_t*)(buf+buflen) = v;
    buflen+=4;
  }
  void i32latlon(const char* label, int32_t v) {
    if(FMT_write) FMT_addField('L', label); // L   : int32_t latitude/longitude
    *(int32_t*)(buf+buflen) = v;
    buflen+=4;
  }
  void i64(const char* label, int64_t v) {
    if(FMT_write) FMT_addField('q', label); // q   : int64_t
    *(int64_t*)(buf+buflen) = v;
    buflen+=8;
  }   
  void f32(const char* label, float v) {
    if(FMT_write) FMT_addField('f', label); // f   : float
    *(float*)(buf+buflen) = v;
    buflen+=4;
  }
  void f64(const char* label, double v) {
    if(FMT_write) FMT_addField('d', label); // d   : double
    *(double*)(buf+buflen) = v;
    buflen+=8;
  }
  void char4(const char* label, const char* v) {
    if(FMT_write) FMT_addField('n', label); // n   : char[4]
    strncpy((char*)(buf+buflen), v, 4);
    buflen+=4;
  }
  void char16(const char* label, const char* v) {
    if(FMT_write) FMT_addField('N', label);  // N   : char[16]
    strncpy((char*)(buf+buflen), v, 16);
    buflen+=16;
  }
  void char64(const char* label, const char* v) {
    if(FMT_write) FMT_addField('Z', label);  // Z   : char[64]
    strncpy((char*)(buf+buflen), v, 64);
    buflen+=64;
  }
  void blob64(const char* label, const int16_t* v) {
    if(FMT_write) FMT_addField('a', label);  // a   : int16_t[32]
    memcpy((void*)(buf+buflen), (void*)v, 64); 
    buflen+=64;
  }
};

//BinLog static var init
uint32_t BinLog::msg_name[LOG_TYPE_LEN] = {};
uint8_t BinLog::msg_name_len = 0;
bool BinLog::locked = false;
bool BinLog::started = false;


//black box public interface
class BlackBox {
  public:
    void setup() {
      bbfs.setup();
    }

    void start() {
      BinLog::start();
    }

    void stop() {
      BinLog::stop();
    }

    void erase() {
      stop();
      bbfs.erase();
    }

    void dir() {
      bbfs.dir();
    }

    void bench() {
      bbfs.bench();
    }
};

BlackBox bb;


//logging functions
void bb_log_msg(const char* msg) {
  BinLog::log_msg(msg);
}

void bb_log_parm(const char* name, float value, float default_value) {
  BinLog::log_parm(name, value, default_value);
}

void bb_log_imu() {
  if(!BinLog::getLock()) return; //prevent other tasks to write at the same time
  BinLog bl("IMU");
  bl.TimeUS(millis());
  bl.i16x100("ax",ahrs.ax*100); //G
  bl.i16x100("ay",ahrs.ay*100); //G
  bl.i16x100("az",ahrs.az*100); //G
  bl.i16("gx",ahrs.gx*10); //dps
  bl.i16("gy",ahrs.gy*10); //dps
  bl.i16("gz",ahrs.gz*10); //dps
  bl.i16x100("mx",ahrs.mx*100); //uT
  bl.i16x100("my",ahrs.my*100); //uT
  bl.i16x100("mz",ahrs.mz*100); //uT
  bl.i16("roll",ahrs.roll*10); //deg
  bl.i16("pitch",ahrs.pitch*10);; //deg
  bl.i16("yaw",ahrs.yaw*10);; //deg
}

//TODO - set all fields
void bb_log_gps() {
    if(!BinLog::getLock()) return; //prevent other tasks to write at the same time
    // Information received from GNSS systems attached to the autopilot
    BinLog bl("GPS");
    bl.TimeUS(millis()); //TimeUS: Time since system startup [us]
    bl.u8("I",0); //I: GPS instance number
    bl.u8("Status",3); //Status: GPS Fix type; 2D fix, 3D fix etc.
    bl.u32("GMS",123); //GMS: milliseconds since start of GPS Week
    bl.u16("GWk",23452); //GWk: weeks since 5 Jan 1980
    bl.u8("NSats",12); //NSats: number of satellites visible
    bl.i16x100("HDop",120); //HDop: horizontal dilution of precision
    bl.i32latlon("Lat",gps.lat); //Lat: latitude [deg*10e7]
    bl.i32latlon("Lng",gps.lon); //Lng: longitude [deg*10e7]
    bl.i32x100("Alt",50); //Alt: altitude
    bl.f32("Spd",10); //Spd: ground speed
    bl.f32("GCrs",23); //GCrs: ground course
    bl.f32("VZ",0); //VZ: vertical speed
    bl.f32("Yaw",12); //Yaw: vehicle yaw
    bl.u8("U",1); //U: boolean value indicating whether this GPS is in use
}

void bb_log_mode(uint8_t fm, const char* name) {
    if(!BinLog::getLock()) return; //prevent other tasks to write at the same time
    BinLog bl("MODE");
    bl.TimeUS(millis());
    bl.u8flightmode("Mode",fm);
    bl.u8("ModeNum",fm);
    bl.u8("Rsn",1); //ModeReason
    bl.char16("Name",name); //extenstion to "standard" ArduPilot BinLog
}

/*
void bb_log_att() {
    if(!BinLog::getLock()) return; //prevent other tasks to write at the same time
    BinLog bl("ATT");
    bl.TimeUS(millis());
    bl.i16x100("DesRoll",0);
    bl.i16x100("Roll",t);
    bl.i16x100("DesPitch",0);
    bl.i16x100("Pitch",-t);
    bl.u16x100("DesYaw",0);
    bl.u16x100("Yaw",t);
    bl.u16x100("ErrRP",1);
    bl.u16x100("ErrYaw",2);
    bl.u8 ("AEKF",3);
}
*/

void bb_log_bat(){}  //TODO battery
void bb_log_baro(){} //TODO barometer