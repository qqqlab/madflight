/*========================================================================================================================
This file contains all necessary functions and code used for blackbox logging to avoid cluttering the main code

Each USE_BB_xxx section in this file defines:
 - bb.setup() - setup interface
 - bb.csvDump() - dump last log to serial debug port as csv (tab delimited text)
 - bb.erase() - erase files
 - bb.dir() - list files

In addition there are these functions common for all blackbox variants:
 - bb.start() - start a logging session
 - bb.stop() - stop a logging session
 - bb.log_xxx() - call these functions to write to logger, modify/add functions as needed

HOW TO ADD YOUR OWN LOGGERS
 1. add a recordtype_e enum
 2. add a new BlackBoxBase::log_xxx() for your logger. Use writeU() and writeI() to write variable byte encoded integer (these take 1-5 bytes of space)
 3. add a call to your logger in BlackBoxBase::start() to write the header
 4. call bb.log_xxx() in madflight.ino to log data

Note: this header needs to be included after all globals are defined in madflight.ino, so that these variables are visible here without declaring each global variable extern in this file.
========================================================================================================================*/

#define BB_USE_NONE 1
#define BB_USE_FLASH 2
#define BB_USE_RAM 3

#include "BlackBox_Defines.h"
#include "BlackBoxWriter.h"
#include "BlackBoxDecoder.h"

class BlackBox {

private:

  enum recordtype_e {
    BB_REC_IMU,
    BB_REC_PID,
    BB_REC_BAT,
    BB_REC_BARO,
    BB_REC_GPS,
  };

public:

  void log_imu() {
    if(bbw.isBusy()) return; //sets busy flag
    bbw.writeBeginRecord(BB_REC_IMU, "IMU");
    bbw.writeUnsignedVB("ts",micros());
    bbw.writeSignedVB("ax*100",AccX*100); //G
    bbw.writeSignedVB("ay*100",AccY*100); //G
    bbw.writeSignedVB("az*100",AccZ*100); //G
    bbw.writeSignedVB("gx*10",GyroX*10); //dps
    bbw.writeSignedVB("gy*10",GyroY*10); //dps
    bbw.writeSignedVB("gz*10",GyroZ*10); //dps
    bbw.writeSignedVB("mx*100",MagX*100); //uT
    bbw.writeSignedVB("my*100",MagX*100); //uT
    bbw.writeSignedVB("mz*100",MagX*100); //uT  
    bbw.writeSignedVB("roll*10",ahrs_roll*10); //deg
    bbw.writeSignedVB("pitch*10",ahrs_pitch*10);; //deg
    bbw.writeSignedVB("yaw*10",ahrs_yaw*10);; //deg
    bbw.writeEndrecord(); //clears busy flag
  }

  void log_pid() {
    if(bbw.isBusy()) return;
    bbw.writeBeginRecord(BB_REC_PID, "PID");
    bbw.writeUnsignedVB("ts",micros());
    bbw.writeSignedVB("roll_PID*100",roll_PID*100); //-1 to +1
    bbw.writeSignedVB("pitch_PID*100",pitch_PID*100);; //-1 to +1
    bbw.writeSignedVB("yaw_PID*100",yaw_PID*100);; //-1 to +1
    bbw.writeEndrecord();
  }

  void log_bat() {
    if(bbw.isBusy()) return;
    bbw.writeBeginRecord(BB_REC_BAT, "BAT");
    bbw.writeUnsignedVB("ts",micros());
    bbw.writeUnsignedVB("bat_mA",bat.i*1000); //Battery current (A)
    bbw.writeUnsignedVB("bat_mV",bat.v*1000); //battery voltage (V)
    bbw.writeUnsignedVB("bat_mAh",bat.mah); //battery usage (Ah)
    bbw.writeUnsignedVB("bat_mWh",bat.wh*1000); //battery usage (Wh)
    bbw.writeEndrecord();
  }

  void log_baro() {
    if(bbw.isBusy()) return;
    bbw.writeBeginRecord(BB_REC_BARO, "BARO");
    bbw.writeUnsignedVB("ts",micros());
    bbw.writeUnsignedVB("baro_pa",baro.press_pa); //Barometer pressure (Pa)
    bbw.writeSignedVB("baro_t*100",baro.temp_c*100); //barometer temp (C)
    bbw.writeEndrecord();
  }

  void log_gps() {
    if(bbw.isBusy()) return;
    bbw.writeBeginRecord(BB_REC_GPS, "GPS");
    bbw.writeUnsignedVB("ts",micros()); 
    bbw.writeI32("lat",gps.lat);
    bbw.writeI32("lon",gps.lon);
    bbw.writeEndrecord();
  }

  void start() {
    if(isStarted) return;
    isStarted = true;
    //create a new file
    fs->writeOpen();
    bbw.writeHeaders();
    //add here all loggers, this will write headers with record and field info which are needed by the decoder
    log_imu();
    log_pid();
    log_bat();
    log_gps();
    //start logging
    bbw.startLogging();
  }

  void stop() {
    if(!isStarted) return;
    isStarted = false;
    bbw.stopLogging();
    fs->writeClose();
  }

  void setup() {
    isStarted = false;
    fs->setup();
    bbw.setup(fs);
  }

  void csvDump(int fileno) {
    fs->readOpen(fileno);
    BlackBoxDecoder bbd;
    bbd.csv_decode(fs);
  }

  void erase() {
    stop();
    fs->erase();
  }

  void dir() {
    fs->dir();
  }



  BlackBox(BlackBoxFS *fs) {
    this->fs = fs;
  }

private:

  BlackBoxFS *fs;
  BlackBoxWriter bbw;

  bool isStarted = false;
};

//=====================================================================================================================
// Logging to FLASH (internal or external)
//=====================================================================================================================
#if BB_USE == BB_USE_FLASH

#include "SPIFlash/Adafruit_SPIFlashBase.h"

//TODO
// Un-comment to run example with custom SPI and SS e.g with FRAM breakout
// #define CUSTOM_CS   A5
// #define CUSTOM_SPI  SPI

#if defined(CUSTOM_CS) && defined(CUSTOM_SPI)
  Adafruit_FlashTransport_SPI flashTransport(CUSTOM_CS, CUSTOM_SPI);
#elif defined(ARDUINO_ARCH_ESP32)
  // ESP32 use same flash device that store code for file system.
  // SPIFlash will parse partition.cvs to detect FATFS/SPIFS partition to use
  Adafruit_FlashTransport_ESP32 flashTransport;
#elif defined(ARDUINO_ARCH_RP2040)
  // RP2040 use same flash device that store code for file system. Therefore we
  // only need to specify start address and size (no need SPI or SS)
  // By default (start=0, size=0), values that match file system setting in
  // 'Tools->Flash Size' menu selection will be used.
  Adafruit_FlashTransport_RP2040 flashTransport;
#else
  #error No (Q)SPI flash defined for your board !
#endif

Adafruit_SPIFlashBase flash(&flashTransport);


class BlackBoxFS_Flash : public BlackBoxFS {
private:
  uint32_t flash_write_addr;
  uint8_t buf_write[256];
  int buf_write_idx;

  uint32_t flash_read_addr;
  uint8_t buf_read[256];
  int buf_read_idx;

public:
  void setup() {
    flash.begin();
    flash_write_addr = findStartPage();
    Serial.printf("BB_USE_FLASH  JEDEC=%X size=%d free=%d\n", flash.getJEDECID(), flash.size(), flash.size()-flash_write_addr);
  }

  virtual void writeOpen() {
    memset(buf_write, 0xff, 256);
    buf_write_idx = 0;
    flash_write_addr = findStartPage();
  }

  virtual void writeChar(uint8_t c) {
    if(buf_write_idx < 256) {
     buf_write[buf_write_idx++] = c;
    }
    if(buf_write_idx >= 256) {
      flash.writeBuffer(flash_write_addr, buf_write, 256);
      flash_write_addr += 256;
      memset(buf_write, 0xff, 256);
      buf_write_idx = 0;
    }
  }

  virtual void writeClose() {
    if(buf_write_idx==0) return;
    //write 0xff in unused part of page
    memset(buf_write + buf_write_idx, 0xff, 256 - buf_write_idx);
    //write final page
    flash.writeBuffer(flash_write_addr, buf_write, 256);
    flash_write_addr += 256;
  }


  virtual void readOpen(int fileno) {
    int adr[100];
    int filecnt = findFiles(adr,100);
    if(fileno <= 0) {
      fileno = filecnt - 1 + fileno; //0:last file, -1:2nd last file, etc
    }else{
      fileno--;
    }
    if(fileno<0 || fileno>=filecnt) fileno = filecnt;
    readSeek( adr[fileno] );
  }

  virtual uint8_t readChar() {
    if(buf_read_idx >= 256) {
      flash.readBuffer(flash_read_addr, buf_read, 256);
      flash_read_addr += 256;
      buf_read_idx = 0;
    }
    return buf_read[buf_read_idx++];
  }


  void erase() {
    Serial.println("Erasing flash, might take a while...");
    flash.eraseChip();
    flash.waitUntilReady();
    Serial.println("Erasing flash completed");
  }

  void dir() {
    int adr[100];
    int filecnt = findFiles(adr,100);
    for(int i=0;i<filecnt;i++) {
      Serial.printf("file %d: size=%d\n", i+1, adr[i+1]-adr[i]);
    }
    Serial.printf("Total %d files, %d bytes\n", filecnt, adr[filecnt]);
  }

private:

  //find file start addresses, returns number of files, adr contains one more element with start of next file
  int findFiles(int *adr, int len) {
    int filecnt = 0;
    int size = flash_write_addr;
    int j=0;
    readSeek(0);
    for(int i=0; i<size; i++) {
      uint8_t c = readChar();
      if(c == BB_STARTLOG[j]) {
        j++;
        if(j==16) {
          if(!(filecnt<len-1)) break;
          adr[filecnt++] = i-15;
          j=0;
        }
      }
    }
    adr[filecnt] = size;
    return filecnt;
  }

  //find first empty page
  uint32_t findStartPage() {
    const uint32_t readlen = 256;
    const uint32_t pagelen = 256;
    uint32_t len = flash.size()/pagelen;
    uint32_t page_max = len - 1;
    uint32_t page_min = 0;
    uint8_t buf[readlen];
    while(page_max != page_min + 1) {
      uint32_t page = page_min + (page_max - page_min) / 2;
      flash.readBuffer(page*pagelen, buf, readlen);
      //Serial.printf("page_min=%8d page_max=%8d page=%8d ", page_min, page_max, page);
      //for(int i=0;i<16;i++) Serial.printf(" %02X",buf[i]);
      //Serial.println();
      for(int i=0;i<readlen;i++) if(buf[i]!=0xff) {
        page_min = page;
        break;
      }
      if(page_min != page) page_max = page;
    }
    return page_max * pagelen;
  }


  void readSeek(uint32_t adr) {
    buf_read_idx = 256; //force read flash on next callback_bbReadChar()
    flash_read_addr = (adr/256)*256; //beginning of page
    readChar(); //read page
    buf_read_idx = adr % 256; //next call to callback_bbReadChar() will read from address adr
  }

};

BlackBoxFS_Flash bb_fs;



/*
//=====================================================================================================================
// Logging to SPI FLASH
//=====================================================================================================================
#if BB_USE == BB_USE_FLASH

//#define HW_PIN_FLASH_CS    PB3
//extern SPIClass bb_spi;

#define BB_BUF_SIZE 64 //write this many bytes per write cycle - needs to be a power of 2, max 256

class BlackBox: public BlackBoxBase {
public:

  void setup() {
    w25qxx_size = 0;
    w25qxx_adr = 0xffffffff;
    buf_len = 0;
    w25qxx.setSPIPort(bb_spi);
    w25qxx.begin(HW_PIN_BB_CS,18000000);  
    bbw.begin(callback_bbWriteChar);
    findStart();
    Serial.printf("BB_USE_FLASH size=%d start=%d\n", (int)w25qxx_size, (int)w25qxx_adr);
  }

  void csvDump() {
    stop();
    buf_idx = BB_BUF_SIZE;
    w25qxx_readadr = w25qxx_startadr;
    BlackBoxDecoder bbd;
    bbd.csv_decode(callback_bbReadChar, callback_SerialPrintChar);  
  }

  void erase() {
    Serial.println("Full flash erase started, this will take a while to complete...");
    w25qxx.eraseAll(true);
    Serial.println("Full flash erase completed.");
  }

private:

  void findStart() {
    w25qxx_size = w25qxx.readSize();
    w25qxx_adr = w25qxx_size>>1;
    uint32_t stepsize = w25qxx_adr>>1;
    uint8_t buf[16];
    while(stepsize>=16) {
      w25qxx.read(w25qxx_adr, buf, 16);
      int32_t step = -stepsize;
      for(int i=0;i<16;i++) if(buf[i]!=0xff) {
        step = +stepsize;
        break;
      }
      w25qxx_adr += step;
      stepsize >>= 1;
    }
    w25qxx_adr = ((w25qxx_adr / BB_BUF_SIZE) + 1) * BB_BUF_SIZE; //move to next BB_BUF_SIZE boundary
    w25qxx_startadr = w25qxx_adr;
  }

  //static callback
  #include "W25Qxx/W25Qxx.h"
  static W25Qxx w25qxx;
  static uint8_t buf[BB_BUF_SIZE];
  static uint32_t w25qxx_size;
  static uint32_t w25qxx_adr;
  static uint32_t w25qxx_startadr;
  static int buf_len;
  static int buf_idx;
  static uint32_t w25qxx_readadr;

  static uint8_t callback_bbReadChar() {
    if(buf_idx >= BB_BUF_SIZE) {
      w25qxx.read(w25qxx_readadr, buf, BB_BUF_SIZE);
      w25qxx_readadr += BB_BUF_SIZE;
      buf_idx = 0;
    }
    return buf[buf_idx++];
  }

  static void callback_bbWriteChar(uint8_t c) {
    if(buf_len < BB_BUF_SIZE) {
      buf[buf_len++] = c;
    }else{
      w25qxx.pageWrite(w25qxx_adr, buf, BB_BUF_SIZE, false); //non-blocking write
      w25qxx_adr += BB_BUF_SIZE;
      buf_len = 0;
    }
  }
};

BlackBox::W25Qxx BlackBox::w25qxx;
uint8_t BlackBox::buf[BB_BUF_SIZE];
uint32_t BlackBox::w25qxx_size;
uint32_t BlackBox::w25qxx_adr;
uint32_t BlackBox::w25qxx_startadr;
int BlackBox::buf_len;
int BlackBox::buf_idx;
uint32_t BlackBox::w25qxx_readadr;
*/

//=====================================================================================================================
// Logging to RAM Memory
//=====================================================================================================================
#elif BB_USE == BB_USE_RAM

#ifndef BB_RAM_BUF_SIZE
  #define BB_RAM_BUF_SIZE 60000
#endif

class BlackBoxFS_RAM : public BlackBoxFS {
private:

  uint8_t *buf;
  int buf_size; //buffer size
  int buf_w; //number of bytes written to buffer
  int buf_r; //read pointer

public:

  //setup the file system
  void setup() {
    mem_setup();
    buf_w = 0;
    Serial.printf("BB_USE_RAM size=%d\n", buf_size);
  }

  //create new file for writing
  void writeOpen() {}

  //write char to file
  void writeChar(uint8_t c) {
    if(buf_w < buf_size) {
      buf[buf_w++] = c;
    }
  }

  //close file
  void writeClose() {};

  //open fileno for reading. Fileno 0:last file, -1:2nd last file, etc
  void readOpen(int fileno) {
    int adr[100];
    int filecnt = findFiles(adr,100);
    if(fileno == 0) {
      fileno = filecnt - 1 + fileno; //0:last file, -1:2nd last file, etc
    }else{
      fileno--;
    }
    if(fileno<0 || fileno>=filecnt) fileno = filecnt;
    buf_r = adr[fileno];
  }

  //read char from file
  uint8_t readChar() {
    if(buf_r < buf_w) {
      return buf[buf_r++];
    }else{
      return 0xff;
    }
  }

  //erase all files
  void erase() {
    setup();
  }

  //list files
  void dir() {
    int adr[100];
    int filecnt = findFiles(adr,100);
    for(int i=0;i<filecnt;i++) {
      Serial.printf("file %d: size=%d\n", i+1, adr[i+1]-adr[i]);
    }
    Serial.printf("Total %d files, %d bytes\n", filecnt, adr[filecnt]);
  }

private:

  //find file start addresses, returns number of files, adr contains one more element with start of next file
  int findFiles(int *adr, int len) {
    int filecnt = 0;
    int size = buf_w;
    int j=0;
    buf_r = 0;
    for(int i=0; i<size; i++) {
      uint8_t c = readChar();
      if(c == BB_STARTLOG[j]) {
        j++;
        if(j==16) {
          if(!(filecnt<len-1)) break;
          adr[filecnt++] = i-15;
          j=0;
        }
      }
    }
    adr[filecnt] = size;
    return filecnt;
  }

  void mem_setup() {
    if(buf) return;
    buf_size = 0;
    #if defined ARDUINO_ARCH_ESP32
      //alloc psram if available
      uint32_t psramsize = ESP.getFreePsram();
      if(psramsize) {
        buf = (uint8_t*)ps_malloc(psramsize);
        if(buf) buf_size = psramsize;
      }
    #endif
    if(!buf) {
      buf = (uint8_t*)malloc(BB_RAM_BUF_SIZE);
      if(buf) buf_size = BB_RAM_BUF_SIZE;
    }
    if(!buf) {
      Serial.println("BB_USE_RAM memory allocation failed.");
    }
  }

};

BlackBoxFS_RAM bb_fs;

//=====================================================================================================================
// None or undefined
//=====================================================================================================================
#elif BB_USE == BB_USE_NONE || !defined BB_USE
class BlackBoxFS_None : public BlackBoxFS {
public:
  void setup() {}
  
  void writeOpen() {}
  void writeChar(uint8_t c) {}
  void writeClose() {}

  void readOpen(int fileno) {}
  uint8_t readChar() {return 0;}

  void erase() {}
  void dir() {}
};

BlackBoxFS_None bb_fs;

//=====================================================================================================================
// Invalid value
//=====================================================================================================================
#else
  #error "invalid BB_USE value"
#endif

BlackBox bb(&bb_fs);