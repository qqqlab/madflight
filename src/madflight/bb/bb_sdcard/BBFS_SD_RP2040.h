/*==========================================================================================
BBFS_SD_RP2040.h: madflight sdcard spi logging file system

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


#include "SPI.h"
#include "SD.h"

extern SPIClassRP2040 *bb_spi;

class BBFS_SD : public BBFS {
private:
  const char* BB_LOG_DIR_NAME = "/log";
  bool setup_done = false;
  uint8_t wbuf[512];
  uint32_t wbuf_len = 0;
  String filename;
  File file = {};

public:
  //setup the file system
  void setup() override {
    Serial.println("BB: BB_USE_SD");
    sd_setup();
  }

  bool writeOpen() override {
    close();

    int attempt = 0;
    while(++attempt<3) {
      if(sd_setup()) {
        memset(wbuf, 0xff, sizeof(wbuf));
        wbuf_len = 0;

        sd_logOpen(BB_LOG_DIR_NAME);
        if(file) {
          Serial.printf("BB: start OK - attempt=%d file=%s\n", attempt, filename.c_str());
          return true;
        }
      }

      Serial.println("BB: start retry");
      setup_done = false; //force setup to re-run
    }
    Serial.println("BB: start FAILED");
    return false;
  }

private:
  void _writeChar(uint8_t c) {
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

public:
  void write(const uint8_t *buf, const uint8_t len) override {
    for(int i=0;i<len;i++) {
      _writeChar(buf[i]);
    }
  }

  void close() override {
    if(file) {
      if(wbuf_len>0) {
        file.write(wbuf, wbuf_len);
        wbuf_len = 0;
      }
      file.flush();
      int len = file.size();
      file.close();
      file = {};
      Serial.printf("BB: stop file=%s len=%d\n", filename.c_str(), len);
    }
  }

  void erase() override {
    if(sd_setup()) {
      sd_deleteFilesFromDir(BB_LOG_DIR_NAME);
    }
  }

  void dir() override {
    int attempt = 0;
    while(++attempt<2) {
      if(sd_setup()) {
        if(sd_listDir(BB_LOG_DIR_NAME)) return;
      }
      setup_done = false; //force setup to re-run
    }
  }

  void bench() override {
      const char* path = "madfli.ght";
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

      if(SD.exists(path)) {
        SD.remove(path);
      }

      file = SD.open(path, FILE_WRITE);
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
      
      file = SD.open(path);
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

  void info() override {
    // 0 - SD V1, 1 - SD V2, or 3 - SDHC/SDXC
    // print the type of card
    Serial.println();
    Serial.print("Card type:         ");
    switch (SD.type()) {
      case 0:
        Serial.println("SD1");
        break;
      case 1:
        Serial.println("SD2");
        break;
      case 3:
        Serial.println("SDHC/SDXC");
        break;
      default:
        Serial.println("Unknown");
    }

    Serial.print("Cluster size:      ");
    Serial.println(SD.clusterSize());
    Serial.print("Blocks x Cluster:  ");
    Serial.println(SD.blocksPerCluster());
    Serial.print("Blocks size:       ");
    Serial.println(SD.blockSize());

    Serial.print("Total Blocks:      ");
    Serial.println(SD.totalBlocks());

    Serial.print("Total Cluster:     ");
    Serial.println(SD.totalClusters());

    // print the type and size of the first FAT-type volume
    uint32_t volumesize;
    Serial.print("Volume type is:    FAT");
    Serial.println(SD.fatType(), DEC);

    volumesize = SD.totalClusters();
    volumesize *= SD.clusterSize();
    Serial.print("Volume size (b):   ");
    Serial.println(volumesize);
    volumesize /= 1024;
    Serial.print("Volume size (Kb):  ");
    Serial.println(volumesize);
    Serial.print("Volume size (Mb):  ");
    volumesize /= 1024;
    Serial.println(volumesize);
    Serial.print("Volume size (Gb):  ");
    Serial.println((float)volumesize / 1024.0);

    Serial.print("Card size:         ");
    Serial.println(SD.size());

    FSInfo fs_info;
    SDFS.info(fs_info);

    Serial.print("Total bytes:       ");
    Serial.println(fs_info.totalBytes);

    Serial.print("Used bytes:        ");
    Serial.println(fs_info.usedBytes);

    sd_listDir("/");
  }

private:

  bool sd_setup() {
    if(setup_done) return true;
    
    SD.end(); //force begin() to re-initialize SD

/*
    // Ensure the SPI pinout the SD card is connected to is configured properly
    // Select the correct SPI based on _MISO pin for the RP2040
    bool sdInitialized = false;
    if (HW_PIN_SD_SPI_MISO == 0 || HW_PIN_SD_SPI_MISO == 4 || HW_PIN_SD_SPI_MISO == 16) {
      SPI.setRX(HW_PIN_SD_SPI_MISO);
      SPI.setTX(HW_PIN_SD_SPI_MOSI);
      SPI.setSCK(HW_PIN_SD_SPI_SCLK);
      sdInitialized = SD.begin(HW_PIN_SD_SPI_CS, SPI);
    } else if (HW_PIN_SD_SPI_MISO == 8 || HW_PIN_SD_SPI_MISO == 12) {
      SPI1.setRX(HW_PIN_SD_SPI_MISO);
      SPI1.setTX(HW_PIN_SD_SPI_MOSI);
      SPI1.setSCK(HW_PIN_SD_SPI_SCLK);
      sdInitialized = SD.begin(HW_PIN_SD_SPI_CS, SPI1);
    } else {
      Serial.println(F("BB: ERROR Unknown SPI Configuration"));
      setup_done = false;
      return setup_done;
    }

    if (!sdInitialized) {
      Serial.println("BB: Card Mount Failed");
      setup_done = false;
      return setup_done;
    }
*/

    if (!SD.begin(HW_PIN_BB_CS, *bb_spi)) {
      Serial.println("BB: Card Mount Failed");
      setup_done = false;
      return setup_done;
    }

    setup_done = true;
    return setup_done;
  }

  bool sd_listDir(const char * dirname, uint8_t levels=0){
      Serial.printf("Listing directory: %s\n", dirname);

      File root = SD.open(dirname);
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
              Serial.printf("%-18s <DIR>\n",file.name());
              if(levels){
                  sd_listDir(file.fullName(), levels -1);
              }
          } else {
              Serial.printf("%-14s %9d\n",file.name(),file.size());
          }
          file = root.openNextFile();
      }
      return true;
  }

  void sd_deleteFilesFromDir(const char * dirname){
      File root = SD.open(dirname);
      if(!root) return;
      if(!root.isDirectory()) return;

      File file = root.openNextFile();
      while(file){
          if(!file.isDirectory()){
            SD.remove(file.fullName());
          }
          file = root.openNextFile();
      }
  }

  void sd_logOpen(const char * dirname){
      File root = SD.open(dirname);
      if(!root){
         if(!SD.mkdir(dirname)){
            Serial.println("BB: mkdir /log failed");
            return;
        }
        root = SD.open(dirname);
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
      file = SD.open(filename, FILE_WRITE);
  }
};