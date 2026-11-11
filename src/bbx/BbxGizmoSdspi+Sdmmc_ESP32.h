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

#include "bbx.h"

#ifdef BBX_USE_MMC

  #include "SD_MMC.h"
  
  #define _BB_SDFS SD_MMC
  #define _BB_SETUP_STR "BBX: SDMMC"

class BbxGizmoSdmmc : public BbxGizmo {
  
public:
  BbxGizmoSdmmc(BbxConfig *config) {
    this->config = config;
  }

#else

  #include "SPI.h"
  #include "SD.h"
  
  #define _BB_SDFS SD
  #define _BB_SETUP_STR "BBX: SDSPI"

class BbxGizmoSdspi : public BbxGizmo {
public:
  BbxGizmoSdspi(BbxConfig *config) {
    this->config = config;
  }

#endif


private:
  BbxConfig *config;
  const char* BB_LOG_DIR_NAME = "/log";
  bool setup_done = false;
  uint8_t wbuf[512];
  int wbuf_len = 0;
  String filename;
  File file;

public:
  //setup the file system
  void setup() override {
    MF_printf(_BB_SETUP_STR);
    MF_printf("\n");
    sd_setup();
  }

  bool writeOpen() override {
    close();

    int attempt = 0;
    while(++attempt<2) {
      if(sd_setup()) {
        memset(wbuf, 0xff, sizeof(wbuf));
        wbuf_len = 0;

        sd_logOpen(BB_LOG_DIR_NAME);
        if(file) {
          MF_printf("BBX:   start OK - attempt=%d file=%s\n", attempt, filename.c_str());
          return true;
        }
      }

      setup_done = false; //force setup to re-run
    }
    MF_printf("BBX:   start FAILED\n");
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
    for(int i=0;i<len;i++) _writeChar(buf[i]);
  }

  void close() override {
    if(wbuf_len>0) {
      file.write(wbuf, wbuf_len);
      wbuf_len = 0;
    }
    file.close();
  }

  void erase() override {
    if(sd_setup()) {
      sd_deleteFilesFromDir(BB_LOG_DIR_NAME);
    }
  }

  void dir () override {
    int attempt = 0;
    while(++attempt<2) {
      if(sd_setup()) {
        if(sd_listDir(BB_LOG_DIR_NAME, 0)) return;
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
        MF_printf("BBX:   bench - malloc failed\n");
        return;
    }
    
    file = _BB_SDFS.open(path, FILE_WRITE);
    if(!file){
        MF_printf("BBX:   bench - Failed to open file for writing\n");
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
    MF_printf("BBX:   %s %u bytes written in %u ms %f kbps\r\n", path, (int)flen, (int)end, (float)flen/end);
    file.close();
    
    file = _BB_SDFS.open(path);
    if(!file){
        MF_printf("BBX:   bench - Failed to open file for reading\n");
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
    MF_printf("BBX:   %s %u bytes read in %u ms %f kbps\r\n", path, (int)flen, (int)end, (float)flen/end);
    file.close();

    free(buf);
  }

  void info() override {
    uint8_t cardType = _BB_SDFS.cardType();
    if(cardType == CARD_NONE){
      MF_printf("No SD card attached\n");
      return;
    }
    MF_printf("CardType: ");
    if(cardType == CARD_MMC){
        MF_printf("MMC");
    } else if(cardType == CARD_SD){
        MF_printf("SD");
    } else if(cardType == CARD_SDHC){
        MF_printf("SDHC");
    } else {
        MF_printf("UNKNOWN");
    }
    MF_printf("\n");
    //MF_printf("SectorSize: %d\n", _BB_SDFS.sectorSize()); //'class fs::SDMMCFS' has no member named 'sectorSize'
    MF_printf("CardSize: %lluMB\n", _BB_SDFS.cardSize() / (1024 * 1024));
    MF_printf("used: %lluMB\n", _BB_SDFS.usedBytes()/ (1024 * 1024));
    MF_printf("free: %lluMB\n", (_BB_SDFS.totalBytes() - _BB_SDFS.usedBytes()) / (1024 * 1024));
    MF_printf("\n");
  }

  int read(const char* filename, uint8_t **data) override {
    if(!sd_setup()) return 0;
    if(!_BB_SDFS.exists(filename)) return 0;
    File file = _BB_SDFS.open(filename, FILE_READ);
    int len = file.size();
    if(len == 0) return 0;
    uint8_t* buf = (uint8_t*)malloc(len);
    if(!buf) return 0;
    file.read(buf, len);
    file.close();
    *data = buf;
    return len;
  }

private:

  bool sd_setup() {
    if(setup_done) return true;

    _BB_SDFS.end(); //force begin() to re-initialize
 
    #ifdef BBX_USE_MMC
      if (config->pin_mmc_clk < 0 || config->pin_mmc_cmd < 0 || config->pin_mmc_dat < 0) {
        MF_printf("BBX: SDMMC Setup Failed, missing clk/cmd/dat pins\n");
        return false;
      }
      _BB_SDFS.setPins(config->pin_mmc_clk, config->pin_mmc_cmd, config->pin_mmc_dat);
      if (!_BB_SDFS.begin("/sdcard", true, true, SDMMC_FREQ_DEFAULT, 5)) {
        MF_printf("BBX: SDMMC Card Mount Failed\n");
        return false;
      }
    #else
      if (config->spi_cs <= 0 || !config->spi_bus) {
        MF_printf("BBX: SDSPI Setup Failed, missing cs pin or missing SPI bus\n");
        return false;
      }
      if (!_BB_SDFS.begin(config->spi_cs, *config->spi_bus)) {
        MF_printf("BBX: SDSPI Card Mount Failed\n");
        return false;
      }
    #endif

    uint8_t cardType = _BB_SDFS.cardType();
    if(cardType == CARD_NONE){
      MF_printf("BBX:   No SD card attached\n");
      return false;
    }

    setup_done = true;
    return true;
  }

  bool sd_listDir(const char * dirname, uint8_t levels = 0){
    MF_printf("Listing directory: %s\n", dirname);

    File root = _BB_SDFS.open(dirname);
    if(!root){
        MF_printf("Failed to open directory\n");
        return false;
    }
    if(!root.isDirectory()){
        MF_printf("Not a directory\n");
        return true;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            MF_printf("%-18s <DIR>\n",file.name());
            if(levels){
                sd_listDir(file.path(), levels -1);
            }
        } else {
            MF_printf("%-14s %9d\n",file.name(),file.size());
        }
        file = root.openNextFile();
    }
    return true;
  }

  void sd_deleteFilesFromDir(const char * dirname){
    File root = _BB_SDFS.open(dirname);
    if(!root) return;
    if(!root.isDirectory()) return;

    File file = root.openNextFile();
    while(file){
        if(!file.isDirectory()){
          _BB_SDFS.remove(file.path());
        }
        file = root.openNextFile();
    }
  }

  void sd_logOpen(const char * dirname){
    File root = _BB_SDFS.open(dirname);
    if(!root){
       if(!_BB_SDFS.mkdir(dirname)){
          MF_printf("BBX:   mkdir /log failed\n");
          return;
      }
      root = _BB_SDFS.open(dirname);
      if(!root) {
          MF_printf("BBX:   mkdir /log failed\n");
          return;
      }
    }
    if(!root.isDirectory()){
        MF_printf("BBX:   /log is not a directory\n");
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
    MF_printf("%s\n", filename);
    file = _BB_SDFS.open(filename, FILE_WRITE, true);
  }
};


#undef _BB_SDFS
#undef _BB_SETUP_STR
