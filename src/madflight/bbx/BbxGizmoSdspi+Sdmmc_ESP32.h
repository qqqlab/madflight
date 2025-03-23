
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
    Serial.println(_BB_SETUP_STR);
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
          Serial.printf("BBX:   start OK - attempt=%d file=%s\n", attempt, filename.c_str());
          return true;
        }
      }

      setup_done = false; //force setup to re-run
    }
    Serial.println("BBX:   start FAILED");
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
        Serial.println("BBX:   bench - malloc failed");
        return;
    }
    
    file = _BB_SDFS.open(path, FILE_WRITE);
    if(!file){
        Serial.println("BBX:   bench - Failed to open file for writing");
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
    Serial.printf("BBX:   %s %u bytes written in %u ms %f kbps\r\n", path, (int)flen, (int)end, (float)flen/end);
    file.close();
    
    file = _BB_SDFS.open(path);
    if(!file){
        Serial.println("BBX:   bench - Failed to open file for reading");
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
    Serial.printf("BBX:   %s %u bytes read in %u ms %f kbps\r\n", path, (int)flen, (int)end, (float)flen/end);
    file.close();

    free(buf);
  }

  void info() override {
    uint8_t cardType = _BB_SDFS.cardType();
    if(cardType == CARD_NONE){
      Serial.println("No SD card attached");
      return;
    }
    Serial.print("CardType: ");
    if(cardType == CARD_MMC){
        Serial.print("MMC");
    } else if(cardType == CARD_SD){
        Serial.print("SD");
    } else if(cardType == CARD_SDHC){
        Serial.print("SDHC");
    } else {
        Serial.print("UNKNOWN");
    }
    Serial.println();
    Serial.printf("SectorSize: %d\n", _BB_SDFS.sectorSize());
    Serial.printf("CardSize: %lluMB\n", _BB_SDFS.cardSize() / (1024 * 1024));
    Serial.printf("used: %lluMB\n", _BB_SDFS.usedBytes()/ (1024 * 1024));
    Serial.printf("free: %lluMB\n", (_BB_SDFS.totalBytes() - _BB_SDFS.usedBytes()) / (1024 * 1024));
    Serial.println();
  }

private:

  bool sd_setup() {
    if(setup_done) return true;

    _BB_SDFS.end(); //force begin() to re-initialize
 
    #ifdef BBX_USE_MMC
      _BB_SDFS.setPins(config->pin_mmc_clk, config->pin_mmc_cmd, config->pin_mmc_dat);
      if (!_BB_SDFS.begin("/sdcard", true, true, SDMMC_FREQ_DEFAULT, 5)) {
        Serial.println("BBX: SDMMC Card Mount Failed");
        return setup_done;
      }
    #else
      if (!_BB_SDFS.begin(config->spi_cs, *config->spi_bus)) {
        Serial.println("BBX: SDSPI Card Mount Failed");
        return setup_done;
      }
    #endif

    uint8_t cardType = _BB_SDFS.cardType();
    if(cardType == CARD_NONE){
      Serial.println("BBX:   No SD card attached");
      return setup_done;
    }

    setup_done = true;
    return setup_done;
  }

  bool sd_listDir(const char * dirname, uint8_t levels = 0){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = _BB_SDFS.open(dirname);
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
                sd_listDir(file.path(), levels -1);
            }
        } else {
            Serial.printf("%-14s %9d\n",file.name(),file.size());
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
          Serial.println("BBX:   mkdir /log failed");
          return;
      }
      root = _BB_SDFS.open(dirname);
      if(!root) {
          Serial.println("BBX:   mkdir /log failed");
          return;
      }
    }
    if(!root.isDirectory()){
        Serial.println("BBX:   /log is not a directory");
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
    file = _BB_SDFS.open(filename, FILE_WRITE, true);
  }
};


#undef _BB_SDFS
#undef _BB_SETUP_STR
