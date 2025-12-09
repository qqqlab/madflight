#ifdef ARDUINO_ARCH_RP2040

#include "BbxGizmoSdcard_RP2.h"
#include "../hal/hal.h"
#include <Arduino.h> //Serial
#include <SPI.h>
#include <SdFat.h> //pico-arduino bundled version, alternative: #include "SdFat_Adafruit_Fork.h" //use adafruit lib

// FsFile system on SD Card
SdFat sd;
// returns true if card was found
static bool sdcard_begin(BbxConfig *config) {
  sd.end(); //force begin() to re-initialize SD

  //begin sdcard
  switch(config->gizmo) {
    case Cfg::bbx_gizmo_enum::mf_SDSPI :
       if (!sd.begin( SdSpiConfig(config->spi_cs, SHARED_SPI, SD_SCK_MHZ(50), config->spi_bus) )) return false;
       break;
    case Cfg::bbx_gizmo_enum::mf_SDMMC :
       if (!sd.begin( SdioConfig(config->pin_mmc_clk, config->pin_mmc_cmd, config->pin_mmc_dat) )) return false;
      break;
    default:
      return false;
  }

  return true;
}

//--------------------------------------------------------------------+
// TinyUSB SDCard Config
//--------------------------------------------------------------------+
#ifdef USE_TINYUSB
#include <Adafruit_TinyUSB.h>

// USB Mass Storage object
Adafruit_USBD_MSC usb_msc;

// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and
// return number of copied bytes (must be multiple of block size)
static int32_t msc_read_cb (uint32_t lba, void* buffer, uint32_t bufsize) {
  bool rc = sd.card()->readSectors(lba, (uint8_t*) buffer, bufsize/512);
  return rc ? bufsize : -1;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and 
// return number of written bytes (must be multiple of block size)
static int32_t msc_write_cb (uint32_t lba, uint8_t* buffer, uint32_t bufsize) {
  bool rc = sd.card()->writeSectors(lba, buffer, bufsize/512);
  return rc ? bufsize : -1;
}

// Callback invoked when WRITE10 command is completed (status received and accepted by host).
// used to flush any pending cache.
static void msc_flush_cb (void) {
  sd.card()->syncDevice();
//  sd.cacheClear();   // clear file system's cache to force refresh ---> only in "SdFat_Adafruit_Fork.h" not in "SdFat.h"
}


void hal_usb_setup() {
  //make sure usb setup runs only once
  static bool usb_setup_done = false;
  if(!usb_setup_done) {
    usb_setup_done = true;

    // Set disk vendor id, product id and revision with string up to 8, 16, 4 characters respectively
    usb_msc.setID("madfligh", "SD Card", "1.0");

    // Set read write callback
    usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);

    // Still initialize MSC but tell usb stack that MSC is not ready to read/write
    // If we don't initialize, board will be enumerated as CDC only
    usb_msc.setUnitReady(false);
    usb_msc.begin();

    // If already enumerated, additional class driver begin() e.g msc, hid, midi won't take effect until re-enumeration
    if (TinyUSBDevice.mounted()) {
      TinyUSBDevice.detach();
      delay(10);
      TinyUSBDevice.attach();
    }
  }

  static bool unit_ready = false;
  if(sd.clusterCount() > 0 && !unit_ready) {
    // Set disk size, SD block size is always 512
    usb_msc.setCapacity(sd.card()->sectorCount(), 512);

    // MSC is ready for read/write
    usb_msc.setUnitReady(true);
    unit_ready = true;
  }
}
#else //USE_TINYUSB
void hal_usb_setup() {}
#endif //USE_TINYUSB


//--------------------------------------------------------------------+
// BbxGizmoSdcard
//--------------------------------------------------------------------+

BbxGizmoSdcard* BbxGizmoSdcard::create(BbxConfig *config) {
  //check config
  switch(config->gizmo) {
    case Cfg::bbx_gizmo_enum::mf_SDSPI :
      if(!config->spi_bus) return nullptr;
      if(config->spi_cs < 0) return nullptr;
      break;
    case Cfg::bbx_gizmo_enum::mf_SDMMC :
      if(config->pin_mmc_clk < 0) return nullptr;
      if(config->pin_mmc_cmd < 0) return nullptr;
      if(config->pin_mmc_dat < 0) return nullptr;
      break;
    default:
      return nullptr;
  }

  //create gizmo
  BbxGizmoSdcard* gizmo = new BbxGizmoSdcard(config);

  //start sdcard
  gizmo->sd_setup();

  return gizmo;
}

bool BbxGizmoSdcard::sd_setup() {
  if(setup_done) return true;
  hal_usb_setup(); //start tinyusb if not started yet (need to do this before sdcard_begin())
  setup_done = sdcard_begin(config);
  hal_usb_setup(); //connect card if inserted

  return setup_done;
}

//setup the file system
void BbxGizmoSdcard::setup() {
  if(sd_setup()) {
    Serial.printf("BBX: SDCARD size:%d MB  free:%d MB\n", (int)((float)sd.clusterCount()*sd.bytesPerCluster()/1000000), (int)((float)sd.freeClusterCount()*sd.bytesPerCluster()/1000000));
  }else{
    Serial.printf("BBX: SDCARD not found\n");
  }
}

bool BbxGizmoSdcard::writeOpen() {
  close();

  int attempt = 0;
  while(++attempt<3) {
    if(sd_setup()) {
      memset(wbuf, 0xff, sizeof(wbuf));
      wbuf_len = 0;

      sd_logOpen(BB_LOG_DIR_NAME);
      if(file) {
        Serial.printf("BBX:   start OK - attempt=%d file=%s\n", attempt, filename.c_str());
        return true;
      }
    }

    Serial.println("BBX:   start retry");
    setup_done = false; //force setup to re-run
  }
  Serial.println("BBX:   start FAILED");
  return false;
}

void BbxGizmoSdcard::_writeChar(uint8_t c) {
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

void BbxGizmoSdcard::write(const uint8_t *buf, const uint8_t len) {
  for(int i=0;i<len;i++) {
    _writeChar(buf[i]);
  }
}

void BbxGizmoSdcard::close() {
  if(file) {
    if(wbuf_len>0) {
      file.write(wbuf, wbuf_len);
      wbuf_len = 0;
    }
    file.flush();
    int len = file.size();
    file.close();
    file = {};
    Serial.printf("BBX:   stop file=%s len=%d\n", filename.c_str(), len);
  }
}

void BbxGizmoSdcard::erase() {
  if(sd_setup()) {
    sd_deleteFilesFromDir(BB_LOG_DIR_NAME);
  }
}

void BbxGizmoSdcard::dir() {
  int attempt = 0;
  while(++attempt<2) {
    if(sd_setup()) {
      if(sd_listDir(BB_LOG_DIR_NAME)) return;
    }
    setup_done = false; //force setup to re-run
  }
}

void BbxGizmoSdcard::bench() {
    const char* path = "madfli.ght";
    uint8_t *buf;
    size_t len = 0;
    uint32_t start;
    uint32_t end;
    size_t flen;
    FsFile file;
    
    buf = (uint8_t*)malloc(512);
    if(!buf) {
        Serial.println("BBX:   bench - malloc failed");
        return;
    }

    if(sd.exists(path)) {
      sd.remove(path);
    }

    file = sd.open(path, FILE_WRITE);
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
    
    file = sd.open(path);
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

void BbxGizmoSdcard::info() {
  // 0 - SD V1, 1 - SD V2, or 3 - SDHC/SDXC
  // print the type of card
  Serial.print("Card type:      ");
  switch (sd.card()->type()) {
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
  Serial.printf("Volume type:    FAT%d\n", sd.fatType());
  Serial.printf("Cluster size:   %d\n", sd.bytesPerCluster());
  Serial.printf("Volume size:    %d MB\n", (int)((float)sd.clusterCount()*sd.bytesPerCluster()/1000000));
  Serial.printf("Free:           %d MB\n", (int)((float)sd.freeClusterCount()*sd.bytesPerCluster()/1000000));

  Serial.println();

  sd_listDir("/");
}

bool BbxGizmoSdcard::sd_listDir(const char * dirname, uint8_t levels) {
    Serial.printf("Listing directory: %s\n", dirname);

    FsFile root = sd.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return false;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return true;
    }

    FsFile file = root.openNextFile();
    char fn[200];
    while(file){
        file.getName(fn,200);
        if(file.isDirectory()){
            Serial.printf("%-25s <DIR>\n", fn);
            if(levels){
                sd_listDir(fn, levels -1);
            }
        } else {
            Serial.printf("%-21s %9d\n", fn, (int)file.size());
        }
        file = root.openNextFile();
    }
    return true;
}

void BbxGizmoSdcard::sd_deleteFilesFromDir(const char * dirname){
    FsFile root = sd.open(dirname);
    if(!root) return;
    if(!root.isDirectory()) return;

    FsFile file = root.openNextFile();
    while(file){
        if(!file.isDirectory()){
          file.remove();
        }
        file = root.openNextFile();
    }
}

void BbxGizmoSdcard::sd_logOpen(const char * dirname){
  FsFile root = sd.open(dirname);
  if(!root){
      if(!sd.mkdir(dirname)){
        Serial.println("BBX:   mkdir /log failed");
        return;
    }
    root = sd.open(dirname);
    if(!root) {
        Serial.println("BBX:  mkdir /log failed");
        return;
    }
  }
  if(!root.isDirectory()){
      Serial.println("BBX:   /log is not a directory");
      return;
  }

  FsFile dirfile = root.openNextFile();
  int maxnr = 0;
  while(dirfile){
    char dirname[200];
    dirfile.getName(dirname,200);
    String fn = String(dirname);
    if(fn.endsWith(".bin")) {
      int nr = fn.toInt();
      if(maxnr<nr) maxnr = nr;
    }
    dirfile = root.openNextFile();
  }
  filename = String(dirname) + '/' + String(maxnr+1) + ".bin";
  Serial.println(filename);
  file = sd.open(filename, FILE_WRITE);
}


int BbxGizmoSdcard::read(const char* filename, uint8_t **data) {
  if(!sd_setup()) return 0;
  if(!sd.exists(filename)) return 0;
  FsFile file = sd.open(filename, FILE_READ);
  int len = file.size();
  if(len == 0) return 0;
  uint8_t* buf = (uint8_t*)malloc(len);
  if(!buf) return 0;
  file.read(buf, len);
  file.close();
  *data = buf;
  return len;
}

void BbxGizmoSdcard::printSummary() {
  Serial.printf("BBX: SDCARD - ");
  if(sd.clusterCount() > 0) {
    Serial.printf("Size: %d MB, ", (int)((float)sd.clusterCount()*sd.bytesPerCluster()/1000000));
    Serial.printf("Free: %d MB, ", (int)((float)sd.freeClusterCount()*sd.bytesPerCluster()/1000000));
  } else {
    Serial.printf("No card inserted, ");
  }
  #ifdef USE_TINYUSB
    Serial.printf("TinyUSB Mass Storage available\n");
  #else
    Serial.printf("No TinyUSB Mass Storage\n");
  #endif
}

#endif //#ifdef ARDUINO_ARCH_RP2040
