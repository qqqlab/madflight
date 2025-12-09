#ifdef ARDUINO_ARCH_RP2040

#pragma once

#include "bbx.h"
#include <SdFat.h> //FsFile

class BbxGizmoSdcard : public BbxGizmo {
private:
  BbxConfig *config;
  const char* BB_LOG_DIR_NAME = "/log";
  bool setup_done = false;
  uint8_t wbuf[512];
  uint32_t wbuf_len = 0;
  String filename;
  FsFile file = {}; //current log file

  BbxGizmoSdcard(BbxConfig *_config) : config{_config} {}; //private constructor

public:
  static BbxGizmoSdcard* create(BbxConfig *config);

  //setup the file system
  void setup() override;
  bool writeOpen() override;

private:
  void _writeChar(uint8_t c);

public:
  void write(const uint8_t *buf, const uint8_t len) override;
  void close() override;
  void erase() override;
  void dir() override;
  void bench() override;
  void info() override;
  int read(const char* filename, uint8_t **data) override;
  void printSummary() override;
  
private:
  bool sd_setup();
  bool sd_listDir(const char * dirname, uint8_t levels=0);
  void sd_deleteFilesFromDir(const char * dirname);
  void sd_logOpen(const char * dirname);
};

#endif //#ifdef ARDUINO_ARCH_RP2040
