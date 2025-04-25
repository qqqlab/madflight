#pragma once

#include "bbx.h"
#include <SPI.h>
#include <SD.h>

class BbxGizmoSdspi : public BbxGizmo {
private:
  SPIClass *spi;
  int pin_cs;

public:
  BbxGizmoSdspi(SPIClass *spi, int pin_cs);

private:
  const char* BB_LOG_DIR_NAME = "/log";
  bool setup_done = false;
  uint8_t wbuf[512];
  uint32_t wbuf_len = 0;
  String filename;
  File file = {};

public:
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

private:
  bool sd_setup();
  bool sd_listDir(const char * dirname, uint8_t levels=0);
  void sd_deleteFilesFromDir(const char * dirname);
  void sd_logOpen(const char * dirname);
};