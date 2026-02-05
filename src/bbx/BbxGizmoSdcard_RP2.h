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
