/*==========================================================================================
MIT License

Copyright (c) 2026 https://madflight.com

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

#pragma once

#include "bbx.h"
#include "../hal/hal.h"

class BbxGizmoOpenlog : public BbxGizmo {
private:
  BbxConfig *config;
  MF_Serial *ser;
  BbxGizmoOpenlog() {} //private constructor

public:
  static BbxGizmoOpenlog* create(BbxConfig *config) {
    if(config->bbx_baud <= 0) config->bbx_baud = 115200;
    MF_Serial *ser = hal_get_ser_bus(config->bbx_ser_bus, config->bbx_baud);
    if(!ser) {
      Serial.println("BBX: ERROR invalid serial port");
      return nullptr;
    }
    auto gizmo = new BbxGizmoOpenlog();
    gizmo->config = config;
    gizmo->ser = ser;
    return gizmo;
  }

  void write(const uint8_t *buf, const uint8_t len) override {
    ser->write((uint8_t*)buf, len);
  }

  void printSummary() override {
    Serial.printf("BBX: Openlog on serial%d at %d baud\n", config->bbx_ser_bus, config->bbx_baud);
  }

  void info() override {
    printSummary();
  }

  bool writeOpen() override {
    return true;
  }

  void setup() override {}
  void close() override {}
  void erase() override {}
  void dir() override {}
  void bench() override {}
  int read(const char* filename, uint8_t **data) override {return 0;}
};
