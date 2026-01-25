/*==========================================================================================
MIT License

Copyright (c) 2023-2025 https://madflight.com

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

#define RUNTIMETRACE_NUM 20

#include <Arduino.h>

class RuntimeTrace {
  friend class RuntimeTraceGroup;
public:
  RuntimeTrace(const char* name);
  void start();
  void stop(bool updated);

private:
  char name[9] = {};
  volatile uint32_t start_ts = 0; //start() timestamp
  uint32_t reset_ts = micros(); //counting since
  uint32_t n = 0; //accumulated loops
  uint32_t dt = 0; //accumulated runtime
  uint32_t n_upd = 0; //accumulated updated loops
  uint32_t dt_upd = 0; //accumulated updated runtime
  float perc = 0; //for print
  float perc_upd = 0; //for print

  void reset(uint32_t now);
  void print(uint32_t now);
};

class RuntimeTraceGroup {
public:
  static int add(RuntimeTrace *item);
  static void print();
  static void reset();

private:
  static RuntimeTrace* arr[RUNTIMETRACE_NUM];  
};
