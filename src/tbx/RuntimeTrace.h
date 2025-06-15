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
public:
  char name[9] = {0};
  uint32_t begin_ts = 0;
  uint32_t acc_cnt = 0; //accumulated update loops
  uint32_t acc_dt = 0; //accumulated runtime
  bool report = true;

  RuntimeTrace(const char* name, bool report = true);
  void begin();
  void end();
};

class RuntimeTraceGroup {
private:
  uint32_t ts = 0; //last statistics timestamp
  RuntimeTrace* arr[RUNTIMETRACE_NUM] = {0};

public:
  int add(RuntimeTrace *item);
  void report(uint32_t now);
};

extern RuntimeTraceGroup runtime_trace_group;
