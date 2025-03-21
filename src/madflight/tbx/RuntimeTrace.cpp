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

#include "RuntimeTrace.h"

RuntimeTraceGroup runtime_trace_group;

RuntimeTrace::RuntimeTrace(const char* name, bool report) {
  strncpy(this->name, name, 8);
  this->report = report;
  runtime_trace_group.add(this);
}

void RuntimeTrace::begin() {
  begin_ts = micros();
}

void RuntimeTrace::end() {
  uint32_t now = micros();
  acc_dt += (micros() - begin_ts);
  acc_cnt++;
  if(report) runtime_trace_group.report(now);
}

int RuntimeTraceGroup::add(RuntimeTrace *item) {
  for(int i=0;i<RUNTIMETRACE_NUM;i++) {
    if(!arr[i]) {
      arr[i] = item;
      return i;
    }
  }
  return -1;
}

void RuntimeTraceGroup::report(uint32_t now) {
  //report once per second
  if(now - ts > 10000000) {
    Serial.printf("TRACE:");
    uint32_t dt = (now - ts) / 1000; //in [ms]
    ts = now;
    for(int i=0;i<RUNTIMETRACE_NUM;i++) {
      RuntimeTrace *t = arr[i];
      if(!t) break;
      uint32_t hz = t->acc_cnt * 1000 / dt;
      uint32_t rt = t->acc_dt / t->acc_cnt;
      t->acc_cnt = 0;
      t->acc_dt = 0;
      Serial.printf("\t%s.hz:%d\t%s.rt:%d", t->name, (int)hz, t->name, (int)rt);
    }
    Serial.println();
  }
}

/*
RuntimeTrace::RuntimeTrace(const char* name) {}
void RuntimeTrace::begin() {}
void RuntimeTrace::end() {}
int RuntimeTraceGroup::add(RuntimeTrace *item) {}
void RuntimeTraceGroup::report(uint32_t now) {}
*/