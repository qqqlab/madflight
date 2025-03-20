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
