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