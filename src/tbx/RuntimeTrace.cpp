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

RuntimeTrace* RuntimeTraceGroup::arr[RUNTIMETRACE_NUM] = {};

RuntimeTrace::RuntimeTrace(const char* name) {
  strncpy(this->name, name, 8);
  RuntimeTraceGroup::add(this);
}

void RuntimeTrace::start() {
  start_ts = micros();
}

void RuntimeTrace::stop(bool updated) {
  volatile uint32_t now = micros();
  dt += (now - start_ts);
  n++;
  if(updated) {
    dt_upd += (now - start_ts);
    n_upd++;
  }
}

void RuntimeTrace::reset(uint32_t now) {
  reset_ts = now;
  dt = 0;
  n = 0;
  dt_upd = 0;
  n_upd = 0;
}

void RuntimeTrace::print(uint32_t now) {
  uint32_t reset_dt = (now - reset_ts);
  float hz = 1e6f * n / reset_dt;
  perc = 100.0f * dt / reset_dt;
  float rt = (n == 0 ? 0 : (float)dt / n);
  float hz_upd = 1e6f * n_upd / reset_dt;
  perc_upd = 100.0f * dt_upd / reset_dt;
  float rt_upd = (n_upd == 0 ? 0 : (float)dt_upd / n_upd);
  Serial.printf("%-8s %8.0fHz  %6.2f%%   %9.2fus %8.0fHz  %6.2f%%   %9.2fus\n", name, hz, perc, rt,  hz_upd, perc_upd, rt_upd);
}






int RuntimeTraceGroup::add(RuntimeTrace *item) {
  for(int i=0 ; i < RUNTIMETRACE_NUM; i++) {
    if(!arr[i]) {
      arr[i] = item;
      return i;
    }
  }
  return -1;
}

void RuntimeTraceGroup::print() {
  if(!arr[0]) return;
  volatile uint32_t now = micros();
  uint32_t reset_dt = (now - arr[0]->reset_ts);
  Serial.printf("\n=== Wallclock Runtime - Measurement Period: %.2f seconds ===\n\n", 1e-6 * reset_dt);

  Serial.printf("Module        Calls  Wallclock     Runtime    Updates  Wallclock     Runtime\n");

  //show non _xxx traces
  float perc_sum = 0;
  float perc_sum_t = 0;  
  for(int i  = 0; i < RUNTIMETRACE_NUM; i++) {
    RuntimeTrace *t = arr[i];
    if(!t) break;
    if(t->name[0] == '_') continue;
    t->print(now);
    perc_sum += t->perc;
    perc_sum_t += t->perc_upd;
  }

  Serial.printf("Other                %6.2f%%                           %6.2f%%              \n", 100.f - perc_sum, 0);

  Serial.printf("Total  ------------  %6.2f%%  -----------------------  %6.2f%%  ------------\n", 100.f, perc_sum_t);

  //show _xxx traces
  for(int i  = 0; i < RUNTIMETRACE_NUM; i++) {
    RuntimeTrace *t = arr[i];
    if(!t) break;
    if(t->name[0] != '_') continue;
    t->print(now);
    perc_sum += t->perc;
    perc_sum_t += t->perc_upd;
  }

  reset();
}

void RuntimeTraceGroup::reset() {
  volatile uint32_t now = micros();
  for(int i  = 0; i < RUNTIMETRACE_NUM; i++) {
    RuntimeTrace *t = arr[i];
    if(!t) break;
    t->reset(now);
  }
}
