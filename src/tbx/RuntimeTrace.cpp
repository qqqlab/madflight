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

// adds approx 7us overhead on RP2350 @ 150Mhz with pico-arduino v5.5.0 - millis() = to_us_since_boot(get_absolute_time())
// adds approx 4us overhead on RP2350 @ 150Mhz with pico-arduino v5.5.1 - millis() = time_us_32()

#ifndef MF_USE_RUNTIMETRACE
  #define MF_USE_RUNTIMETRACE 1
#endif

#include "RuntimeTrace.h"

#if MF_USE_RUNTIMETRACE
//-----------------------------------------------------------------------------
// RuntimeTrace
//-----------------------------------------------------------------------------
RuntimeTrace::RuntimeTrace(const char* name) {
  strncpy(this->name, name, 8);
  RuntimeTraceGroup::add(this);
}

void RuntimeTrace::start() {
  start_ts = micros();
}

void RuntimeTrace::stop(bool updated) {
  if(updated) {
    dt1 += (micros() - start_ts);
    n1++;
  }else{
    dt0 += (micros() - start_ts);
    n0++;
  }
}

void RuntimeTrace::reset(uint32_t now) {
  reset_ts = now;
  dt0 = 0;
  n0 = 0;
  dt1 = 0;
  n1 = 0;
}

void RuntimeTrace::print(uint32_t now, float *perc, float *perc1) {
  uint32_t reset_dt = (now - reset_ts);
  float hz = 1e6f * (n0 + n1) / reset_dt;
  *perc = 100.0f * (dt0 + dt1) / reset_dt;
  float rt = (n0 + n1 == 0 ? 0 : (float)(dt0 + dt1) / (n0 + n1));
  float hz1 = 1e6f * n1 / reset_dt;
  *perc1 = 100.0f * dt1 / reset_dt;
  float rt1 = (n1 == 0 ? 0 : (float)dt1 / n1);
  Serial.printf("%-8s %8.0fHz  %6.2f%%   %9.2fus %8.0fHz  %6.2f%%   %9.2fus\n", name, hz, *perc, rt,  hz1, *perc1, rt1);
}

//-----------------------------------------------------------------------------
// RuntimeTraceGroup
//-----------------------------------------------------------------------------
RuntimeTrace* RuntimeTraceGroup::arr[RUNTIMETRACE_NUM] = {};

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

  Serial.printf("Module        Calls  Wallclock     Rt/call    Updates  Wallclock      Rt/upd\n");

  //show traces
  float perc_sum = 0;
  float perc1_sum = 0;  
  for(int i  = 0; i < RUNTIMETRACE_NUM; i++) {
    RuntimeTrace *t = arr[i];
    if(!t) break;
    float perc, perc1;
    t->print(now, &perc, &perc1);
    perc_sum += perc;
    perc1_sum += perc1;
  }

  Serial.printf("Total  ------------  %6.2f%%  -----------------------  %6.2f%%  ------------\n", perc_sum, perc1_sum);

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

#else //#if MF_USE_RUNTIMETRACE

//-----------------------------------------------------------------------------
// disable RuntimeTrace
//-----------------------------------------------------------------------------
RuntimeTrace::RuntimeTrace(const char* name) {}
void RuntimeTrace::start() {}
void RuntimeTrace::stop(bool updated) {}
void RuntimeTrace::reset(uint32_t now) {}
void RuntimeTrace::print(uint32_t now, float *perc, float *perc1) {}
int RuntimeTraceGroup::add(RuntimeTrace *item) {return 0;}
void RuntimeTraceGroup::print() {}
void RuntimeTraceGroup::reset() {}

#endif //#if MF_USE_RUNTIMETRACE