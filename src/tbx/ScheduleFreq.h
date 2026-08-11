/*==========================================================================================
MIT License

Copyright (c) 2025 https://madflight.com

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

/*ScheduleFreq.h - schedule on exact intervals

ScheduleFreq schedule_10Hz = ScheduleFreq(10);

void loop() {
  if(schedule_10Hz.expired()) {
    Serial.println("schedule"); //this is executed at exactly 10Hz (as long as loop() is called often enough)
  }
}

*/

#pragma once

#include <Arduino.h>

class ScheduleFreq {
  public:
    uint32_t ts_last = 0; //[us]
    uint32_t interval = 0; //[us]

    ScheduleFreq(float freq_hz) {
      if(freq_hz <= 0){
        interval = 0xFFFFFFFF;
      }else if(freq_hz >= 1000000){
        interval = 1;
      }else{
        interval = 1000000 / freq_hz;
      }
    }

    //returns true if at least 1 interval passed since last call
    bool expired() {
      uint32_t now = micros();
      if(now - ts_last < interval) return false;
      ts_last += interval; //timeout for next dt interval
      if(now - ts_last < interval) return true;
      ts_last = now; //resync
      return true;
    }

    //returns number of expired intervals since last call
    uint32_t expired_count() {
      uint32_t now = micros();
      uint32_t dt = now - ts_last;
      if(dt < interval) {
        return 0;
      }
      if(dt < 2 * interval) {
        //early exit to avoid divide/multiply overhead
        ts_last += interval; //timeout for next dt interval
        return 1;
      }
      //last call was more than 2 intervals ago
      uint32_t intervals = dt / interval;
      ts_last += intervals * interval; //timeout for next dt interval
      return intervals;
    }
};

// Extended ScheduleFreq - keeps track of actual update timestamps
class ScheduleFreqExt {
  public:
    uint32_t ts_interval_us = 0;
    uint32_t dt_interval_us = 0;
    uint32_t ts_actual_us = 0;
    uint32_t dt_actual_us = 0;

    ScheduleFreqExt(float freq_hz) {
      if(freq_hz <= 0){
        dt_interval_us = 0xFFFFFFFF;
      }else if(freq_hz >= 1000000){
        dt_interval_us = 1;
      }else{
        dt_interval_us = 1000000 / freq_hz;
      }
    }

    bool expired() {
      uint32_t now = micros();
      if(now - ts_interval_us < dt_interval_us) {
        return false;
      }
      ts_interval_us += dt_interval_us; //timeout for next dt interval
      if(now - ts_interval_us >= dt_interval_us) {
        ts_interval_us = now; //resync
      }
      if(ts_actual_us == 0){
        dt_actual_us = dt_interval_us; //assume interval (avoid returning 0)
      }else{
        dt_actual_us = now - ts_actual_us;
      } 
      ts_actual_us = now;
      return true;
    }

    float dt_actual_s() {
      return (float)dt_actual_us * 1e-6f;
    }
};
