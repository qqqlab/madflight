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

/* simple scheduler, keeping exact sampling period

example usage:

MF_Schedule schedule;

if(schedule.interval(1000)) {
  //this code is called every 1000 us
}

*/

#pragma once

class MF_Schedule {
private:
  uint32_t ts = 0;

public:
  MF_Schedule() {
    ts = micros();
  }

  bool interval(uint32_t interval_us)
  {
    uint32_t now = micros();
    if(now - ts < interval_us) return false;
    if(now - ts < 2 * interval_us) {
      ts += interval_us; //keep exact interval_us timing
    }else{
      ts = now; //unless we missed an interval
    }
    return true;
  }
};
