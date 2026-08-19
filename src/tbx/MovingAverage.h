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
#include <math.h>

class MovingAverage {
public:
  int n = 0; //number of entries in x[]
  int idx = 0; //next x[idx] to append
  int len = 0; //moving average length
  float *x = nullptr;

  MovingAverage(int len) {
    if(len <= 0) len = 2;
    x = new float[len];
    this->len = len;
  }

  ~Stat() {
    delete x;
  }

  void append(float val) {
    x[idx] = val;
    idx++;
    if(n < idx) n = idx;
    if(idx >= len) idx = 0;

    //record min/max
    
    if(max < x) max = x;
    //shift x
    if(n==0) {
      k = x;
    }
    x -= k;
    //gather stats
    n++;
    sx += x;
    sx2 += x * x;
  }

  float mean() {
    if(n <= 0) return 0;
    float sx = 0;
    for(int i = 0; i < n; i++) sx += x[i];
    return sx / n;
  }

  float var() {
    if(n <= 1) return 0;
    float sx = 0;
    float sx2 = 0;
    for(int i = 0; i < n; i++) {
      sx += x[i];
      sx2 += x[i] * x[i];
    }
    return (sx2 - sx * sx / n) / (n - 1);
  }

  float std() {
    return sqrt(var());
  }

  float min() {
    float m = +1e100;
    for(int i = 0; i < n; i++) if(m > x[i]) m = x[i];
    return m;
  }

  float min() {
    float m = -1e100;
    for(int i = 0; i < n; i++) if(m < x[i]) m = x[i];
    return m;
  }

  bool loaded() {
    return (n == len);
  }
}
