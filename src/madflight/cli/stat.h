#pragma once

#include <math.h>
#include <Arduino.h>

//Algorithms for calculating variance, Computing shifted data: https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
class Stat {
public:
  int n = 0;
  float k = 0;
  float sx = 0;
  float sx2 = 0;
  float min = +1e100;
  float max = -1e100;
  void append(float x) {
    //record min/max
    if(min > x) min = x;
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
    return k + sx / n;
  }
  float var() {
    return (sx2 - sx * sx / n) / (n - 1);
  }
  float std() {
    return sqrt(var());
  }
  void print(const char *hdr) {
    Serial.printf("%s mean:%+f stdev:%f var:%f min:%+f max:%+f n:%d\n", hdr, mean(), std(), var(), min, max, n);
  }
};
