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
  int hist_len = 0;
  float *hist = nullptr;

  Stat(int hist_len = 0) {
    if(hist_len > 0) {
      hist = new float[hist_len];
      if(hist) this->hist_len = hist_len;
    }
  }

  ~Stat() {
    delete hist;
  }

  void append(float x) {
    //record history
    if(n < hist_len) hist[n] = x;
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

  int hist_cnt() {
    return (n < hist_len ? n : hist_len);
  }

  void print(const char *hdr, float dt = 0) {
    Serial.printf("%s\tmean:%+f\tstdev:%f\tmin:%+f\tmax:%+f\tn:%d", hdr, mean(), std(), min, max, n);
    if(dt > 0) Serial.printf("\trate:%.0fHz\tnoise:%f", n / dt, std() / sqrt(n / dt));
    Serial.println();
  }

  //print spikes that are more than 4*std away from mean
  void print_spikes(const char *hdr) {
    if(hist_len == 0) return;
    float m = mean();
    float limit = 4 * std();
    Serial.printf("Spikes outside %f to %f for %s\n", m - limit, m + limit, hdr);
    int i = 2;
    while(i < hist_cnt() - 2) {
      if(fabs(hist[i] - m) > limit) {
        Serial.printf("%6d: %f %f -->", i-2, hist[i-2],  hist[i-1]); //before
        while(fabs(hist[i] - m) > limit) {
          Serial.printf(" %f", hist[i]); //more than limit off mean
          i++;
        }
        Serial.printf(" <-- %f %f\n", hist[i], hist[i+1]); //after
      }
      i++;
    }
  }

};
