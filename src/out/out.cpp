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

#include "out.h"

//global module class instance
Out out;

char Out::getType(uint8_t idx) {
  if(idx >= OUT_SIZE) return 0;
  return type[idx];
}

float Out::get(uint8_t idx) {//get last set value (might not be output because of armed == false)
  if(idx >= OUT_SIZE) return 0;
  return command[idx];
}

void Out::setup() {}

bool Out::_setupOutput(char typ, uint8_t idx, int pin, int freq_hz, int pwm_min_us, int pwm_max_us){
  if(idx >= OUT_SIZE) return false;
  type[idx] = typ;
  pins[idx] = pin;
  command[idx] = 0.0f;
  eperiod[idx] = -1;
  eperiodEnabled[idx] = (type[idx] == 'B');
  switch(type[idx]) {
    case 'D':
    case 'B':
      //do nothing
      break;
    case 'M':
    case 'S':
      pwm[idx].begin(pin, freq_hz, pwm_min_us, pwm_max_us);
      pwm[idx].writeFactor(command[idx]);
      break;
    default:
      return false;
  }
  return true;
}

bool Out::setupDshot(uint8_t cnt, int* idxs, int* pins, int freq_khz) {
  for(int i = 0; i < cnt; i++) {
    if(!_setupOutput('D', idxs[i], pins[i], 0, 0, 0)) return false;
    dshot_idxs[i] = idxs[i];
  }
  if(!dshot.setup(pins, cnt, freq_khz)) return false;
  dshot_cnt = cnt;
  return true; 
}

bool Out::setupDshotBidir(uint8_t cnt, int* idxs, int* pins, int freq_khz) {
  for(int i = 0; i < cnt; i++) {
    if(!_setupOutput('B', idxs[i], pins[i], 0, 0, 0)) return false;
    dshot_idxs[i] = idxs[i];
  }
  if(!dshotbidir.setup(pins, cnt, freq_khz)) return false;
  dshot_cnt = cnt;
  return true; 
}

bool Out::setupMotors(uint8_t cnt, int* idxs, int* pins, int freq_hz, int pwm_min_us, int pwm_max_us) {
  for(int i = 0; i < cnt; i++) {
    if(!setupMotor(idxs[i], pins[i], freq_hz, pwm_min_us, pwm_max_us)) return false;
  }
  return true;
}

bool Out::setupMotor(uint8_t idx, int pin, int freq_hz, int pwm_min_us, int pwm_max_us) {
  return _setupOutput('M', idx, pin, freq_hz, pwm_min_us, pwm_max_us);
}

bool Out::setupServo(uint8_t idx, int pin, int freq_hz, int pwm_min_us, int pwm_max_us) {
  return _setupOutput('S', idx, pin, freq_hz, pwm_min_us, pwm_max_us);
}

void Out::set(uint8_t idx, float value) {
  if(idx >= OUT_SIZE) return;
  command[idx] = value;
  switch(type[idx]) {
    case 'D':
    case 'B':
      //only send throttle values on last mot index
      if(idx == dshot_idxs[dshot_cnt - 1]) {
        uint16_t throttle[OUT_SIZE];
        for(int i = 0; i < dshot_cnt; i++) {
          int idx2 = dshot_idxs[i];
          float v = constrain(command[idx2], 0.0f, 1.0f);
          if(!armed) v = 0;
          throttle[idx2] = v * 2000;
        }
        if(type[idx] == 'D') {
          dshot.set_throttle(throttle);
        }else{
          dshotbidir.get_eperiod(eperiod);
          dshotbidir.set_throttle(throttle);
        }
      }
      topic.publish(this);
      break;
    case 'M':
    case 'S':
      if(armed) {
        pwm[idx].writeFactor(value);
      }else{
        if(type[idx] == 'M') pwm[idx].writeFactor(0);
      }
      topic.publish(this);
      break;
  }
}

int Out::rpm(uint8_t idx, int poles) {
  if(idx >= OUT_SIZE) return -1;
  if(eperiod[idx] < 0) return eperiod[idx];
  if(eperiod[idx] == 0) return 0;
  return 120000000 / poles / eperiod[idx];
}