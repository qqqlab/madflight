/*==========================================================================================
out.h - madflight motor and servo output driver

MIT License

Copyright (c) 2023-2024 https://madflight.com

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

#include "../interface.h"

static PWM pwm[HW_OUT_COUNT]; //ESC and Servo outputs (values: 0.0 to 1.0)
static float command[HW_OUT_COUNT] = {}; //last commanded outputs (values: 0.0 to 1.0)
static char type[HW_OUT_COUNT] = {};

char Out::getType(uint8_t i) {
  if(i >= HW_OUT_COUNT) return 'X';
  return type[i];
}

float Out::get(uint8_t i) {//get last set value (might not be output because of armed == false)
  if(i >= HW_OUT_COUNT) return 0;
  return command[i];
}

void Out::setup() {
}

bool Out::_setupOutput(char typ, uint8_t i, int pin, int freq_hz, int pwm_min_us, int pwm_max_us){
  if(i >= HW_OUT_COUNT) return false;
  type[i] = typ;
  pwm[i].begin(pin, freq_hz, pwm_min_us, pwm_max_us);
  command[i] = 0;
  pwm[i].writeFactor(command[i]);
  return true;
}

bool Out::setupMotor(uint8_t i, int pin, int freq_hz, int pwm_min_us, int pwm_max_us) {
  return _setupOutput('M', i, pin, freq_hz, pwm_min_us, pwm_max_us);
}
bool Out::setupServo(uint8_t i, int pin, int freq_hz, int pwm_min_us, int pwm_max_us) {
  return _setupOutput('S', i, pin, freq_hz, pwm_min_us, pwm_max_us);
}

void Out::set(uint8_t i, float value) {
  if(i >= HW_OUT_COUNT) return;
  command[i] = value;
  if(armed) {
    pwm[i].writeFactor(value);
  }else{
    if(type[i] == 'M') pwm[i].writeFactor(0);
  }
}

Out out;

