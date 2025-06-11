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

#pragma once

#define OUT_SIZE 16 //max number of outputs

#include "../hal/hal.h"
#include <stdint.h> //uint8_t

class Out {
  public:
    bool armed = false; //output is enabled when armed == true

    void setup();
    bool setupMotor(uint8_t i, int pin, int freq_hz = 400, int pwm_min_us = 950, int pwm_max_us = 2000);
    bool setupServo(uint8_t i, int pin, int freq_hz = 400, int pwm_min_us = 950, int pwm_max_us = 2000);
    void set(uint8_t i, float value); //set output (might not be output value because of armed == false)
    float get(uint8_t i); //get last set value (might not be output value because of armed == false)
    char getType(uint8_t i); //type 'M' or 'S'

  private:
    bool _setupOutput(char typ, uint8_t i, int pin, int freq_hz, int pwm_min_us, int pwm_max_us);
    
    PWM pwm[OUT_SIZE]; //ESC and Servo outputs (values: 0.0 to 1.0)
    float command[OUT_SIZE] = {}; //last commanded outputs (values: 0.0 to 1.0)
    char type[OUT_SIZE] = {};
};

extern Out out;
