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

#include "../hal/hal.h" //class PWM, Dshot
#include <stdint.h> //uint8_t
#include "../tbx/msg.h"

struct OutState {
  public:
    //uint32_t ts; //TODO
    bool armed = false; //output is enabled when armed == true
    float command[OUT_SIZE] = {}; //last commanded outputs (values: 0.0 to 1.0)
    int eperiod[OUT_SIZE] = {}; //ePeriod in [us], 0 when motor stopped, negative on error
};

class Out : public OutState {
  public:
    MsgTopic<OutState> topic = MsgTopic<OutState>("out");

    bool eperiodEnabled[OUT_SIZE] = {}; //ePeriod enabled flag

    void setup();
    bool setupDshot(uint8_t cnt, int* idxs, int* pins, int freq_khz = 300);
    bool setupDshotBidir(uint8_t cnt, int* idxs, int* pins, int freq_khz = 300);    
    bool setupMotors(uint8_t cnt, int* idxs, int* pins, int freq_hz = 400, int pwm_min_us = 950, int pwm_max_us = 2000);
    bool setupMotor(uint8_t idx, int pin, int freq_hz = 400, int pwm_min_us = 950, int pwm_max_us = 2000);
    bool setupServo(uint8_t idx, int pin, int freq_hz = 400, int pwm_min_us = 950, int pwm_max_us = 2000);
    void set(uint8_t idx, float value); //set output (might not be output value because of armed == false)
    float get(uint8_t idx); //get last set value (might not be output value because of armed == false)
    char getType(uint8_t idx); //type 'D', 'M', or 'S'
    int rpm(uint8_t idx, int poles = 14); //get RPM

  private:
    bool _setupOutput(char typ, uint8_t idx, int pin, int freq_hz, int pwm_min_us, int pwm_max_us);

    char type[OUT_SIZE] = {};
    int pins[OUT_SIZE] = {};


    //PWM
    PWM pwm[OUT_SIZE]; //ESC and Servo outputs (values: 0.0 to 1.0)

    //Dshot
    Dshot dshot;
    DshotBidir dshotbidir;
    int dshot_cnt = 0; //number of dshot pins
    int dshot_idxs[OUT_SIZE]; //dshot out indices
};

extern Out out;
