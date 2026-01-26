/*==========================================================================================
MIT License

Copyright (c) 2023-2026 https://madflight.com

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

/*=============================================================================
Minimal Servo / PWM library for RP2040

NOTE: no checking is done on overlapping slices - user should take care not to assign different frequencies to two pins in the same slice.

Example
-------
#include "RP2040_PWM.h"

PWM motor;
PWM servo;

float motor_pwm = 125;
float servo_pwm = 1000;

void setup() {
  motor.begin(1,2000,125,250); //pin 1 (slice 0): Oneshot motor ESC 2000Hz pulse 125-250 us
  servo.begin(2,50,1000,2000); //pin 2 (slice 1): regular servo 50Hz pulse 1000-2000 us 
}

void loop() {
  motor_pwm++;
  if(motor_pwm > motor.get_max_us()) motor_pwm = motor.get_min_us();
  motor.writeMicroseconds(motor_pwm);

  servo_pwm++;
  if(servo_pwm > servo.get_max_us()) servo_pwm = servo.get_min_us();
  servo.writeMicroseconds(servo_pwm);

  delay(3);
}

=============================================================================*/

#pragma once

//Maximum number of PWM outputs
#define PWM_MAX 16

#include <hardware/pwm.h>

class PWM
{
  public:
    PWM() {};
    bool begin(int pin, float req_freq, float min_us, float max_us) {
        this->pin = pin;
        this->req_freq = req_freq;
        this->min_us = min_us;
        this->max_us = max_us;

        //find divider so that full 16bit count results in requested freq (i.e. find divider for maximum resolution)
        float divider = (float)clock_get_hz(clk_sys) / (1<<16) / req_freq;
        //divider is 8:4 fractional - round divider up to next 1/16th, to ensure that wrap less than 16 bits
        divider = ceil(divider * 16) / 16.0;
        if(divider < 1) divider = 1;

        //calculate timer wrap count value
        float act_clk = (float)clock_get_hz(clk_sys) / divider;
        wrap = (int)(act_clk / req_freq - 1);
        act_freq = act_clk / (wrap + 1);
        inv_duty_resolution_us = 1.0e-6 * act_freq * (wrap + 1); //pre-calcuted inverse to speed up setting PWM value

        // configure PWM hardware
        slicenum = pwm_gpio_to_slice_num(pin); // get slice number
        gpio_set_function(pin, GPIO_FUNC_PWM); // assign GPIO to pwm functionality
        pwm_set_clkdiv_mode(slicenum, PWM_DIV_FREE_RUNNING); // set the clkdiv mode (this might not actually do anything)
        pwm_set_phase_correct(slicenum, false); // disable phase correct (if enabled, frequency is halved and duty cycle is doubled)
        pwm_set_clkdiv(slicenum, divider); // set divider
        pwm_set_wrap(slicenum, wrap); // set wrap
        pwm_set_gpio_level(pin, 0); // set count
        pwm_set_enabled(slicenum, true); // enable PWM slice
        
        return true;
    };

    void writeFactor(float f) {
      writeMicroseconds(min_us + f * (max_us - min_us));
    }

    void writeMicroseconds(float us) {
      if(us < min_us) us = min_us;
      if(us > max_us) us = max_us;
      int duty = us * inv_duty_resolution_us;
      if(duty < 0) duty = 0;
      if(duty > wrap) duty = wrap;
      pwm_set_gpio_level(pin, us * inv_duty_resolution_us);
    };

    int get_slicenum() {return slicenum;}
    float get_min_us() {return min_us;}
    float get_max_us() {return max_us;}
    float get_duty_resolution_us() {return 1/inv_duty_resolution_us;}
    int get_req_freq() {return req_freq;}
    int get_act_freq() {return act_freq;}

  private:
    int pin;
    int slicenum;
    int wrap;
    float min_us;
    float max_us;
    float req_freq; //requested frequency
    float act_freq; //actual frequency
    float inv_duty_resolution_us;
};
