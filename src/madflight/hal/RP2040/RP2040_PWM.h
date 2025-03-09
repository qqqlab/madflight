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
    bool begin(int pin, int freq, float min_us, float max_us) {
        this->pin = pin;
        this->req_freq = freq;
        this->min_us = min_us;
        this->max_us = max_us;

        //find divider so that full 16bit count results in freq
        this->max_duty = (1<<16) - 1;
        float divider = clock_get_hz(clk_sys) / (max_duty+1) / freq;
        this->act_freq = freq;
        this->inv_duty_resolution_us = 1.0e-6 * act_freq * (max_duty+1);

        // get slice number
        this->slicenum = pwm_gpio_to_slice_num(pin);
        
        // assign GPIO to pwm functionality
        gpio_set_function(pin, GPIO_FUNC_PWM);

        // set the clkdiv mode (this might not actually do anything)
        pwm_set_clkdiv_mode(slicenum, PWM_DIV_FREE_RUNNING);

        // disable phase correct (if enabled, frequency is halved and duty cycle is doubled)
        pwm_set_phase_correct(slicenum, false);

        // set clkdiv to system clock in Mhz (125Mhz clock = clkdiv of 125)
        // makes it so that our on/off threshold on each pwm channel is equal to the on time per cycle in microseconds
        pwm_set_clkdiv(slicenum, divider);

        // set wrap to full scale for maximum resolution
        pwm_set_wrap(slicenum, max_duty);

        // set count
        pwm_set_gpio_level(pin, 0);

        // enable PWM slice
        pwm_set_enabled(slicenum, true);
        
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
      if(duty > max_duty) duty = max_duty;
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
    int bits;
    int max_duty;
    float min_us;
    float max_us;     
    int req_freq; //requested frequency
    int act_freq; //actual frequency
    float inv_duty_resolution_us;
};
