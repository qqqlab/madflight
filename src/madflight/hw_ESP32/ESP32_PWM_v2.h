/*=============================================================================
Minimal Servo / PWM library for ESP32

Example
-------
#include "ESP32_PWM.h"

PWM motor;
PWM servo;

float motor_pwm = 125;
float servo_pwm = 1000;

void setup() {
  motor.begin(12,2000,125,250); //pin 12: Oneshot motor ESC 2000Hz pulse 125-250 us
  servo.begin(22,50,1000,2000); //pin 22: regular servo 50Hz pulse 1000-2000 us 
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

#if ESP_ARDUINO_VERSION_MAJOR > 2
  #error ESP32_PWM_v2.h needs Arduino-ESP32 version 2 (not a later version), please use Boards Manager to update "esp32"
#endif  

//Maximum number of PWM outputs - NOTE: some ESP32 chips have less than this
#define PWM_MAX 16

#include "esp32-hal-ledc.h"
#include "Arduino.h"
#include <driver/ledc.h> //defines LEDC_TIMER_BIT_MAX

class PWM
{
  public:
    PWM() {};
    bool begin(int pin, int freq, float min_us, float max_us) {
      this->req_freq = freq;
      this->pin = pin; 
      this->min_us = min_us;
      this->max_us = max_us;
      this->ch = -1;
      this->bits = 0;
      this->act_freq = 0;
      this->max_duty = 0;
      this->inv_duty_resolution_us = 1;

      //exit if no free channel
      int ch = findFreeChannel(freq);
      if(ch<0) return false;

      //find maximum number of bits, or exit if less than 7
      int act_freq;
      int bits = LEDC_TIMER_BIT_MAX;
      while(1) {
        act_freq = ledcSetup(ch, freq, bits);
        if(act_freq > 0) break;
        bits--;
        if(bits < 7) return false;
      }

      ledcWrite(ch, 0); //start with no output
      ledcAttachPin(pin, ch);      

      this->ch = ch;
      this->bits = bits;
      this->act_freq = act_freq;
      this->max_duty =  (1<<bits) - 1;
      this->inv_duty_resolution_us = 1.0e-6 * act_freq * (max_duty+1);
      channels[ch] = this;

      return true;
    };
    
    void writeMicroseconds(float us) {
      if(us < min_us) us = min_us;
      if(us > max_us) us = max_us;
      int duty = us * inv_duty_resolution_us;
      if(duty < 0) duty = 0;
      if(duty > max_duty) duty = max_duty;
      ledcWrite(ch, us * inv_duty_resolution_us);
    };
    
    void writeFactor(float f) {
      writeMicroseconds(min_us + f * (max_us - min_us));
    }
    
    int get_ch() {return ch;}
    float get_min_us() {return min_us;}
    float get_max_us() {return max_us;}
    float get_duty_resolution_us() {return 1/inv_duty_resolution_us;}
    int get_req_freq() {return req_freq;}
    int get_act_freq() {return act_freq;}

  private:
    static PWM *channels[PWM_MAX];
    int pin;
    int ch;
    int bits;
    int max_duty;
    float min_us;
    float max_us;     
    int req_freq; //requested frequency
    int act_freq; //actual frequency
    float inv_duty_resolution_us;
    static int findFreeChannel(int freq);
};


PWM *PWM::channels[] = {0};


//two channels share the same timer - have also the same freq 
//try first to find a free channel with matching freq
//if not found, then find first free timer
int PWM::findFreeChannel(int freq)
{
  //find free channel with other channel in same group with same req_freq
  for(int i=0;i<PWM_MAX;i++){
    if(!channels[i]) {
      int other_i = (i % 2 == 0 ? i+1 : i-1);
      PWM *other_ch = channels[other_i];
      if(other_ch && other_ch->req_freq == freq) {
        return i;
      }
    }
  }
  //no free channel with matching freq found -> find first free group
  for(int i=0;i<PWM_MAX;i++){
    if(!channels[i]) {
      int other_i = (i % 2 == 0 ? i+1 : i-1);
      PWM *other_ch = channels[other_i];
      if(!other_ch) {
        return i;
      }
    }
  }      
  return -1;
} 
