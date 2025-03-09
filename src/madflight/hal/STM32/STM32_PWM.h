// madflight https://github.com/qqqlab/madflight

//Note: this implementation does not check if frequency overwrites the frequency of previously started PWM instances

/*=============================================================================
Minimal Servo / PWM library for STM32

Example
-------
#include "STM32_PWM.h"

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

class PWM
{
  public:
    PWM() {};
        
    bool begin(int pin, int freq, float min_us, float max_us) {
        this->pin = pin;         
        this->freq = freq;
        this->min_us = min_us;
        this->max_us = max_us;

        // Automatically retrieve TIM instance and channel associated to pin
        // This is used to be compatible with all STM32 series automatically.
        Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pin), PinMap_PWM);
        channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin), PinMap_PWM));

        // Instantiate HardwareTimer object. 
        MyTim = new HardwareTimer(Instance);

        //set mode and period
        MyTim->setMode(channel, TIMER_OUTPUT_COMPARE_PWM1, pin); //PWM1: pin high when counter < channel compare, low otherwise
        MyTim->setOverflow(freq, HERTZ_FORMAT);
        
        //don't start output here, start output with first call to writeMicroseconds()
        //MyTim->setCaptureCompare(channel, min_us, MICROSEC_COMPARE_FORMAT);
        //MyTim->resume();

      return true;
    };

    void writeMicroseconds(uint32_t us) {
        if(!MyTim) return;
        if(us < min_us) us = min_us;
        if(us > max_us) us = max_us;
        MyTim->setCaptureCompare(channel, us, MICROSEC_COMPARE_FORMAT);
        MyTim->resume();
    };

    void writeFactor(float f) {
        writeMicroseconds(min_us + f * (max_us - min_us));
    }

  private:
    int pin;
    int freq;
    float min_us;
    float max_us; 
    TIM_TypeDef *Instance;
    uint32_t channel;
    HardwareTimer *MyTim = nullptr;
};
