#pragma once

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
};

extern Out out;
