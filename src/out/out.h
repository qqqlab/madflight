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
#define OUT_MOT_TIMEOUT 250000 //switch motors off 250ms after last set received

#include "../hal/hal.h"
#include "../tbx/MsgBroker.h"

struct __attribute__((aligned(4))) OutState {
  public:
    uint32_t ts = 0; //time of last set() or testmode_enable() call
    float command[OUT_SIZE] = {}; //last commanded outputs (values: 0.0 to 1.0)
    int eperiod[OUT_SIZE] = {}; //ePeriod in [us], 0 when motor stopped, negative on error
};

class Out : public OutState {
  public:
    MsgTopic<OutState> topic = MsgTopic<OutState>("out");

    enum type_enum {
      UNUSED     = 0,
      BRUSHED    = 'm', //use char for easy printing of type
      MOTPWM     = 'M',
      DSHOT      = 'D',
      DSHOTBIDIR = 'd',
      SERVO      = 'S'
    };

    enum mode_enum {
      DISARMED = 0,
      ARMED = 1,
      TESTMOTOR = 2
    } _mode = DISARMED;

    int8_t publish_trigger_idx = -1; //this index triggers a publish in set_output() - updated by setup_xxx()

    void setup();
    bool update();

    mode_enum mode() {return _mode;}
    bool armed();
    void set_armed(bool set_armed);
    void testmotor_enable(bool set_testmode);
    void testmotor_set_output(uint8_t idx, float value);
    bool is_motor(uint8_t idx);
    bool is_servo(uint8_t idx);
    void stop_all_motors(); //unconditionally stop all motors, but does not change mode
    void emergency_stop(); //unconditionally disarm
    bool setup_dshot(uint8_t cnt, int* idxs, int freq_khz = 300);
    bool setup_dshot_bidir(uint8_t cnt, int* idxs, int freq_khz = 300);
    bool setup_motors(uint8_t cnt, int* idxs, int freq_hz = 400, int pwm_min_us = 950, int pwm_max_us = 2000);
    bool setup_brushed(uint8_t cnt, int* idxs, int freq_khz);
    bool setup_motor(uint8_t idx, int freq_hz = 400, int pwm_min_us = 950, int pwm_max_us = 2000);
    bool setup_servo(uint8_t idx, int freq_hz = 400, int pwm_min_us = 950, int pwm_max_us = 2000);
    void set_output(uint8_t idx, float value); //set output (when ARMED, ignored in DISARMED and TESTMOTOR mode)
    float get_output(uint8_t idx); //get last output value
    type_enum type(uint8_t idx);
    int rpm(uint8_t idx, int poles = 14); //get RPM, returns -1 if not available
    static int eperiod_to_rpm(int eperiod, int poles = 14);
    int8_t pin(uint8_t idx);
    void set_pin(uint8_t idx, int pin);
    const char* get_mode_string();
    void print(Print &p = Serial);

  private:


    bool _setup_output(uint8_t idx, type_enum typ, float freq_hz, float pwm_min_us, float pwm_max_us);
    void _set_output(uint8_t idx, float value);  //unconditional set output - no armed check

    bool _set_mode(mode_enum mode); // returns true on changed

    type_enum _type[OUT_SIZE] = {};
    int8_t _pin[OUT_SIZE] = {};

    //PWM
    PWM _pwm[OUT_SIZE]; //ESC and Servo outputs (values: 0.0 to 1.0)

    //Dshot
    Dshot _dshot;
    DshotBidir _dshotbidir;
    int _dshot_cnt = 0; //number of dshot pins
    int _dshot_idxs[OUT_SIZE]; //dshot out indices

    uint32_t _update_ts = 0;
    uint32_t _watchdog_ts = 0;


};

extern Out out;
