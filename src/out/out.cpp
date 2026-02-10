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

char Out::get_type(uint8_t idx) {
  if(idx >= OUT_SIZE) return 0;
  return type[idx];
}

float Out::get(uint8_t idx) {//get last set value (might not be output because of armed == false)
  if(idx >= OUT_SIZE) return 0;
  return command[idx];
}

void Out::setup() {}

bool Out::_setup_output(uint8_t idx, char typ, int freq_hz, int pwm_min_us, int pwm_max_us){
  if(idx >= OUT_SIZE) return false;
  type[idx] = typ;
  command[idx] = 0.0f;
  eperiod[idx] = -1;
  eperiod_enabled[idx] = (type[idx] == 'B');
  switch(type[idx]) {
    case 'D':
    case 'B':
      //do nothing
      break;
    case 'M':
    case 'S':
      pwm[idx].begin(pins[idx], freq_hz, pwm_min_us, pwm_max_us);
      pwm[idx].writeFactor(command[idx]);
      break;
    default:
      return false;
  }
  return true;
}

bool Out::setup_dshot(uint8_t cnt, int* idxs, int freq_khz) {
  int dshot_pins[OUT_SIZE];
  for(int i = 0; i < cnt; i++) {
    if(!_setup_output(idxs[i], 'D', 0, 0, 0)) return false;
    dshot_idxs[i] = idxs[i];
    dshot_pins[i] = pins[ idxs[i] ];
  }
  if(!dshot.setup(dshot_pins, cnt, freq_khz)) return false;
  dshot_cnt = cnt;
  return true; 
}

bool Out::setup_dshot_bidir(uint8_t cnt, int* idxs, int freq_khz) {
  int dshot_pins[cnt];
  for(int i = 0; i < cnt; i++) {
    if(!_setup_output(idxs[i], 'B', 0, 0, 0)) return false;
    dshot_idxs[i] = idxs[i];
    dshot_pins[i] = pins[ idxs[i] ];
  }
  if(!dshotbidir.setup(dshot_pins, cnt, freq_khz)) return false;
  dshot_cnt = cnt;
  return true; 
}

bool Out::setup_motors(uint8_t cnt, int* idxs, int freq_hz, int pwm_min_us, int pwm_max_us) {
  for(int i = 0; i < cnt; i++) {
    if(!setup_motor(idxs[i], freq_hz, pwm_min_us, pwm_max_us)) return false;
  }
  return true;
}

bool Out::setup_motor(uint8_t idx, int freq_hz, int pwm_min_us, int pwm_max_us) {
  return _setup_output(idx, 'M', freq_hz, pwm_min_us, pwm_max_us);
}

bool Out::setup_servo(uint8_t idx, int freq_hz, int pwm_min_us, int pwm_max_us) {
  return _setup_output(idx, 'S', freq_hz, pwm_min_us, pwm_max_us);
}

void Out::set(uint8_t idx, float value) {
  if(idx >= OUT_SIZE) return;

  //do nothing in testmode
  if(_mode == mode_enum::TESTMODE) return;

  // handle disarmed
  if(_mode == mode_enum::DISARMED) {
    stop_all_motors();
    return;
  }

  _set(idx, value);
  _watchdog_ts = micros(); //reset watchdog timer
}

//unconditional set - no armed check
void Out::_set(uint8_t idx, float value) {
  bool updated = true;
  command[idx] = value;
  switch(type[idx]) {
    case 'D':
    case 'B':
      //only send throttle values on last mot index
      if(idx == dshot_idxs[dshot_cnt - 1]) {
        uint16_t throttle[OUT_SIZE];
        for(int i = 0; i < dshot_cnt; i++) {
          float v = constrain(command[ dshot_idxs[i] ], 0.0f, 1.0f);
          throttle[i] = v * 2000;
        }
        if(type[idx] == 'D') {
          dshot.set_throttle(throttle);
        }else{
          dshotbidir.get_eperiod(eperiod);
          dshotbidir.set_throttle(throttle);
        }
      }
      break;
    case 'M':
    case 'S':
      pwm[idx].writeFactor(value);
      break;
    default:
      updated = false;
  }

  if(updated) {
    ts = micros();
    topic.publish(this);
    bbx.log_out();
  }
}

int Out::rpm(uint8_t idx, int poles) {
  if(idx >= OUT_SIZE) return -1;
  if(eperiod[idx] < 0) return eperiod[idx];
  if(eperiod[idx] == 0) return 0;
  return 120000000 / poles / eperiod[idx];
}

bool Out::update() {
  //disarm motors after timeout
  if(_mode != mode_enum::DISARMED && micros() - _watchdog_ts >= OUT_MOT_TIMEOUT) {
    _mode = mode_enum::DISARMED;
    //TODO - publish disarm
  }

  //stop motors when disarmed
  if(_mode == mode_enum::DISARMED) {
    stop_all_motors();
    return false;
  }

  //keep dshot alive even if no set() was called
  if(micros() - _update_ts >= 10000) {
    for(int i = 0; i < OUT_SIZE; i++) {
      if(is_motor(i)) _set(i, command[i]); //all motors to current value
    }
    _update_ts = micros();
  }

  return true;
}

bool Out::armed() {
  return (_mode == mode_enum::ARMED);
}

void Out::set_armed(bool set_armed) {
  if(set_armed && _mode == mode_enum::DISARMED) {
    _mode == mode_enum::ARMED;
    //TODO: publish changed armed status
  }else if(!set_armed && _mode == mode_enum::ARMED) {
    _mode == mode_enum::DISARMED;
    //TODO: publish changed armed status
  }
}

//enable testmode
void Out::testmode_enable(bool set_testmode) {
  if(set_testmode && _mode != mode_enum::TESTMODE) {
    _mode = mode_enum::TESTMODE;
    stop_all_motors(); //stop motors on changing to/from testmode
  }else if(!set_testmode && _mode == mode_enum::TESTMODE) {
    _mode = mode_enum::DISARMED;
    stop_all_motors(); //stop motors on changing to/from testmode
  }
  _watchdog_ts = micros(); //reset watchdog timer
}

void Out::testmode_set(uint8_t idx, float value) {
  if(_mode != mode_enum::TESTMODE) return;
  if(idx >= OUT_SIZE) return;
  //Serial1.printf("testmode_set %d %f tm=%d\n",idx,value,_testmode);
  _set(idx, value);
  _watchdog_ts = micros(); //reset watchdog timer
}

bool Out::is_motor(uint8_t idx) {
  if(idx >= OUT_SIZE) return false;
  return (type[idx] == 'M' || type[idx] == 'D' || type[idx] == 'B');
}

bool Out::is_servo(uint8_t idx) {
  if(idx >= OUT_SIZE) return false;
  return (type[idx] == 'S');
}

//unconditionally stop all motors
void Out::stop_all_motors() {
  for(int i = 0; i < OUT_SIZE; i++) {
    if(is_motor(i)) _set(i, 0);
  }
}

int8_t Out::get_pin(uint8_t idx) {
  if(idx >= OUT_SIZE) return -1;
  return pins[idx];
}

void Out::set_pin(uint8_t idx, int pin) {
  if(idx >= OUT_SIZE) return;
  pins[idx] = pin;
}
