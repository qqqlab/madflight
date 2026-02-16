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

Out::type_enum Out::type(uint8_t idx) {
  if(idx >= OUT_SIZE) return type_enum::UNUSED;
  return _type[idx];
}

float Out::get_output(uint8_t idx) {//get last set value (might not be output because of armed == false)
  if(idx >= OUT_SIZE) return 0;
  return command[idx];
}

void Out::setup() {}

bool Out::_setup_output(uint8_t idx, type_enum typ, float freq_hz, float pwm_min_us, float pwm_max_us){
  if(idx >= OUT_SIZE) return false;
  if(publish_trigger_idx <= idx) publish_trigger_idx = idx;
  _type[idx] = typ;
  command[idx] = 0.0f;
  eperiod[idx] = -1;
  switch(_type[idx]) {
    case type_enum::DSHOT:
    case type_enum::DSHOTBIDIR:
      //do nothing
      break;
    case type_enum::MOTPWM:
    case type_enum::BRUSHED:
    case type_enum::SERVO:
      _pwm[idx].begin(_pin[idx], freq_hz, pwm_min_us, pwm_max_us);
      _pwm[idx].writeFactor(command[idx]);
      break;
    default:
      return false;
  }
  return true;
}

bool Out::setup_dshot(uint8_t cnt, int* idxs, int freq_khz) {
  int dshot_pins[OUT_SIZE];
  for(int i = 0; i < cnt; i++) {
    if(!_setup_output(idxs[i], type_enum::DSHOT, 0, 0, 0)) return false;
    _dshot_idxs[i] = idxs[i];
    dshot_pins[i] = _pin[ idxs[i] ];
  }
  if(!_dshot.setup(dshot_pins, cnt, freq_khz)) return false;
  _dshot_cnt = cnt;
  return true; 
}

bool Out::setup_dshot_bidir(uint8_t cnt, int* idxs, int freq_khz) {
  int dshot_pins[cnt];
  for(int i = 0; i < cnt; i++) {
    if(!_setup_output(idxs[i], type_enum::DSHOTBIDIR, 0, 0, 0)) return false;
    _dshot_idxs[i] = idxs[i];
    dshot_pins[i] = _pin[ idxs[i] ];
  }
  if(!_dshotbidir.setup(dshot_pins, cnt, freq_khz)) return false;
  _dshot_cnt = cnt;
  return true; 
}

bool Out::setup_brushed(uint8_t cnt, int* idxs, int freq_hz) {
  for(int i = 0; i < cnt; i++) {  
    if(!_setup_output(idxs[i], type_enum::BRUSHED, freq_hz, 0, 1e6f / freq_hz)) return false;
  }
  return true;
}

bool Out::setup_motors(uint8_t cnt, int* idxs, int freq_hz, int pwm_min_us, int pwm_max_us) {
  for(int i = 0; i < cnt; i++) {
    if(!setup_motor(idxs[i], freq_hz, pwm_min_us, pwm_max_us)) return false;
  }
  return true;
}

bool Out::setup_motor(uint8_t idx, int freq_hz, int pwm_min_us, int pwm_max_us) {
  return _setup_output(idx, type_enum::MOTPWM, freq_hz, pwm_min_us, pwm_max_us);
}

bool Out::setup_servo(uint8_t idx, int freq_hz, int pwm_min_us, int pwm_max_us) {
  return _setup_output(idx, type_enum::SERVO, freq_hz, pwm_min_us, pwm_max_us);
}

void Out::set_output(uint8_t idx, float value) {
  if(idx >= OUT_SIZE) return;

  //do nothing in testmode
  if(_mode == mode_enum::TESTMOTOR) return;

  //set the output
  _set_output(idx, value);

  //report it
  if(idx == publish_trigger_idx) {
    topic.publish(this);
  }

  //reset watchdog timer
  _watchdog_ts = micros(); 
}

//unconditional set - no armed check
void Out::_set_output(uint8_t idx, float value) {
  command[idx] = value;
  switch(_type[idx]) {
    case type_enum::DSHOT:
    case type_enum::DSHOTBIDIR:
      //only send throttle values on last mot index
      if(idx == _dshot_idxs[_dshot_cnt - 1]) {
        uint16_t throttle[OUT_SIZE];
        for(int i = 0; i < _dshot_cnt; i++) {
          float v = constrain(command[ _dshot_idxs[i] ], 0.0f, 1.0f);
          throttle[i] = v * 2000;
        }
        if(_type[idx] == type_enum::DSHOT) {
          _dshot.set_throttle(throttle);
        }else{
          _dshotbidir.get_eperiod(eperiod);
          _dshotbidir.set_throttle(throttle);
        }
      }
      break;
    case type_enum::MOTPWM:
    case type_enum::BRUSHED:
    case type_enum::SERVO:
      _pwm[idx].writeFactor(value);
      break;
    default:
      return;
  }

  //record update timestamp
  ts = micros();
}

int Out::rpm(uint8_t idx, int poles) {
  if(idx >= OUT_SIZE) return -1;
  if(_type[idx] != type_enum::DSHOTBIDIR) return -1;
  return eperiod_to_rpm(eperiod[idx], poles);
}

int Out::eperiod_to_rpm(int eperiod, int poles) {
  if(eperiod <= 0) return eperiod;
  return 120000000 / poles / eperiod;
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
      if(is_motor(i)) _set_output(i, command[i]); //all motors to current value
    }
    _update_ts = micros();
  }

  return true;
}

//returns true on changed
bool Out::_set_mode(mode_enum mode_new) {
  if(_mode == mode_new) return false;
  //stop motors on disarm, or switch to/from testmotor
  if(mode_new == mode_enum::DISARMED || mode_new == mode_enum::TESTMOTOR || _mode == mode_enum::TESTMOTOR) {
    stop_all_motors();
  }
  //TODO: publish mode change
  _mode = mode_new;
  return true;
}

bool Out::armed() {
  return (_mode == mode_enum::ARMED);
}

void Out::emergency_stop() {
  _set_mode(mode_enum::DISARMED);
  stop_all_motors(); //execute regardless of current _mode
}

void Out::set_armed(bool set_armed) {
  if(set_armed && _mode == mode_enum::DISARMED) {
    _set_mode(mode_enum::ARMED);
  }else if(!set_armed && _mode == mode_enum::ARMED) {
    _set_mode(mode_enum::DISARMED);
  }
}

//enable testmode
void Out::testmotor_enable(bool set_testmode) {
  _set_mode( set_testmode ? mode_enum::TESTMOTOR : mode_enum::DISARMED);
  _watchdog_ts = micros(); //reset watchdog timer
}

void Out::testmotor_set_output(uint8_t idx, float value) {
  if(_mode != mode_enum::TESTMOTOR) return;
  if(idx >= OUT_SIZE) return;
  _set_output(idx, value);
  _watchdog_ts = micros(); //reset watchdog timer
}

bool Out::is_motor(uint8_t idx) {
  if(idx >= OUT_SIZE) return false;
  return (_type[idx] == type_enum::MOTPWM || _type[idx] == type_enum::DSHOT || _type[idx] == type_enum::DSHOTBIDIR || _type[idx] == type_enum::BRUSHED);
}

bool Out::is_servo(uint8_t idx) {
  if(idx >= OUT_SIZE) return false;
  return (_type[idx] == type_enum::SERVO);
}

//unconditionally stop all motors
void Out::stop_all_motors() {
  for(int i = 0; i < OUT_SIZE; i++) {
    if(is_motor(i)) _set_output(i, 0);
  }
}

int8_t Out::pin(uint8_t idx) {
  if(idx >= OUT_SIZE) return -1;
  return _pin[idx];
}

void Out::set_pin(uint8_t idx, int pin) {
  if(idx >= OUT_SIZE) return;
  _pin[idx] = pin;
}

const char* Out::get_mode_string() {
  switch(_mode) {
    case mode_enum::DISARMED: return "DISARMED";
    case mode_enum::ARMED: return"ARMED";
    case mode_enum::TESTMOTOR: return "TESTMODE";
    default: return "UNKNOWN";
  }
}

void Out::print(Print &p) {
  p.printf("OUT: idx:gpio:type = ");
  for(int i = 0; i < OUT_SIZE; i++) {
    switch(_type[i]) {
      case type_enum::DSHOT:      p.printf("%d:G%d:Dshot ",i,_pin[i]); break;
      case type_enum::DSHOTBIDIR: p.printf("%d:G%d:DshotBidir ",i,_pin[i]); break;
      case type_enum::MOTPWM:     p.printf("%d:G%d:PWM ",i,_pin[i]); break;
      case type_enum::BRUSHED:    p.printf("%d:G%d:Brushed ",i,_pin[i]); break;
      case type_enum::SERVO:      p.printf("%d:G%d:Servo ",i,_pin[i]); break;
      case type_enum::UNUSED: if(_pin[i]>=0) p.printf("%d:G%d:Unused ",i,_pin[i]);
    }
  }
  p.println();
}
