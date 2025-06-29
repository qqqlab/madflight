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

#define MF_MOD "RCL"

#include "rcl.h"
#include "../hal/hal.h"
#include "../cfg/cfg.h"

//the gizmos
#include "RclGizmoMavlink.h"
#include "RclGizmoCrsf.h"
#include "RclGizmoSbus.h" //TODO need SERIAL_8E2
#include "RclGizmoDsm.h"
#include "RclGizmoPpm.h"
#include "RclGizmoIbus.h"
//#include "RclGizmoPwm.h" //not implemented

//set defaults
#ifndef RCL_TIMEOUT
  #define RCL_TIMEOUT 3000 // lost connection timeout in milliseconds
#endif
#ifndef RCL_THROTTLE_DEADBAND
  #define RCL_THROTTLE_DEADBAND 60 //pwm deadband for zero throttle in microseconds
#endif

//create global Rcl class instance
Rcl rcl;

int Rcl::setup() {
  cfg.printModule("rcl");

  //clear vars
  throttle = 0;
  roll = 0;
  pitch = 0;
  yaw = 0;
  vspeed = 0;
  armed = false;
  _arm_sw_prev = true; //default to true, to require arm switch first off, then on to enter armed state!

  //create gizmo
  delete gizmo;
  switch(config.gizmo) {

    case Cfg::rcl_gizmo_enum::mf_NONE : {
      gizmo = nullptr;
      break;
    }

    case Cfg::rcl_gizmo_enum::mf_MAVLINK : {
      MF_Serial* ser_bus = hal_get_ser_bus(config.ser_bus_id);
      if(!ser_bus) {
        Serial.println("\n" MF_MOD ": ERROR Serial bus not connected, check pins.\n");
        return -1002;
      }
      gizmo = new RclGizmoMavlink(ser_bus, config.baud, pwm);
      break;
    }

    case Cfg::rcl_gizmo_enum::mf_CRSF : {
      MF_Serial* ser_bus = hal_get_ser_bus(config.ser_bus_id);
      if(!ser_bus) {
        Serial.println("\n" MF_MOD ": ERROR Serial bus not connected, check pins.\n");
        return -1002;
      }
      gizmo = new RclGizmoCrsf(ser_bus, config.baud, pwm);
      break;
    }

    case Cfg::rcl_gizmo_enum::mf_SBUS : {
      gizmo = RclGizmoSbus::create(config.ser_bus_id, pwm, config.baud, true);
      break;
    }

    case Cfg::rcl_gizmo_enum::mf_SBUS_NOT_INV : {
      gizmo = RclGizmoSbus::create(config.ser_bus_id, pwm, config.baud, false);
      break;
    }

    case Cfg::rcl_gizmo_enum::mf_DSM : {
      MF_Serial* ser_bus = hal_get_ser_bus(config.ser_bus_id);
      if(!ser_bus) {
        Serial.println("\n" MF_MOD ": ERROR Serial bus not connected, check pins.\n");
        return -1002;
      }
      gizmo = new RclGizmoDsm(ser_bus, config.baud, pwm);
      break;
    }

    case Cfg::rcl_gizmo_enum::mf_PPM : {
      gizmo = new RclGizmoPpm(config.ppm_pin, pwm);
      break;
    }

    case Cfg::rcl_gizmo_enum::mf_IBUS : {
      gizmo = RclGizmoIbus::create(config.ser_bus_id, pwm, config.baud);
      if (!gizmo) {
        Serial.println("\n" MF_MOD ": ERROR Serial bus not connected, check pins.\n");
      }

      break;
    }
  }

  //check gizmo
  if(!installed() && config.gizmo != Cfg::rcl_gizmo_enum::mf_NONE) {
    Serial.println(MF_MOD ": ERROR check pin/bus config");
    return -1001;
  }

  pwm[RCL_MAX_CH] = 1500; //set 'unassigned' pwm value

  //setup stick/switch parameters from config values
  _setupStick(THR, cfg.rcl_thr_ch, cfg.rcl_thr_pull, cfg.rcl_thr_mid, cfg.rcl_thr_push);
  _setupStick(ROL, cfg.rcl_rol_ch, cfg.rcl_rol_left, cfg.rcl_rol_mid, cfg.rcl_rol_right);
  _setupStick(PIT, cfg.rcl_pit_ch, cfg.rcl_pit_pull, cfg.rcl_pit_mid, cfg.rcl_pit_push);
  _setupStick(YAW, cfg.rcl_yaw_ch, cfg.rcl_yaw_left, cfg.rcl_yaw_mid, cfg.rcl_yaw_right);
  st[ARM].ch  = _getCh(cfg.rcl_arm_ch);
  st[ARM].min = cfg.rcl_arm_min;
  st[ARM].mid = 0; //NOT USED
  st[ARM].max = cfg.rcl_arm_max;
  st[ARM].inv = 0; //NOT USED
  st[FLT].ch  = _getCh(cfg.rcl_flt_ch);
  st[FLT].min = cfg.rcl_flt_min;
  st[FLT].mid = 0; //NOT USED
  st[FLT].max = 0; //NOT USED
  st[FLT].inv = 0; //NOT USED
  st_flt_spacing = (cfg.rcl_flt_max - cfg.rcl_flt_min) / 5;

  Serial.printf(MF_MOD ": Setup completed.  Channels: throttle:%d roll:%d pitch:%d yaw:%d armed:%d flightmode:%d\n"
  , (int)cfg.rcl_thr_ch, (int)cfg.rcl_rol_ch, (int)cfg.rcl_pit_ch, (int)cfg.rcl_yaw_ch, (int)cfg.rcl_arm_ch, (int)cfg.rcl_flt_ch);
  return 0;
}


bool Rcl::update() { //returns true if channel pwm data was updated
  if(!gizmo) return false;
  bool rv = gizmo->update();

  if(rv) {
    //throttle: 0.0 in range from stick full back to rcl_cfg_thro_low, 1.0 on full throttle
    float tlow = st[THR].min + RCL_THROTTLE_DEADBAND;
    throttle = constrain( ((float)(pwm[st[THR].ch] - tlow)) / (st[THR].max - tlow), 0.0, 1.0);

    //roll,pitch,yaw
    vspeed =  st[THR].inv * _ChannelNormalize(pwm[st[THR].ch], st[THR].min, st[THR].mid, st[THR].max, cfg.rcl_deadband); // output: -1 (descent, st back) to 1 (ascent, st forward)
    roll   =  st[ROL].inv * _ChannelNormalize(pwm[st[ROL].ch], st[ROL].min, st[ROL].mid, st[ROL].max, cfg.rcl_deadband); // output: -1 (roll left, st left) to 1 (roll right, st right)
    pitch  = -st[PIT].inv * _ChannelNormalize(pwm[st[PIT].ch], st[PIT].min, st[PIT].mid, st[PIT].max, cfg.rcl_deadband); // output: -1 (pitch down, st back) to 1 (pitch up, st forward)
    yaw    =  st[YAW].inv * _ChannelNormalize(pwm[st[YAW].ch], st[YAW].min, st[YAW].mid, st[YAW].max, cfg.rcl_deadband); // output: -1 (yaw left, st left) to 1 (yaw right, st right)

    //armed state
    if(st[ARM].ch < RCL_MAX_CH) {
      //use arm switch
      bool arm_sw = (st[ARM].min <= pwm[st[ARM].ch] && pwm[st[ARM].ch] < st[ARM].max); //get arm switch state
      if(throttle == 0 && arm_sw && !_arm_sw_prev) {
        //Arm: Throttle is zero and radio armed switch was flipped from disarmed to armed position
        armed = true;
      }else if (!arm_sw) {
        //Disarm: As soon as arm switch is in disarmed position. "Kill switch"
        armed = false;
      }
      _arm_sw_prev = arm_sw;
    }else{
      //use stick commands (no arm switch defined)
      if(!armed && throttle == 0.0 && pitch > 0.9 && yaw > 0.9 && roll < -0.9) {
        //Arm: Pull both sticks toward you, yaw full right, and roll full left and keep then there for 2 sec
        if(_arm_ts == 0) {
          _arm_ts = millis();
        }else if(millis() - _arm_ts > 2000) {
          armed = true;
        }
      }else if(armed && throttle == 0.0 && pitch > 0.9 && yaw < -0.9 && roll > 0.9) {
        //Disarm: Pull both sticks toward you, yaw full left, and roll full right and keep then there for 2 sec
        if(_arm_ts == 0) {
          _arm_ts = millis();
        }else if(millis() - _arm_ts > 2000) {
          armed = false;
        }
      }else {
        _arm_ts = 0;
      }
    }

    //flightmode 6 position switch
    if(st_flt_spacing<5) {
      flightmode = 0;
    }else{
      flightmode = constrain( ( pwm[st[FLT].ch] - st[FLT].min + st_flt_spacing/2) / st_flt_spacing, 0, 5); //output 0..5
    }

    //set update timestamp
    update_time = millis();
  }
  return rv;
}

bool Rcl::connected() {
  return ((uint32_t)millis() - update_time <= (RCL_TIMEOUT) );
}

//helper to nomalize a channel based on min,center,max calibration
float Rcl::_ChannelNormalize(int val, int min, int center, int max, int deadband) {
  int rev = 1; //1=normal, -1=reverse channel
  //needs: min < center < max
  if(val<min) return rev * -1.0;
  if(val<center-deadband) return (float)(rev * (val-(center-deadband))) / ((center-deadband)-min); //returns -1 to 0
  if(val<center+deadband) return 0;
  if(val<max) return (float)(rev * (val-(center+deadband))) / (max-(center+deadband));
  return rev * 1.0;
}

//normalize 1-based parameter channel to 0-RCL_MAX_CH, where RCL_MAX_CH is used for invalid channels
int Rcl::_getCh(int ch) {
  if(ch >= 1 && ch <= RCL_MAX_CH) return ch - 1; else return RCL_MAX_CH;
}

void Rcl::_setupStick(int stickno, int ch, int left_pull, int mid, int right_push) {
  st[stickno].ch  = _getCh(ch);
  st[stickno].mid = mid;
  if(left_pull < right_push) {
    st[stickno].min = left_pull;
    st[stickno].max = right_push;
    st[stickno].inv = 1;
  }else{
    st[stickno].min = right_push;
    st[stickno].max = left_pull;
    st[stickno].inv = -1;
  }
}
