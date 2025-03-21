//needs to be header because of "cfg." usage in this file

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
  Cfg::printModule("rcl");

  //clear vars
  throttle = 0;
  roll = 0;
  pitch = 0;
  yaw = 0;
  vspeed = 0;
  arm = false;

  if(!config.ser_bus && config.gizmo != Cfg::rcl_gizmo_enum::mf_PPM && config.gizmo != Cfg::rcl_gizmo_enum::mf_NONE) {
    Serial.println("\n" MF_MOD ": ERROR Serial bus not connected, check pins.\n");
    return -1002;
  }

  //create gizmo
  delete gizmo;
  switch(config.gizmo) {
    case Cfg::rcl_gizmo_enum::mf_NONE :
      gizmo = nullptr;
      break;      
    case Cfg::rcl_gizmo_enum::mf_MAVLINK :
      if(config.ser_bus) {
        gizmo = new RclGizmoMavlink(config.ser_bus, config.baud, pwm);
      }
      break;
    case Cfg::rcl_gizmo_enum::mf_CRSF :
      if(config.ser_bus) {
        gizmo = new RclGizmoCrsf(config.ser_bus, config.baud, pwm);
      }
      break;
    case Cfg::rcl_gizmo_enum::mf_SBUS :
      if(config.ser_bus) {
        gizmo = new RclGizmoSbus(config.ser_bus, pwm); //baud is fixed 100000
        Serial.println("\n" MF_MOD ": ERROR SBUS not implemented yet. Please create an issue on github to get it implemented!\n");
      }
      break;
    case Cfg::rcl_gizmo_enum::mf_DSM :
      if(config.ser_bus) {
        gizmo = new RclGizmoDsm(config.ser_bus, config.baud, pwm);
      }
      break;
    case Cfg::rcl_gizmo_enum::mf_PPM :
      gizmo = new RclGizmoPpm(config.ppm_pin, pwm);
      break;
    }
    
  //check gizmo
  if(!installed() && config.gizmo != Cfg::rcl_gizmo_enum::mf_NONE) {
    Serial.println(MF_MOD ": ERROR check pin/bus config");
    return -1001;
  }

  //setup stick/switch parameters from config values
  Serial.printf(MF_MOD ": Channels - throttle:%d roll:%d pitch:%d yaw:%d armed:%d flightmode:%d\n", (int)cfg.rcl_thr_ch, (int)cfg.rcl_rol_ch, (int)cfg.rcl_pit_ch, (int)cfg.rcl_yaw_ch, (int)cfg.rcl_arm_ch, (int)cfg.rcl_flt_ch);
  _setupStick(THR, cfg.rcl_thr_ch, cfg.rcl_thr_pull, cfg.rcl_thr_mid, cfg.rcl_thr_push);
  _setupStick(ROL, cfg.rcl_rol_ch, cfg.rcl_rol_left, cfg.rcl_rol_mid, cfg.rcl_rol_right);
  _setupStick(PIT, cfg.rcl_pit_ch, cfg.rcl_pit_pull, cfg.rcl_pit_mid, cfg.rcl_pit_push);
  _setupStick(YAW, cfg.rcl_yaw_ch, cfg.rcl_yaw_left, cfg.rcl_yaw_mid, cfg.rcl_yaw_right);
  st[ARM].ch  = cfg.rcl_arm_ch - 1;
  st[ARM].min = cfg.rcl_arm_min;
  st[ARM].mid = 0; //NOT USED
  st[ARM].max = cfg.rcl_arm_max;
  st[ARM].inv = 0; //NOT USED
  st[FLT].ch  = cfg.rcl_flt_ch - 1;
  st[FLT].min = cfg.rcl_flt_min;
  st[FLT].mid = 0; //NOT USED
  st[FLT].max = 0; //NOT USED
  st[FLT].inv = 0; //NOT USED
  st_flt_spacing = (cfg.rcl_flt_max - cfg.rcl_flt_min) / 5;

  //check st parameters
  for(int i=0;i<6;i++) {
    if(st[i].ch >= cfg.rcl_num_ch) {
      Serial.println(MF_MOD ": ERROR invalid channel in config, re-calibrate radio");
      st[i].ch  = 0;
      st[i].min = 1;
      st[i].mid = 2;
      st[i].max = 3;
      st[i].inv = 1;
    }
  }

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

    //arm switch
    arm = (st[ARM].min <= pwm[st[ARM].ch] && pwm[st[ARM].ch] < st[ARM].max);

    //flightmode 6 position switch
    flightmode = constrain( ( pwm[st[FLT].ch] - st[FLT].min + st_flt_spacing/2) / st_flt_spacing, 0, 5); //output 0..5

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

void Rcl::_setupStick(int stickno, int ch, int left_pull, int mid, int right_push) {
  st[stickno].ch  = ch-1;
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

#undef MF_MOD
