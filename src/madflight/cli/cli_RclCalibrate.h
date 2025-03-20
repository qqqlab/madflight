#pragma once

#include "../hal/hal.h"
#include "../imu/imu.h"

//this class assumes rcl.update() is called in the background... (i.e. in imu_loop())

void rcl_cal_imu_loop() {
   rcl.update();
}

class RclCalibrate {

public:
  static void calibrate() {
    imu.onUpdate = rcl_cal_imu_loop;
    RclCalibrate *cal = new RclCalibrate();
    cal->_calibrate();
  }

  struct stick_t{
    uint8_t ch; //stick channel
    uint16_t left_pull; //spwm at left/pull position
    uint16_t mid; //pwm at center position
    uint16_t right_push; //pwm at right/push position
    uint16_t min;
    uint16_t max;
  };
  
private:
  stick_t thr; //throttle stick
  stick_t yaw; //yaw stick
  stick_t pit; //pitch stick
  stick_t rol; //roll stick
  stick_t arm; //arm switch
  stick_t flt; //flightmode switch

  bool ignore[RCL_MAX_CH] = {}; //channels to ignore
  int16_t pc[RCL_MAX_CH] = {}; //temp value
  int16_t arm_last; //temp value

void prompt(const char *msg) {
  Serial.println(msg);

  //empty buffer
  while(Serial.available()) Serial.read();

  //wait for enter
  for(;;) {
    if(Serial.available()) {
      char c = Serial.read();
      if ( c=='\r' || c=='\n' ) return;
    }
  }
}

void printpwm() {
  for(int i=0;i<cfg.rcl_num_ch;i++) {
    Serial.printf("ch%d:%4d ", i, rcl.pwm[i]);
  }
  Serial.println();
}

void printpwm(const char *msg, int16_t *p) {
  Serial.printf(msg);
  for(int i=0;i<cfg.rcl_num_ch;i++) {
    Serial.printf(" ch%d:%4d", i, p[i]);
  }
  Serial.println();
}

void setVal(int16_t *p, int16_t v) {
  for(int i=0;i<cfg.rcl_num_ch;i++) {
    p[i] = v;
  }
}

void setPc() {
  for(int i=0;i<cfg.rcl_num_ch;i++) {
    pc[i] = rcl.pwm[i];
  }
}

void armToggleInit() {
  arm_last = rcl.pwm[arm.ch];
}

bool armToggled() {
  return (abs((int16_t)rcl.pwm[arm.ch] - arm_last) > 500);
}

void armToggleWait() {
  armToggleInit();
  for(;;) {
    if(armToggled()) return;
    taskYIELD();
  }
}

void findStick(stick_t &stk) {
  int16_t pmin[cfg.rcl_num_ch]={};
  setVal(pmin,9999);
  uint32_t tmin[cfg.rcl_num_ch]={};
  int16_t pmax[cfg.rcl_num_ch]={};
  uint32_t tmax[cfg.rcl_num_ch]={};
  armToggleInit();

  //for each channel: record pwm_min and pwm_max, and record timestamps
  for(int c = 0; ; c++) {
    if(ignore[c]) c++; //skip known channels
    if(c>=cfg.rcl_num_ch) c=0;
    int16_t p = rcl.pwm[c];
    if(pmin[c] > p) {pmin[c] = p; tmin[c] = micros();}
    if(pmax[c] < p) {pmax[c] = p; tmax[c] = micros();}
    if(armToggled()) break;
    taskYIELD();
  }

  //find channel with max difference between pwm_min and pwm_max
  int16_t diffmax = 0;
  int16_t diffc = 0;
  for(int i=0;i<cfg.rcl_num_ch;i++) {
    if(diffmax < pmax[i]-pmin[i]) {
      diffmax = pmax[i]-pmin[i];
      diffc = i;
    }
  }

  //set stick parameters
  stk.ch = diffc;
  stk.min = pmin[stk.ch];
  stk.mid = rcl.pwm[stk.ch];
  stk.max = pmax[stk.ch];
  if( int32_t(tmax[stk.ch] - tmin[stk.ch]) >= 0 ) { //use unsigned math...
    stk.left_pull = pmin[stk.ch];
    stk.right_push = pmax[stk.ch];
  }else{
    stk.left_pull = pmax[stk.ch];
    stk.right_push = pmin[stk.ch];
  }
  ignore[stk.ch] = true; //ignore this channel from now on
}

protected:
void _calibrate() {
  int c;
  Serial.println("\n==== Radio Calibration ====");
  while(!rcl.connected()) {
    Serial.println("Connect radio to continue...");
    delay(500);
  }

  Serial.println("Switch to ARMED (if already armed, toggle until calibration done, then restart 'calradio' DISARMED)");
  setPc();
  for(c=0;;c++) {
    if(c>=cfg.rcl_num_ch) c=0;
    if(abs((int16_t)rcl.pwm[c] - pc[c]) > 500) break;
    taskYIELD();
  }
  arm.ch = c;
  int16_t arm_v1 = pc[c];
  int16_t arm_v2 = rcl.pwm[c];
  
  int16_t arm_armed = arm_v2;
  int16_t arm_disarmed = arm_v1;
  if(abs(arm_disarmed - arm_armed) < 100) arm_disarmed = arm_v2;
  if(arm_disarmed < arm_armed) {
    arm.min = (arm_disarmed+arm_armed)/2;
    arm.max = 2500;
  }else{
    arm.min = 500;
    arm.max = (arm_disarmed+arm_armed)/2;
  }
  Serial.printf("==> ARMED: ch=%d from=%d to=%d\n", arm.ch+1, arm.min, arm.max);
  ignore[arm.ch]=true;

  Serial.println("THROTTLE: First pull throttle idle, then full throttle, then back to center, then toggle arm switch");
  findStick(thr);
  Serial.printf("==> THROTTLE: ch=%d pull=%d mid=%d push=%d\n", rol.ch+1, rol.left_pull, rol.mid, rol.right_push);

  Serial.println("PITCH: First pull stick back, then push forward, then back to center, then toggle arm switch");
  findStick(pit);
  Serial.printf("==> PITCH: ch=%d pull=%d mid=%d push=%d\n", pit.ch+1, pit.left_pull, pit.mid, pit.right_push);

  Serial.println("ROLL: First move stick left, then right, then back to center, then toggle arm switch");
  findStick(rol);
  Serial.printf("==> ROLL: ch=%d left=%d mid=%d right=%d\n", rol.ch+1, rol.left_pull, rol.mid, rol.right_push);

  Serial.println("YAW: First move stick left, then right, then back to center, then toggle arm switch");
  findStick(yaw);
  Serial.printf("==> YAW: ch=%d left=%d mid=%d right=%d\n", yaw.ch+1, yaw.left_pull, yaw.mid, yaw.right_push);

  Serial.println("Select min and max flight modes, then toggle arm switch");
  findStick(flt);
  Serial.printf("==> FLIGHTMODE: ch=%d from=%d to=%d\n", flt.ch+1, flt.min, flt.max);

  //set config
  cfg.rcl_thr_ch    = thr.ch + 1;
  cfg.rcl_thr_pull  = thr.left_pull;
  cfg.rcl_thr_mid   = thr.mid;
  cfg.rcl_thr_push  = thr.right_push;

  cfg.rcl_rol_ch    = rol.ch + 1;
  cfg.rcl_rol_left  = rol.left_pull;
  cfg.rcl_rol_mid   = rol.mid;
  cfg.rcl_rol_right = rol.right_push;

  cfg.rcl_pit_ch    = pit.ch + 1;
  cfg.rcl_pit_pull  = pit.left_pull;
  cfg.rcl_pit_mid   = pit.mid;
  cfg.rcl_pit_push  = pit.right_push;

  cfg.rcl_yaw_ch    = yaw.ch + 1;
  cfg.rcl_yaw_left  = yaw.left_pull;
  cfg.rcl_yaw_mid   = yaw.mid;
  cfg.rcl_yaw_right = yaw.right_push;

  cfg.rcl_arm_ch    = arm.ch + 1;
  cfg.rcl_arm_min   = arm.min;
  cfg.rcl_arm_max   = arm.max;

  cfg.rcl_flt_ch    = flt.ch + 1;
  cfg.rcl_flt_min   = flt.min;
  cfg.rcl_flt_max   = flt.max;

  rcl.setup(); //restart rc to reload config

  Serial.printf("Radio calibration completed! Now use 'pradio' to check, 'cwrite' to save config, or reboot to restart with old config.\n");
}

};
