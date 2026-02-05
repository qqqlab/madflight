#pragma once

#include "../hal/hal.h"
#include "../imu/imu.h"

class RclCalibrate {

public:
  static void calibrate() {
    //disable IMU interrupt
    void (*onUpdate_saved)(void) = imu.onUpdate;
    imu.onUpdate = nullptr;

    //calibrate
    RclCalibrate *cal = new RclCalibrate();
    if(!cal->_calibrate()) {
      Serial.println("\n==== Radio Calibration Aborted ====");
    }
    
    //enable IMU interrupt
    imu.onUpdate = onUpdate_saved;
  }

  struct stick_t{
    int8_t ch = -1; //stick channel (0-based, -1 is not assigned)
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

  bool assigned_ch[RCL_MAX_CH] = {}; //channels which were assigned a function
  int16_t prev_pwm[RCL_MAX_CH] = {}; //previous pwm values
  int16_t prev_arm; //previous arm value

//update rcl, yield and check for cancel, return false if 'q' was pressed
bool do_events(bool *event_key = nullptr) {
  taskYIELD(); //yield RCL (and other) tasks

  while(Serial.available()) {
    //set event_key flag
    if(event_key) *event_key = true;
    
    //exit if 'q' was pressed
    char c = Serial.read();
    if(c=='q' || c=='Q') {
      //empty receive buffer
      while(Serial.available()) Serial.read();
      return false;
    }
  }

  return true;
}

//wait for key, returns false if 'q' was pressed
bool prompt(const char *msg) {
  Serial.println(msg);

  //empty receive buffer
  while(Serial.available()) Serial.read();

  //wait for event
  bool event_key = false;
  for(;;) {
    if(!do_events(&event_key)) return false;;
    if(event_key) break;
  }

  return true;
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

void set_prev_pwm() {
  for(int i=0;i<cfg.rcl_num_ch;i++) {
    prev_pwm[i] = rcl.pwm[i];
  }
}

//returns false if 'q' was pressed
bool findStick(stick_t &stk) {
  int16_t pmin[cfg.rcl_num_ch]={};
  setVal(pmin,9999);
  uint32_t tmin[cfg.rcl_num_ch]={};
  int16_t pmax[cfg.rcl_num_ch]={};
  uint32_t tmax[cfg.rcl_num_ch]={};
  int16_t prev_arm = rcl.pwm[arm.ch];

  //for each channel: record pwm_min and pwm_max, and record timestamps
  //until armed switch toggle or key press
  bool event_key = false;
  int c = 0;
  while(1) {
    if(!do_events(&event_key)) return false; //quit

    c++;
    if(c >= cfg.rcl_num_ch) c=0;
    if(!assigned_ch[c]) { //skip assigned channels
      int16_t p = rcl.pwm[c];
      if(pmin[c] > p) {pmin[c] = p; tmin[c] = micros();}
      if(pmax[c] < p) {pmax[c] = p; tmax[c] = micros();}
      if(event_key || (arm.ch >= 0 && abs((int16_t)rcl.pwm[arm.ch] - prev_arm) > 500)) break;
    }
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
  assigned_ch[stk.ch] = true; //assigned_ch this channel from now on

  return true;
}

protected:
bool _calibrate() {
  Serial.println("\n==== Radio Calibration ====");

  //check connected
  if(!rcl.connected()) {
    Serial.println("ERROR: radio not connected. Connect and try again");
    return false;
  }

  Serial.println("During calibration type 'q' to quit");

  //prompt DISARMED
  if(!prompt("\n--- Step 1: Start Calibration ---\nSet arm switch to DISARMED and CENTER all sticks including throttle, then press ENTER to continue")) return false;

  //get armed switch
  Serial.println("\n--- Step 2: Arm Switch Calibration ---\nSwitch to ARMED, or press enter for no arm switch");
  bool event_key = false;
  set_prev_pwm();
  int c = 0;
  while(1) {
    c++;
    if(c >= cfg.rcl_num_ch) c = 0;
    if(abs((int16_t)rcl.pwm[c] - prev_pwm[c]) > 500) break;
    if(!do_events(&event_key)) return false;
    if(event_key) break;
  }
  if(event_key) {
    //no armed channel - can't arm!
    arm.ch = -1;
    arm.min = 0;
    arm.max = 0;
  }else{
    arm.ch = c;
    int16_t arm_v1 = prev_pwm[c];
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
    assigned_ch[arm.ch] = true;
  }
  Serial.printf("==> Arm Switch: ch=%d from=%d to=%d\n", arm.ch+1, arm.min, arm.max);

  //get throttle
  Serial.println("\n--- Step 3: Throttle Stick Calibration ---\nFirst pull throttle idle, then full throttle, then back to center, then toggle arm switch or press enter");
  if(!findStick(thr)) return false;
  Serial.printf("==> Throttle: ch=%d pull=%d mid=%d push=%d\n", rol.ch+1, rol.left_pull, rol.mid, rol.right_push);

  //get pitch
  Serial.println("\n--- Step 4: Pitch Stick Calibration ---\nFirst pull pitch stick back, then push forward, then back to center, then toggle arm switch or press enter");
  if(!findStick(pit)) return false;
  Serial.printf("==> Pitch: ch=%d pull=%d mid=%d push=%d\n", pit.ch+1, pit.left_pull, pit.mid, pit.right_push);

  //get roll
  Serial.println("\n--- Step 5: Roll Stick Calibration ---\nFirst move roll stick left, then right, then back to center, then toggle arm switch or press enter");
  if(!findStick(rol)) return false;
  Serial.printf("  ==> Roll: ch=%d left=%d mid=%d right=%d\n", rol.ch+1, rol.left_pull, rol.mid, rol.right_push);

  //get yaw
  Serial.println("\n--- Step 6: Yaw Stick Calibration ---\nFirst move yaw stick left, then right, then back to center, then toggle arm switch or press enter");
  if(!findStick(yaw)) return false;
  Serial.printf("  ==> Yaw: ch=%d left=%d mid=%d right=%d\n", yaw.ch+1, yaw.left_pull, yaw.mid, yaw.right_push);

  //get flight mode
  Serial.println("\n--- Step 7: Flight Mode Switch Calibration ---\nSelect min and max flight modes, then toggle arm switch or press enter");
  Serial.println("  -or- Press 'q' for no flight mode switch");
  if(!findStick(flt)) {
    flt.ch = -1;
    flt.min = 0;
    flt.max = 0;
  }
  Serial.printf("  ==> Flight Mode Switch: ch=%d from=%d to=%d\n", flt.ch+1, flt.min, flt.max);

  //set config
  cfg.rcl_thr_ch    = thr.ch + 1; //config channels are 1-based
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

  Serial.println();
  rcl.setup(); //restart rcl to reload config (outputs RCL messages)

  Serial.printf("\n=== Radio Calibration Completed ===\nNow use 'prcl' to check, 'save' to save config, or 'reboot' to restart with old config.\n");
  return true;
}

};
