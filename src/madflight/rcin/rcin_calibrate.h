
//this class assumes rcin.update() is called in the background... (i.e. in imu_loop())

void rcin_cal_imu_loop() {
   rcin.update();
}

class RcinCalibrate {

public:
  static void calibrate() {
    imu.onUpdate = rcin_cal_imu_loop;
    RcinCalibrate *cal = new RcinCalibrate();
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

  bool ignore[RCIN_NUM_CHANNELS]={}; //channels to ignore
  int16_t pc[RCIN_NUM_CHANNELS] = {0}; //temp value
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
  for(int i=0;i<RCIN_NUM_CHANNELS;i++) {
    Serial.printf("ch%d:%4d ",i,rcin.pwm[i]);
  }
  Serial.println();
}

void printpwm(const char *msg, int16_t *p) {
  Serial.printf(msg);
  for(int i=0;i<RCIN_NUM_CHANNELS;i++) {
    Serial.printf(" ch%d:%4d",i,p[i]);
  }
  Serial.println();
}

void setVal(int16_t *p, int16_t v) {
  for(int i=0;i<RCIN_NUM_CHANNELS;i++) {
    p[i] = v;
  }
}

void setPc() {
  for(int i=0;i<RCIN_NUM_CHANNELS;i++) {
    pc[i] = rcin.pwm[i];
  }
}

void armToggleInit() {
  arm_last = rcin.pwm[arm.ch];
}

bool armToggled() {
  return (abs((int16_t)rcin.pwm[arm.ch] - arm_last) > 500);
}

void armToggleWait() {
  armToggleInit();
  for(;;) {
    if(armToggled()) return;
    taskYIELD();
  }
}

void findStick(stick_t &stk) {
  int16_t pmin[RCIN_NUM_CHANNELS]={};
  setVal(pmin,9999);
  uint32_t tmin[RCIN_NUM_CHANNELS]={};
  int16_t pmax[RCIN_NUM_CHANNELS]={};
  uint32_t tmax[RCIN_NUM_CHANNELS]={};
  armToggleInit();

  //for each channel: record pwm_min and pwm_max, and record timestamps
  for(int c = 0; ; c++) {
    if(ignore[c]) c++; //skip known channels
    if(c>=RCIN_NUM_CHANNELS) c=0;
    int16_t p = rcin.pwm[c];
    if(pmin[c] > p) {pmin[c] = p; tmin[c] = micros();}
    if(pmax[c] < p) {pmax[c] = p; tmax[c] = micros();}
    if(armToggled()) break;
    taskYIELD();
  }

  //find channel with max difference between pwm_min and pwm_max
  int16_t diffmax = 0;
  int16_t diffc = 0;
  for(int i=0;i<RCIN_NUM_CHANNELS;i++) {
    if(diffmax < pmax[i]-pmin[i]) {
      diffmax = pmax[i]-pmin[i];
      diffc = i;
    }
  }

  //set stick parameters
  stk.ch = diffc;
  stk.min = pmin[stk.ch];
  stk.mid = rcin.pwm[stk.ch];
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
  while(!rcin.connected()) {
    Serial.println("Connect radio to continue...");
    delay(500);
  }

  Serial.println("Switch to ARMED (if already armed, toggle until calibration done, then restart 'calradio' DISARMED)");
  setPc();
  for(c=0;;c++) {
    if(c>=RCIN_NUM_CHANNELS) c=0;
    if(abs((int16_t)rcin.pwm[c] - pc[c]) > 500) break;
    taskYIELD();
  }
  arm.ch = c;
  int16_t arm_v1 = pc[c];
  int16_t arm_v2 = rcin.pwm[c];
  
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
  cfg.RCIN_THR_CH    = thr.ch+1;
  cfg.RCIN_THR_PULL  = thr.left_pull;
  cfg.RCIN_THR_MID   = thr.mid;
  cfg.RCIN_THR_PUSH  = thr.right_push;

  cfg.RCIN_ROL_CH    = rol.ch+1;
  cfg.RCIN_ROL_LEFT  = rol.left_pull;
  cfg.RCIN_ROL_MID   = rol.mid;
  cfg.RCIN_ROL_RIGHT = rol.right_push;

  cfg.RCIN_PIT_CH    = pit.ch+1;
  cfg.RCIN_PIT_PULL  = pit.left_pull;
  cfg.RCIN_PIT_MID   = pit.mid;
  cfg.RCIN_PIT_PUSH  = pit.right_push;

  cfg.RCIN_YAW_CH    = yaw.ch+1;
  cfg.RCIN_YAW_LEFT  = yaw.left_pull;
  cfg.RCIN_YAW_MID   = yaw.mid;
  cfg.RCIN_YAW_RIGHT = yaw.right_push;

  cfg.RCIN_ARM_CH    = arm.ch+1;
  cfg.RCIN_ARM_MIN   = arm.min;
  cfg.RCIN_ARM_MAX   = arm.max;

  cfg.RCIN_FLT_CH    = flt.ch+1;
  cfg.RCIN_FLT_MIN   = flt.min;
  cfg.RCIN_FLT_MAX   = flt.max;

  rcin.setup(); //restart rcin to reload config

  Serial.printf("Radio calibration completed! Now use 'pradio' to check, 'cwrite' to save config, or reboot to restart with old config.\n");
}

};
