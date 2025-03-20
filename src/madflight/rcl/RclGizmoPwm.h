/*

#if cfg.rcl_num_ch > 6
  #error RCL_USE_PWM has max 6 channels, change cfg.rcl_num_ch
#endif

#warning "RCL_USE_PWM not ported/tested - see rcl.h" //TODO
uint32_t channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw, channel_5_raw, channel_6_raw;
uint32_t rising_edge_start_1, rising_edge_start_2, rising_edge_start_3, rising_edge_start_4, rising_edge_start_5, rising_edge_start_6; 
volatile bool rcl_updated = false;

//INTERRUPT SERVICE ROUTINES for reading PWM
void getCh1() {
  int trigger = digitalRead(ch1Pin);
  if(trigger == 1) {
    rising_edge_start_1 = micros();
  }
  else if(trigger == 0) {
    channel_1_raw = micros() - rising_edge_start_1;
  }
}

void getCh2() {
  int trigger = digitalRead(ch2Pin);
  if(trigger == 1) {
    rising_edge_start_2 = micros();
  }
  else if(trigger == 0) {
    channel_2_raw = micros() - rising_edge_start_2;
  }
}

void getCh3() {
  int trigger = digitalRead(ch3Pin);
  if(trigger == 1) {
    rising_edge_start_3 = micros();
  }
  else if(trigger == 0) {
    channel_3_raw = micros() - rising_edge_start_3;
  }
}

void getCh4() {
  int trigger = digitalRead(ch4Pin);
  if(trigger == 1) {
    rising_edge_start_4 = micros();
  }
  else if(trigger == 0) {
    channel_4_raw = micros() - rising_edge_start_4;
  }
}

void getCh5() {
  int trigger = digitalRead(ch5Pin);
  if(trigger == 1) {
    rising_edge_start_5 = micros();
  }
  else if(trigger == 0) {
    channel_5_raw = micros() - rising_edge_start_5;
  }
}

void getCh6() {
  int trigger = digitalRead(ch6Pin);
  if(trigger == 1) {
    rising_edge_start_6 = micros();
  }
  else if(trigger == 0) {
    channel_6_raw = micros() - rising_edge_start_6;
    rcl_updated = true;
  }
}

class RclPwm : public Rcl {
  public:
    void _setup() override {
      Serial.println("RCL_USE_PWM");
      pwm = pwm_instance;
      //Declare interrupt pins 
      pinMode(ch1Pin, INPUT_PULLUP);
      pinMode(ch2Pin, INPUT_PULLUP);
      pinMode(ch3Pin, INPUT_PULLUP);
      pinMode(ch4Pin, INPUT_PULLUP);
      pinMode(ch5Pin, INPUT_PULLUP);
      pinMode(ch6Pin, INPUT_PULLUP);
      delay(20);
      //Attach interrupt and point to corresponding ISR functions
      attachInterrupt(digitalPinToInterrupt(ch1Pin), getCh1, CHANGE);
      attachInterrupt(digitalPinToInterrupt(ch2Pin), getCh2, CHANGE);
      attachInterrupt(digitalPinToInterrupt(ch3Pin), getCh3, CHANGE);
      attachInterrupt(digitalPinToInterrupt(ch4Pin), getCh4, CHANGE);
      attachInterrupt(digitalPinToInterrupt(ch5Pin), getCh5, CHANGE);
      attachInterrupt(digitalPinToInterrupt(ch6Pin), getCh6, CHANGE);
      delay(20);
    }

    bool _update() override {
      pwm[0] = channel_1_raw;
      pwm[1] = channel_2_raw;
      pwm[2] = channel_3_raw;
      pwm[3] = channel_4_raw;
      pwm[4] = channel_5_raw;
      pwm[5] = channel_6_raw;
      bool rv = rcl_updated;
      rcl_updated = false;
      return rv;
    }

  private:
    uint16_t pwm_instance[6];
};
*/