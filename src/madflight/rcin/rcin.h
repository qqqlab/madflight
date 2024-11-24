/*==========================================================================================
rcin.h - madflight RC radio receiver

MIT License

Copyright (c) 2023-2024 https://madflight.com

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

#include "../interface.h" //RCIN interface definition
#include "../cfg/cfg.h"
#include "rcin_calibrate.h"

#ifndef RCIN_NUM_CHANNELS
  #define RCIN_NUM_CHANNELS 8 //number of receiver channels (minimal 6)
#endif

#ifndef RCIN_TIMEOUT
  #define RCIN_TIMEOUT 3000 // lost connection timeout in milliseconds
#endif

#ifndef RCIN_STICK_DEADBAND
  #define RCIN_STICK_DEADBAND 0 //pwm deadband around stick center
#endif

#ifndef RCIN_THROTTLE_DEADBAND
  #define RCIN_THROTTLE_DEADBAND 60 //pwm deadband for zero throttle
#endif

#define RCIN_USE_NONE  0
#define RCIN_USE_CRSF  1
#define RCIN_USE_SBUS  2
#define RCIN_USE_DSM   3
#define RCIN_USE_PPM   4
#define RCIN_USE_PWM   5
#define RCIN_USE_DEBUG 6

//Rcin implements public interface, and is base for specific rcin radio classes
class Rcin : public Rcin_interface {
  public:
    void setup() override;
    bool update() override; //returns true if channel pwm data was updated
    bool connected() override;
    void calibrate() override; //interactive calibration

  private:
    virtual bool _update() = 0; //update the specific radio
    virtual void _setup() = 0;  //setup the specific radio
    float _ChannelNormalize(int val, int min, int center, int max, int deadband);
    void _setupStick(int stickno, int ch, int left_pull, int mid, int right_push);
    
    uint32_t update_time = 0;

    enum stick_enum {THR,ROL,PIT,YAW,ARM,FLT};

    struct stick_t{
      uint8_t ch; //stick/switch channel - 0 based
      uint16_t min; //stick/switch min pwm
      uint16_t mid; //stick center pwm
      uint16_t max; //stick/switch max pwm
      int8_t inv; //stick inverted: -1 inverted, 1 normal
    } st[6];

    uint16_t st_flt_spacing;
    uint16_t dummy_pwm = 0;
};

void Rcin::_setupStick(int stickno, int ch, int left_pull, int mid, int right_push) {
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

void Rcin::setup() {
  throttle = 0;
  roll = 0;
  pitch = 0;
  yaw = 0;
  vspeed = 0;
  arm = false;

  //setup stick/switch parameters from config values
  Serial.printf("RCIN: setup channels thr=%d rol=%d pit=%d yaw=%d arm=%d flt=%d\n", (int)cfg.RCIN_THR_CH, (int)cfg.RCIN_ROL_CH, (int)cfg.RCIN_PIT_CH, (int)cfg.RCIN_YAW_CH, (int)cfg.RCIN_ARM_CH, (int)cfg.RCIN_FLT_CH);
  _setupStick(THR, cfg.RCIN_THR_CH, cfg.RCIN_THR_PULL, cfg.RCIN_THR_MID, cfg.RCIN_THR_PUSH);
  _setupStick(ROL, cfg.RCIN_ROL_CH, cfg.RCIN_ROL_LEFT, cfg.RCIN_ROL_MID, cfg.RCIN_ROL_RIGHT);
  _setupStick(PIT, cfg.RCIN_PIT_CH, cfg.RCIN_PIT_PULL, cfg.RCIN_PIT_MID, cfg.RCIN_PIT_PUSH);
  _setupStick(YAW, cfg.RCIN_YAW_CH, cfg.RCIN_YAW_LEFT, cfg.RCIN_YAW_MID, cfg.RCIN_YAW_RIGHT);
  st[ARM].ch  = cfg.RCIN_ARM_CH-1;
  st[ARM].min = cfg.RCIN_ARM_MIN;
  st[ARM].mid = 0; //NOT USED
  st[ARM].max = cfg.RCIN_ARM_MAX;
  st[ARM].inv = 0; //NOT USED
  st[FLT].ch  = cfg.RCIN_FLT_CH-1;
  st[FLT].min = cfg.RCIN_FLT_MIN;
  st[FLT].mid = 0; //NOT USED
  st[FLT].max = 0; //NOT USED
  st[FLT].inv = 0; //NOT USED
  st_flt_spacing = (cfg.RCIN_FLT_MAX - cfg.RCIN_FLT_MIN) / 5;

  //check st parameters
  for(int i=0;i<6;i++) {
    if(st[i].ch >= RCIN_NUM_CHANNELS) {
      Serial.println("RCIN: invalid channel in config, re-calibrate radio");
      st[i].ch  = 0;
      st[i].min = 1;
      st[i].mid = 2;
      st[i].max = 3;
      st[i].inv = 1;
    }
  }

  //setup specific radio 
  _setup();
}

bool Rcin::update() { //returns true if channel pwm data was updated
  bool rv = _update();
  if(rv) {
    //throttle: 0.0 in range from stick full back to rcin_cfg_thro_low, 1.0 on full throttle
    float tlow = st[THR].min + RCIN_THROTTLE_DEADBAND;
    throttle = constrain( ((float)(pwm[st[THR].ch] - tlow)) / (st[THR].max - tlow), 0.0, 1.0);

    //roll,pitch,yaw
    vspeed =  st[THR].inv * _ChannelNormalize(pwm[st[THR].ch], st[THR].min, st[THR].mid, st[THR].max, RCIN_STICK_DEADBAND); // output: -1 (descent, st back) to 1 (ascent, st forward)
    roll   =  st[ROL].inv * _ChannelNormalize(pwm[st[ROL].ch], st[ROL].min, st[ROL].mid, st[ROL].max, RCIN_STICK_DEADBAND); // output: -1 (roll left, st left) to 1 (roll right, st right)
    pitch  = -st[PIT].inv * _ChannelNormalize(pwm[st[PIT].ch], st[PIT].min, st[PIT].mid, st[PIT].max, RCIN_STICK_DEADBAND); // output: -1 (pitch down, st back) to 1 (pitch up, st forward)
    yaw    =  st[YAW].inv * _ChannelNormalize(pwm[st[YAW].ch], st[YAW].min, st[YAW].mid, st[YAW].max, RCIN_STICK_DEADBAND); // output: -1 (yaw left, st left) to 1 (yaw right, st right)

    //arm switch
    arm = (st[ARM].min <= pwm[st[ARM].ch] && pwm[st[ARM].ch] < st[ARM].max);

    //flightmode 6 position switch
    flightmode = constrain( ( pwm[st[FLT].ch] - st[FLT].min + st_flt_spacing/2) / st_flt_spacing, 0, 5); //output 0..5

    //set update timestamp
    update_time = millis();
  }
  return rv;
}

bool Rcin::connected() {
  return ((uint32_t)millis() - update_time <= (RCIN_TIMEOUT) );
}

//helper to nomalize a channel based on min,center,max calibration
float Rcin::_ChannelNormalize(int val, int min, int center, int max, int deadband) {
  int rev = 1; //1=normal, -1=reverse channel
  //needs: min < center < max
  if(val<min) return rev * -1.0;
  if(val<center-deadband) return (float)(rev * (val-(center-deadband))) / ((center-deadband)-min); //returns -1 to 0
  if(val<center+deadband) return 0;
  if(val<max) return (float)(rev * (val-(center+deadband))) / (max-(center+deadband)); 
  return rev * 1.0;
}

void Rcin::calibrate() {
  RcinCalibrate::calibrate();
}

//=================================================================================================
// None or undefined
//=================================================================================================
#if RCIN_USE == RCIN_USE_NONE || !defined RCIN_USE

class RcinDebug : public Rcin {
  public:
    void _setup() override {
      Serial.printf("RCIN_USE_NONE\n");
      pwm = pwm_instance;
    };
  private:
    bool _update() override { //returns true if channel pwm data was updated
      for(int i=0;i<RCIN_NUM_CHANNELS;i++) pwm[i]=1500;
      return true;
    }
  private:
    uint16_t pwm_instance[RCIN_NUM_CHANNELS];
};

RcinDebug rcin_instance;

//=================================================================================================
//CRSF Receiver 
//=================================================================================================
#elif RCIN_USE == RCIN_USE_CRSF

#include "crsf/crsf.h"
CRSF rcin_crsf;

class RcinCSRF : public Rcin {
  public:
    void _setup() override {
      Serial.println("RCIN: RCIN_USE_CRSF");
      pwm = rcin_crsf.channel;
      rcin_Serial->begin(CRSF_BAUD);
    }

    bool _update() override {
      bool rv = false;
      while(rcin_Serial->available()) {
        int c = rcin_Serial->read();
        //print received data
        //if(c == CRSF_ADDRESS_FLIGHT_CONTROLLER) Serial.printf("\nreceived: "); Serial.printf("%02x ",c);
        if(rcin_crsf.update(c)) {
          //print decoded rc data
          //Serial.print(" decoded RC: "); for(int i=0;i<16;i++) Serial.printf("%d:%d ",i,rcin_crsf.channel[i]); Serial.println();
          rv = true;
        }
      }
      return rv;
    }
};

RcinCSRF rcin_instance;

//=================================================================================================
//SBUS Receiver 
//=================================================================================================
#elif RCIN_USE == RCIN_USE_SBUS
#warning "RCIN_USE_SBUS not ported/tested - see src/rcin/rcin.h" //TODO

#include "sbus/SBUS.h" //sBus interface

SBUS sbus(*rcin_Serial);
uint16_t sbusChannels[16];
bool sbusFailSafe;
bool sbusLostFrame;

class RcinSBUS : public Rcin {
  public:
    void _setup() override {
      Serial.println("RCIN_USE_SBUS");
      pwm = pwm_instance;
      sbus.begin();
    }

    bool _update() override {
      if (sbus.read(sbusChannels, &sbusFailSafe, &sbusLostFrame))
      {
        //sBus scaling below is for Taranis-Plus and X4R-SB
        float scale = 0.615;
        float bias  = 895.0;
        for(int i=0;i<RCIN_NUM_CHANNELS;i++) {
            pwm[i] = sbusChannels[i] * scale + bias;
        }
        return true;
      }
      return false;
    }

  private:
    uint16_t pwm_instance[16];
};

RcinSBUS rcin_instance;

//=================================================================================================
//DSM Receiver
//=================================================================================================
#elif RCIN_USE == RCIN_USE_DSM
#warning "RCIN_USE_DSM not ported/tested - see src/rcin/rcin.h" //TODO
static const uint8_t num_DSM_channels = 6; //If using DSM RX, change this to match the number of transmitter channels you have

#include "dsmrx/DSMRX.h"  

DSM1024 DSM;

void serialEvent3(void)
{
  while (rcin_Serial.available()) {
      DSM.handleSerialEvent(Serial3.read(), micros());
  }
}

class RcinDSM : public Rcin {
  public:
    void _setup() override {
      Serial.println("RCIN_USE_DSM");
      pwm = pwm_instance;
      Serial3.begin(115000);
    }

    bool _update() override {
      if (DSM.timedOut(micros())) {
          //Serial.println("*** DSM RX TIMED OUT ***");
      }
      else if (DSM.gotNewFrame()) {
          uint16_t values[num_DSM_channels];
          DSM.getChannelValues(values, num_DSM_channels);
          for(int i=0;i<RCIN_NUM_CHANNELS;i++) {
              pwm[i] = values[i];
          }
          return true;
      }
      return false;
    }

  private:
    uint16_t pwm_instance[16];
};

RcinDSM rcin_instance;

//=================================================================================================
//PPM Receiver 
//=================================================================================================
#elif RCIN_USE == RCIN_USE_PPM

#if RCIN_NUM_CHANNELS > 6
  #error RCIN_USE_PPM has max 6 channels, change RCIN_NUM_CHANNELS
#endif

volatile uint32_t channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw, channel_5_raw, channel_6_raw;
volatile bool rcin_updated = false;

//INTERRUPT SERVICE ROUTINE for reading PPM
void getPPM() {
  static uint32_t ppm_counter = 0;
  static uint32_t tpulse_last = 0;
  uint32_t tpulse = micros();
  int trig = digitalRead(HW_PIN_RCIN_RX);
  if (trig==1) { //Only care about rising edge
    uint32_t dt_ppm = tpulse - tpulse_last;
    tpulse_last = tpulse;

    channel_1_raw++;

    if (dt_ppm > 5000) { //Waiting for long pulse to indicate a new pulse train has arrived
      ppm_counter = 0;
    }
  
    if (ppm_counter == 1) { //First pulse
      channel_1_raw = dt_ppm;
    }
  
    if (ppm_counter == 2) { //Second pulse
      channel_2_raw = dt_ppm;
    }
  
    if (ppm_counter == 3) { //Third pulse
      channel_3_raw = dt_ppm;
    }
  
    if (ppm_counter == 4) { //Fourth pulse
      channel_4_raw = dt_ppm;
    }
  
    if (ppm_counter == 5) { //Fifth pulse
      channel_5_raw = dt_ppm;
    }
  
    if (ppm_counter == 6) { //Sixth pulse
      channel_6_raw = dt_ppm;
      rcin_updated = true;
    }
    
    ppm_counter = ppm_counter + 1;
  }
}

class RcinPPM : public Rcin {
  public:
    void _setup() override {
      Serial.printf("RCIN_USE_PPM pin=%d\n",HW_PIN_RCIN_RX);
      pwm = pwm_instance;
      //Declare interrupt pin
      pinMode(HW_PIN_RCIN_RX, INPUT_PULLUP);
      delay(20);
      //Attach interrupt and point to corresponding ISR function
      attachInterrupt(digitalPinToInterrupt(HW_PIN_RCIN_RX), getPPM, CHANGE);
    }

    bool _update() override {
      pwm[0] = channel_1_raw;
      pwm[1] = channel_2_raw;
      pwm[2] = channel_3_raw;
      pwm[3] = channel_4_raw;
      pwm[4] = channel_5_raw;
      pwm[5] = channel_6_raw;
      bool rv = rcin_updated;
      rcin_updated = false;
      return rv;
    }

  private:
    uint16_t pwm_instance[6];
};

RcinPPM rcin_instance;

//=================================================================================================
// PWM Receiver
//=================================================================================================
#elif RCIN_USE == RCIN_USE_PWM

#if RCIN_NUM_CHANNELS > 6
  #error RCIN_USE_PWM has max 6 channels, change RCIN_NUM_CHANNELS
#endif

#warning "RCIN_USE_PWM not ported/tested - see src/rcin/rcin.h" //TODO
uint32_t channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw, channel_5_raw, channel_6_raw;
uint32_t rising_edge_start_1, rising_edge_start_2, rising_edge_start_3, rising_edge_start_4, rising_edge_start_5, rising_edge_start_6; 
volatile bool rcin_updated = false;

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
    rcin_updated = true;
  }
}

class RcinPWM : public Rcin {
  public:
    void _setup() override {
      Serial.println("RCIN_USE_PWM");
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
      bool rv = rcin_updated;
      rcin_updated = false;
      return rv;
    }

  private:
    uint16_t pwm_instance[6];
};

RcinPWM rcin_instance;

//=================================================================================================
// Invalid value
//=================================================================================================
#else
  #error "invalid RCIN_USE value"
#endif

Rcin_interface &rcin = rcin_instance;
