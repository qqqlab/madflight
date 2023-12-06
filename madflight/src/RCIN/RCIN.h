/*========================================================================================================================
This file contains all necessary functions and code used for radio communication to avoid cluttering the main code

Each USE_RCIN_xxx section in this file defines:
rcin_Setup() -> init
rcin_GetPWM() -> fills global vars channel_1_pwm..channel_6_pwm with received PWM values, returns false on error

Uses rcin_Serial and rcin_PPM_PIN
========================================================================================================================*/


//========================================================================================================================
//SBUS Receiver 
//========================================================================================================================
#if defined USE_RCIN_SBUS
#warning "USE_RX_SBUS not ported/tested - see src/RCIN/RCIN.h"

#include "SBUS/SBUS.h"   //sBus interface

SBUS sbus(*rcin_Serial);
uint16_t sbusChannels[16];
bool sbusFailSafe;
bool sbusLostFrame;

void rcin_Setup() {
  Serial.println("USE_RX_SBUS");
  sbus.begin();
}

bool rcin_GetPWM(int *pwm) {
    if (sbus.read(&sbusChannels[0], &sbusFailSafe, &sbusLostFrame))
    {
      //sBus scaling below is for Taranis-Plus and X4R-SB
      float scale = 0.615;  
      float bias  = 895.0; 
      pwm[0] = sbusChannels[0] * scale + bias;
      pwm[1] = sbusChannels[1] * scale + bias;
      pwm[2] = sbusChannels[2] * scale + bias;
      pwm[3] = sbusChannels[3] * scale + bias;
      pwm[4] = sbusChannels[4] * scale + bias;
      pwm[5] = sbusChannels[5] * scale + bias;
      return true;
    }
    return false;
}


//========================================================================================================================
//DSM Receiver
//========================================================================================================================
#elif defined USE_RCIN_DSM
#warning "USE_RX_DSM not ported/tested - see src/RCIN/RCIN.h"
static const uint8_t num_DSM_channels = 6; //If using DSM RX, change this to match the number of transmitter channels you have

#include "DSMRX/DSMRX.h"  

DSM1024 DSM;

void rcin_Setup() {
  Serial.println("USE_RX_DSM");
  Serial3.begin(115000);
}

void rcin_GetPWM(int *pwm) {
    if (DSM.timedOut(micros())) {
        //Serial.println("*** DSM RX TIMED OUT ***");
    }
    else if (DSM.gotNewFrame()) {
        uint16_t values[num_DSM_channels];
        DSM.getChannelValues(values, num_DSM_channels);

        pwm[0] = values[0];
        pwm[1] = values[1];
        pwm[2] = values[2];
        pwm[3] = values[3];
        pwm[4] = values[4];
        pwm[5] = values[5];
        return true;
    }
    return false;    
}

void serialEvent3(void)
{
  while (rcin_Serial.available()) {
      DSM.handleSerialEvent(Serial3.read(), micros());
  }
}


//========================================================================================================================
//PPM Receiver 
//========================================================================================================================
#elif defined USE_RCIN_PPM
volatile uint32_t channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw, channel_5_raw, channel_6_raw;

//INTERRUPT SERVICE ROUTINE for reading PPM
void getPPM() {
  static uint32_t ppm_counter = 0;
  static uint32_t tpulse_last = 0;
  uint32_t tpulse = micros();
  int trig = digitalRead(rcin_PPM_PIN);
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
    }
    
    ppm_counter = ppm_counter + 1;
  }
}

void rcin_Setup() {
  Serial.printf("USE_RX_PPM pin=%d\n",rcin_PPM_PIN);
  //Declare interrupt pin
  pinMode(rcin_PPM_PIN, INPUT_PULLUP);
  delay(20);
  //Attach interrupt and point to corresponding ISR function
  attachInterrupt(digitalPinToInterrupt(rcin_PPM_PIN), getPPM, CHANGE);
}

bool rcin_GetPWM(int *pwm) {
  pwm[0] = channel_1_raw;
  pwm[1] = channel_2_raw;
  pwm[2] = channel_3_raw;
  pwm[3] = channel_4_raw;
  pwm[4] = channel_5_raw;
  pwm[5] = channel_6_raw;
  return true;
}

//========================================================================================================================
// PWM Receiver
//========================================================================================================================
#elif defined USE_RCIN_PWM
uint32_t channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw, channel_5_raw, channel_6_raw;
uint32_t rising_edge_start_1, rising_edge_start_2, rising_edge_start_3, rising_edge_start_4, rising_edge_start_5, rising_edge_start_6; 

void rcin_Setup() {
  Serial.println("USE_RX_PWM");
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

bool rcin_GetPWM(int *pwm) {
  pwm[0] = channel_1_raw;
  pwm[1] = channel_2_raw;
  pwm[2] = channel_3_raw;
  pwm[3] = channel_4_raw;
  pwm[4] = channel_5_raw;
  pwm[5] = channel_6_raw;
  return true;
}

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
  }
}
#else
  #error "uncomment one USE_RCIN_xxx"
#endif
