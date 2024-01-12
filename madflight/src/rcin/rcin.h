/*========================================================================================================================
This file contains all necessary functions and code used for radio communication to avoid cluttering the main code

Each RCIN_USE_xxx section in this file defines:
rcin_Setup() -> init
rcin_GetPWM(int *pwm) -> fills pwm[0..RCIN_NUM_CHANNELS-1] received PWM values, returns true if new data was received

========================================================================================================================*/

//#define RCIN_USE_NONE 0 //always need a radio
#define RCIN_USE_CRSF 1
#define RCIN_USE_SBUS 2
#define RCIN_USE_DSM 3
#define RCIN_USE_PPM 4
#define RCIN_USE_PWM 5

//========================================================================================================================
//CRSF Receiver 
//========================================================================================================================
#if RCIN_USE == RCIN_USE_CRSF

#include "crsf/crsf.h"
CRSF crsf;

void rcin_Setup() {
  Serial.println("RCIN_USE_CRSF");
  rcin_Serial->begin(CRSF_BAUD);
}

bool rcin_GetPWM(int *pwm) {
    bool rv = false;
    while(rcin_Serial->available()) {
        int c = rcin_Serial->read();
        //print received data
        //if(c == CRSF_ADDRESS_FLIGHT_CONTROLLER) Serial.printf("\nreceived: "); Serial.printf("%02x ",c);
        if(crsf.update(c)) {
            //print decoded rc data
            //Serial.print(" decoded RC: "); for(int i=0;i<16;i++) Serial.printf("%d:%d ",i,crsf.channel[i]); Serial.println();
            rv = true;
        }
    }
    
    for(int i=0; i<RCIN_NUM_CHANNELS; i++) {
        pwm[i] = (int)crsf.channel[i];
    }
    return rv;
}

//========================================================================================================================
//SBUS Receiver 
//========================================================================================================================
#elif RCIN_USE == RCIN_USE_SBUS
#warning "USE_RX_SBUS not ported/tested - see src/RCIN/RCIN.h" //TODO

#include "sbus/SBUS.h" //sBus interface

SBUS sbus(*rcin_Serial);
uint16_t sbusChannels[16];
bool sbusFailSafe;
bool sbusLostFrame;

void rcin_Setup() {
  Serial.println("RCIN_USE_SBUS");
  sbus.begin();
}

bool rcin_GetPWM(int *pwm) {
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

//========================================================================================================================
//DSM Receiver
//========================================================================================================================
#elif RCIN_USE == RCIN_USE_DSM
#warning "USE_RX_DSM not ported/tested - see src/RCIN/RCIN.h" //TODO
static const uint8_t num_DSM_channels = 6; //If using DSM RX, change this to match the number of transmitter channels you have

#include "dsmrx/DSMRX.h"  

DSM1024 DSM;

void rcin_Setup() {
  Serial.println("RCIN_USE_DSM");
  Serial3.begin(115000);
}

void rcin_GetPWM(int *pwm) {
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

void serialEvent3(void)
{
  while (rcin_Serial.available()) {
      DSM.handleSerialEvent(Serial3.read(), micros());
  }
}

//========================================================================================================================
//PPM Receiver 
//========================================================================================================================
#elif RCIN_USE == RCIN_USE_PPM
volatile uint32_t channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw, channel_5_raw, channel_6_raw;

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
    }
    
    ppm_counter = ppm_counter + 1;
  }
}

void rcin_Setup() {
  Serial.printf("RCIN_USE_PPM pin=%d\n",HW_PIN_RCIN_RX);
  //Declare interrupt pin
  pinMode(HW_PIN_RCIN_RX, INPUT_PULLUP);
  delay(20);
  //Attach interrupt and point to corresponding ISR function
  attachInterrupt(digitalPinToInterrupt(HW_PIN_RCIN_RX), getPPM, CHANGE);
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
#elif RCIN_USE == RCIN_USE_PWM
uint32_t channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw, channel_5_raw, channel_6_raw;
uint32_t rising_edge_start_1, rising_edge_start_2, rising_edge_start_3, rising_edge_start_4, rising_edge_start_5, rising_edge_start_6; 

void rcin_Setup() {
  Serial.println("RCIN_USE_PWM");
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

//=====================================================================================================================
// None or undefined
//=====================================================================================================================
#elif RCIN_USE == RCIN_USE_NONE || !defined RCIN_USE
  #error "RCIN_USE not defined"
  
//=====================================================================================================================
// Invalid value
//=====================================================================================================================
#else
  #error "invalid RCIN_USE value"
#endif



//########################################################################################################################
//  TELEMETRY
//########################################################################################################################


//========================================================================================================================
//  CRSF
//========================================================================================================================
#if RCIN_USE == RCIN_USE_CRSF

#include "crsf/crsf_telemetry.h"
void rcin_telemetry_gps(int32_t lat, int32_t lon, uint16_t sog_kmh, uint16_t cog_deg, uint16_t alt_m, uint8_t sats) {
    uint8_t buf[65];
    int len = CRSF_Telemetry::telemetry_gps(buf, lat, lon, sog_kmh, cog_deg, alt_m, sats);
    rcin_Serial->write(buf, len);
}

void rcin_telemetry_flight_mode(const char *flight_mode) {
    uint8_t buf[65];
    int len = CRSF_Telemetry::telemetry_flight_mode(buf, flight_mode);
    rcin_Serial->write(buf, len);
    //Serial.printf("\nFM(len=%d) ",len);
    //for(int i=0;i<len;i++) Serial.printf("%02X ",buf[i]);
}

void rcin_telemetry_attitude(float pitch, float roll, float yaw) {
    uint8_t buf[65];
    int len = CRSF_Telemetry::telemetry_attitude(buf, pitch, roll, yaw);
    rcin_Serial->write(buf, len);
    //Serial.printf("\natt(len=%d) ",len);
    //for(int i=0;i<len;i++) Serial.printf("%02X ",buf[i]);
}

void rcin_telemetry_battery(float voltage_V, float current_A, int fuel_mAh, uint8_t remaining) {
    uint8_t buf[65];
    int len = CRSF_Telemetry::telemetry_battery(buf, voltage_V, current_A, fuel_mAh, remaining);
    rcin_Serial->write(buf, len);
}

//========================================================================================================================
//  OTHERS
//========================================================================================================================
#else



void rcin_telemetry_gps(int32_t latitude, int32_t longitude, uint16_t groundspeed, uint16_t gps_heading, uint16_t altitude, uint8_t num_satellites) {
}

void rcin_telemetry_flight_mode(const char *flight_mode) {
}

void rcin_telemetry_attitude(float pitch, float roll, float yaw) {
}

void rcin_telemetry_battery(float voltage_V, float current_A, int fuel_mAh, uint8_t remaining) {
}

#endif
