#include <Arduino.h>

#include "qqqlab_AutoBaud.h"

//use high frequency clock if available, and define macro to place ISR in RAM
#if defined ARDUINO_ARCH_ESP32
  #define HAL_CYCLE_COUNT_FREQ() ((int)ESP.getCpuFreqMHz() * 1000000)
  #define HAL_CYCLE_COUNT() ((uint32_t)ESP.getCycleCount())
  #define HAL_IRAM_FUNC(func) IRAM_ATTR func
#elif defined ARDUINO_ARCH_RP2040
  #define HAL_CYCLE_COUNT_FREQ() ((int)rp2040.f_cpu())
  #define HAL_CYCLE_COUNT() ((uint32_t)rp2040.getCycleCount())
  #define HAL_IRAM_FUNC(func) __not_in_flash_func(func)
#else 
  #define HAL_CYCLE_COUNT_FREQ() ((int)1000000)
  #define HAL_CYCLE_COUNT() ((uint32_t)micros())
  #define HAL_IRAM_FUNC(func) func  
#endif


static int bauds[6] = {230400, 115200, 57600, 38400, 19200, 9600};
static int ab_score[6] = {};
static int t_bauds[6] = {};

static uint32_t ts = 0;
static volatile int ab_cnt = -1;
static int ab_pin;

//Accumulate score per baud rate, highest score wins.
//Non inverted serial starts is high when idle, a transmission starts with a low start bit, then 8 data bits, then a high stop bit. 
//Thus, low pulses are exactly 1,2,..,9 Tbit(s) long, but high pulses can be any length longer than 1 Tbit (stop bit plus variable idle time)
static void HAL_IRAM_FUNC(update_score)(int sample, bool bitneg) {
  for(int i=0;i<6;i++) {
    int bitcnt10 = (sample * 10 + t_bauds[i]/2) / t_bauds[i]; //tenth of a bits
    int bitcnt = (bitcnt10 + 5) / 10;
    int bitfrac = bitcnt10 - 10 * bitcnt; //-5 .. 4 - tenth of a bit difference from full bit, eg +3 is 25-35% longer than a integral number of bits
    if(bitcnt10 < 8 ) ab_score[i] -= 10 * (8 - bitcnt10); //too short (both pos and neg pulses)
    if(bitneg) {
      //negative pulse should be exactly 1,2,3,..,9 bits long
      if(bitcnt == 9) ab_score[i] -= 10; //too long
      else if(bitcnt >= 10) ab_score[i] -= 100; //way too long
      else if(bitfrac <= -3 || bitfrac >= 3) ab_score[i] -= 1; //not an integral number of bits
      else if(bitcnt == 1) ab_score[i] += 2; //1 bit long +/- 30%
      else ab_score[i] += 1; //2-9 bits long +/- 30%
    }
    //Serial.printf("baud:%d sample[i]:%d t_baud:%d bitcnt10:%d bitcnt:%d bitfrac:%d bitf:%f ab_score:%d\n",baud, sample,t_baud,bitcnt10,bitcnt,bitfrac,(float)s / t_baud,ab_score[i]);
  }
}

static void HAL_IRAM_FUNC(ab_isr)() {
  int newstate = digitalRead(ab_pin);
  //uint32_t now = micros();
  uint32_t now = HAL_CYCLE_COUNT();
  int dt = now - ts;
  ts = now;
  if(ab_cnt>=0) update_score(dt, newstate!=0);
  ab_cnt++;
}

void autobaud_begin(int pin) {
  for(int i=0;i<6;i++) {
    ab_score[i] = 0;
    t_bauds[i] = (HAL_CYCLE_COUNT_FREQ() + bauds[i]/2) / bauds[i];
  }
  ab_pin = pin;
  ab_cnt = -1; //need two samples to get first delta time
  pinMode(ab_pin, INPUT);
  attachInterrupt(ab_pin, ab_isr, CHANGE);
}

void autobaud_end() {
  detachInterrupt(ab_pin);
}

//retuns baud rate, or 0 if not determined (yet)
int autobaud_get_baud(int minpulses) {
  int baud = 0;
  if(ab_cnt>=minpulses) {
    for(int i=0;i<6;i++) {
      //check if one (and only one) ab_score is positive
      if(ab_score[i] > 0) {
        if(baud==0) {
          baud = bauds[i];
        }else{
          //found second match, exit
          return 0;
        }
      }
    }
  }
  return baud;
}

//retuns baud rate, or 0 if not determined
int autobaud(int pin, int timeout, int minpulses) {
  int baud = 0;
  uint32_t tstart = millis();
  autobaud_begin(pin);
  while((uint32_t)millis() - tstart < timeout) {
    baud = autobaud_get_baud(minpulses);
    if(baud != 0) break;
  }
  autobaud_end();
  return baud;
}
