#ifdef ARDUINO_ARCH_ESP32

#include "ESP32_PWM_v3.h"

PWM *PWM::channels[PWM_MAX] = {0};


//two channels share the same timer - have also the same freq 
//try first to find a free channel with matching freq
//if not found, then find first free timer
int PWM::findFreeChannel(int freq)
{
  //find free channel with other channel in same group with same req_freq
  for(int i=0;i<PWM_MAX;i++){
    if(!channels[i]) {
      int other_i = (i % 2 == 0 ? i+1 : i-1);
      PWM *other_ch = channels[other_i];
      if(other_ch && other_ch->req_freq == freq) {
        return i;
      }
    }
  }
  //no free channel with matching freq found -> find first free group
  for(int i=0;i<PWM_MAX;i++){
    if(!channels[i]) {
      int other_i = (i % 2 == 0 ? i+1 : i-1);
      PWM *other_ch = channels[other_i];
      if(!other_ch) {
        return i;
      }
    }
  }      
  return -1;
} 

#endif // #ifdef ARDUINO_ARCH_ESP32