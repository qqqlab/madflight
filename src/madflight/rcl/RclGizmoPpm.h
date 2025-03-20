#pragma once

#include "rcl.h"

void __ISR_getPPM();
volatile uint32_t __ISR_channel[6];
volatile bool __ISR_rcl_updated = false;

class RclGizmoPpm : public RclGizmo {
  private:
    uint16_t *pwm;

  public:
    RclGizmoPpm(int pin, uint16_t *pwm) {
      this->pwm = pwm;

      //Declare interrupt pin
      pinMode(pin, INPUT_PULLUP);
      delay(20);
      //Attach interrupt and point to corresponding ISR function
      attachInterrupt(digitalPinToInterrupt(pin), __ISR_getPPM, RISING); //Only care about rising edge
    }

    bool update() {
      for(int i=0;i<6;i++) pwm[i] = __ISR_channel[i];
      bool rv = __ISR_rcl_updated;
      __ISR_rcl_updated = false;
      return rv;
    }
};

//INTERRUPT SERVICE ROUTINE for reading PPM
void __ISR_getPPM() {
  static uint32_t ppm_counter = 0;
  static uint32_t tpulse_last = 0;
  uint32_t tpulse = micros();
  uint32_t dt_ppm = tpulse - tpulse_last;
  tpulse_last = tpulse;

  if (dt_ppm > 5000) { //Waiting for long pulse to indicate a new pulse train has arrived
    ppm_counter = 0;
  }

  if (ppm_counter >= 1 && ppm_counter <= 6) { //First-Sixth pulse
    __ISR_channel[ppm_counter-1] = dt_ppm;
  }

  if (ppm_counter == 6) { //Sixth pulse
    __ISR_rcl_updated = true;
  }
  
  ppm_counter = ppm_counter + 1;
}
