/*==========================================================================================
MIT License

Copyright (c) 2023-2025 https://madflight.com

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

#include "common.h"
#include <Arduino.h>
#include "../led/led.h"
#include "../rcl/rcl.h"
#include "../cli/cli.h"

//lowpass frequency to filter beta constant
float lowpass_to_beta(float f0, float fs) {
  return constrain(1 - exp(-2 * PI * f0 / fs), 0.0f, 1.0f);
}

void madflight_panic(String msg) {
  bool do_print = true;
  led.enabled = true;
  for(;;) {
    if(do_print) Serial.print("FATAL ERROR: " + msg + " Press enter to start CLI...\n");
    for(int i = 0; i < 20; i++) {
      led.toggle();
      uint32_t ts = millis();
      while(millis() - ts < 50) {
        if(cli.update()) do_print = false; //process CLI commands, stop error output after first command
        rcl.update(); //keep rcl (mavlink?) running
      } 
    }
  }
}
void madflight_warn(String msg) { 
  Serial.print("WARNING: " + msg + "\n");
  //flash LED for 1 second
  for(int i = 0; i < 20; i++) {
    led.toggle();
    delay(50);
  }
}

void madflight_warn_or_panic(String msg, bool panic) {
  if(panic) {
    madflight_panic(msg);
  }else{
    madflight_warn(msg);
  }
}
