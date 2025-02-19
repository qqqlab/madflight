#include "common.h"
#include <Arduino.h>

//lowpass frequency to filter beta constant
float lowpass_to_beta(float f0, float fs) {
  return constrain(1 - exp(-2 * PI * f0 / fs), 0.0f, 1.0f);
}