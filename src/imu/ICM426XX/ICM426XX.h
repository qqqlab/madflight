/*==========================================================================================
MIT License

Copyright (c) 2026 https://madflight.com

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

#include "../common/SensorDevice.h"
#include "../../hal/hal.h" // pwm module for CLKIN output

class ICM426XX {
private:
  ICM426XX(SensorDevice *dev, uint8_t whoAmI, int pin_clkin);
  void setUserBank(uint8_t bank);
  SensorDevice *dev;
  PWM clkin;

public:
  static bool detect(SensorDevice* dev);
  static ICM426XX* create(SensorDevice *dev, int pin_clkin);
  void read(int16_t *accgyr); //read acc[3],gyr[3]
  const char* type_name();

  uint8_t whoAmI = 0;
  float acc_scale = 1; //Accel scale [G/LSB]
  float gyr_scale = 1; //Gyro scale [dps/LSB]
  uint16_t sampling_rate_hz = 1000;
};