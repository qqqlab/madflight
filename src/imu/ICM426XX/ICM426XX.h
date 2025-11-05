
#pragma once

#include "../MPUxxxx/MPU_interface.h"
#include "../../hal/hal.h" // pwm module for CLKIN output

class ICM426XX {
protected:
  ICM426XX(MPU_Interface *dev, uint8_t whoAmI, int pin_clkin);
  void setUserBank(uint8_t bank);
  MPU_Interface *dev;
  PWM clkin;

public:
  static ICM426XX* detect(MPU_Interface *dev, int pin_clkin);
  void read(int16_t *accgyr); //read acc[3],gyr[3]
  const char* type_name();

  uint8_t whoAmI = 0;
  float acc_scale = 1; //Accel scale [G/LSB]
  float gyr_scale = 1; //Gyro scale [dps/LSB]
  uint16_t sampling_rate_hz = 1000;
};