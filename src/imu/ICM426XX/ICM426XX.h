
#pragma once

#include "../MPUxxxx/MPU_interface.h"

class ICM426XX {
protected:
  ICM426XX(MPU_Interface *dev, uint8_t whoAmI);
  void setUserBank(uint8_t bank);
  MPU_Interface *dev;

public:
  static ICM426XX* detect(MPU_Interface *dev);
  void read(int16_t *accgyr); //read acc[3],gyr[3]
  const char* type_name();

  uint8_t whoAmI = 0;
  float acc_scale = 1; //Accel scale [G/LSB]
  float gyr_scale = 1; //Gyro scale [dps/LSB]
  uint16_t sampling_rate_hz = 1000;
};