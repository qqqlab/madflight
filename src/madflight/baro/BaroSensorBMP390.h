#pragma once

#include "baro_interface.h"
#include "../common/MF_I2C.h"
#include "bmp3/bmp3.h"

class BaroSensorBMP390: public BaroSensor {
protected:
  bfs::Bmp3 bmp;
public:
  int setup(MF_I2C *i2c, int8_t i2c_adr, uint32_t sampleRate) override {
    Serial.printf("BARO: BARO_USE_BMP390/BARO_USE_BMP388 BARO_I2C_ADR=0x%02X\n", i2c_adr);
    bmp.Config(i2c, (bfs::Bmp3::I2cAddr)i2c_adr);
    if (!bmp.Begin()) {
      Serial.println("BARO: BARO_USE_BMP390/BARO_USE_BMP388 sensor not found");
      return 1;
    }

    bmp.ConfigFilterCoef(bfs::Bmp3::FILTER_COEF_OFF);

    //set equal or next higher sample rate
    if(sampleRate > 100) {
      //f = 200;
      bmp.ConfigOsMode(bfs::Bmp3::OS_MODE_PRES_1X_TEMP_1X);
    }else if(sampleRate > 50) {
      //f = 100;
      bmp.ConfigOsMode(bfs::Bmp3::OS_MODE_PRES_2X_TEMP_1X);
    }else if(sampleRate > 25) {
      //f = 50;
      bmp.ConfigOsMode(bfs::Bmp3::OS_MODE_PRES_8X_TEMP_1X);
    }else if(sampleRate > 12) {
      //f = 25;
      bmp.ConfigOsMode(bfs::Bmp3::OS_MODE_PRES_16X_TEMP_2X);
    }else {
      //f = 12.5;
      bmp.ConfigOsMode(bfs::Bmp3::OS_MODE_PRES_32X_TEMP_2X); //12.5Hz
    }

    return 0;
  }

  bool update (float *press, float *temp) override {
    if (!bmp.Read()) return false;
    *press = bmp.pres_pa();
    *temp = bmp.die_temp_c();
    return true;
  }
};
