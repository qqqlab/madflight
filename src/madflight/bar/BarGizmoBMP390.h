#pragma once

#include "bar.h"
#include "../hal/MF_I2C.h"
#include "bmp3/bmp3.h"

class BarGizmoBMP390: public BarGizmo {
protected:
  bfs::Bmp3 bmp;
public:
  BarGizmoBMP390(MF_I2C *i2c, int8_t i2c_adr, uint32_t sampleRate) {
    Serial.printf("BAR: BBMP390/BMP388 I2C_ADR=0x%02X\n", i2c_adr);
    bmp.Config(i2c, (bfs::Bmp3::I2cAddr)i2c_adr);
    if (!bmp.Begin()) {
      Serial.println("BAR: BMP390/BMP388 gizmo not found");
      return;
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
  }

  bool update (float *press, float *temp) override {
    if (!bmp.Read()) return false;
    *press = bmp.pres_pa();
    *temp = bmp.die_temp_c();
    return true;
  }
};
