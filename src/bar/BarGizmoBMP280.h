#pragma once

#include "bar.h"
#include "../hal/MF_I2C.h"
#include "BMP280/BMP280.h"

class BarGizmoBMP280: public BarGizmo {
protected:
  Adafruit_BMP280 bar_BMP280;
  float press_old = 0;
public:
  const char* name() override {return "BMP280";}
  BarGizmoBMP280(MF_I2C *i2c, int8_t i2c_adr, uint32_t sample_rate) {
    (void) sample_rate; //TODO use sample_rate
    unsigned status;
    status = bar_BMP280.begin(i2c, i2c_adr, BMP280_CHIPID);
    Serial.printf("BAR: BMP280 i2c_adr=0x%02X SensorID=0x%02X\n", i2c_adr, bar_BMP280.sensorID());

    if (!status) {
      Serial.println(F("Could not find a valid BMP280 gizmo, check wiring or try a different address!"));
      Serial.print("SensorID was: 0x");
      Serial.println(bar_BMP280.sensorID(),16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
    }

    
    //"standard resolution": OSR_T=1x, OSR_P=4x -> measurement rate 83.33 Hz in NORMAL mode, 2.1 Pa RMS noise
    bar_BMP280.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X4,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_OFF,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */
    //return status;
  }

  bool update(float *press, float *temp) override {
    //driver does not return whether data is fresh, return false if pressure did not change
    float press_new = bar_BMP280.readPressure();
    if(press_new == press_old) return false;
    
    press_old = press_new;
    *press = press_new;
    *temp = bar_BMP280.readTemperature();
    return true;
  }
};
