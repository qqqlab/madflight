#pragma once

#include "baro_interface.h"
#include "../common/MF_I2C.h"
#include "BMP280/BMP280.h"

class BaroSensorBMP280: public BaroSensor {
protected:
  Adafruit_BMP280 baro_BMP280;
public:
  int setup(MF_I2C *i2c, int8_t i2c_adr, uint32_t sampleRate) override {
    (void) sampleRate; //TODO use sampleRate
    unsigned status;
    status = baro_BMP280.begin(i2c, i2c_adr, BMP280_CHIPID);
    Serial.printf("BARO: BARO_USE_BMP280 BARO_I2C_ADR=0x%02X SensorID=0x%02X\n", i2c_adr, baro_BMP280.sensorID());

    if (!status) {
      Serial.println(F("Could not find a valid BMP280 sensor, check wiring or try a different address!"));
      Serial.print("SensorID was: 0x");
      Serial.println(baro_BMP280.sensorID(),16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
    }

    baro_BMP280.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X1,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_OFF,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */
    return status;
  }

  bool update(float *press, float *temp) override {
    //driver does not return whether data is fresh, return true if pressure changed
    float press_new = baro_BMP280.readPressure();
    bool rv = (press_new != *press);
    *press = press_new;
    *temp = baro_BMP280.readTemperature();
    return rv;
  }
};
