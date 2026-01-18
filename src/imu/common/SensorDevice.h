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

// Interface for SPI/I2C sensors

#pragma once

#include <Arduino.h>
#include <SPI.h>
#include "../../hal/MF_I2C.h"
#include "../imu.h"

class SensorDevice {
  public:
    virtual ~SensorDevice() {}
    virtual void setFreq(int freq) = 0;
    virtual uint32_t writeRegs( uint8_t reg, uint8_t *data, uint16_t n ) = 0;
    virtual void readRegs( uint8_t reg, uint8_t *data, uint16_t n ) = 0;
    virtual bool isSPI() = 0;

    uint32_t writeReg( uint8_t reg, uint8_t data ) {
      return writeRegs(reg, &data, 1);
    }

    uint8_t readReg(uint8_t reg) {
      uint8_t data = 0;
      readRegs(reg, &data, 1);
      return data;
    }

    void setLowSpeed() {
      if(isSPI()) {
        setFreq(1000000); 
      }else{
        setFreq(100000); 
      }
    }

    static SensorDevice* createImuDevice(ImuConfig *config);
};

//================================================================
// SPI
//================================================================
class SensorDeviceSPI : public SensorDevice {
  private:
    SPIClass * _spi;
    int _freq;
    uint8_t _spi_cs;
    uint8_t _spi_mode;

  public:
    SensorDeviceSPI(SPIClass *spi, uint8_t cs, uint8_t spi_mode = SPI_MODE3) {
      _spi = spi; 
      _spi_cs = cs;
      _spi_mode = spi_mode;
      pinMode(_spi_cs, OUTPUT);
      digitalWrite(_spi_cs, HIGH);
      setLowSpeed();
    }

    void setFreq(int freq) override {
      _freq = freq;
    }

    uint32_t writeRegs( uint8_t reg, uint8_t *data, uint16_t n ) override {
      _spi->beginTransaction(SPISettings(_freq, MSBFIRST, _spi_mode));
      digitalWrite(_spi_cs, LOW);
      _spi->transfer(reg & 0x7f);
      uint32_t rv = 0;
      for(uint32_t i = 0; i < n; i++) {
        rv += _spi->transfer(data[i]);
      }
      digitalWrite(_spi_cs, HIGH);
      _spi->endTransaction();
      return rv;
    }

    void readRegs( uint8_t reg, uint8_t *data, uint16_t n ) override {
      _spi->beginTransaction(SPISettings(_freq, MSBFIRST, _spi_mode));
      digitalWrite(_spi_cs, LOW);
      _spi->transfer(reg | 0x80);
      for(uint32_t i = 0; i < n; i++) {
        data[i] = _spi->transfer(0x00);
      }
      digitalWrite(_spi_cs, HIGH);
      _spi->endTransaction();
    }

    bool isSPI() {
      return true;
    }
};

//================================================================
// I2C
//================================================================

class SensorDeviceI2C : public SensorDevice{
  private:
    MF_I2C *_i2c;
    uint8_t _i2c_adr;

  public:
    SensorDeviceI2C(MF_I2C *i2c, uint8_t i2c_adr) {
      _i2c = i2c;
      _i2c_adr = i2c_adr;
      setLowSpeed();
    }

    void setFreq(int freq) override {
      _i2c->setClock(freq);
    }

    uint32_t writeRegs( uint8_t reg, uint8_t *data, uint16_t n ) override {
      _i2c->beginTransmission(_i2c_adr); 
      _i2c->write(reg);
      for(uint32_t i = 0; i < n; i++) {
        _i2c->write(data[i]);
      }
      _i2c->endTransmission();
      //Serial.printf("WriteReg(reg0x%02X, data=0x%02X) --> ", reg, data); ReadReg(reg);
      return 0;
    }

    void readRegs( uint8_t reg, uint8_t *data, uint16_t n ) override {
      _i2c->beginTransmission(_i2c_adr); 
      _i2c->write(reg);
      _i2c->endTransmission(false); //false = repeated start
      uint8_t bytesReceived = _i2c->requestFrom(_i2c_adr, n);
      if(bytesReceived == n) {
        _i2c->readBytes(data, bytesReceived);
      }
      //Serial.printf("ReadRegs(reg=0x%02X, n=%d) --> data[%d]=0x%02X\n", reg, n, bytesReceived, data[0]);
    }

    bool isSPI() {
      return false;
    }
};
