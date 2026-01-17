// madflight https://github.com/qqqlab/madflight
// Interface for MPU sensors

#pragma once

#include "../../hal/MF_I2C.h"
#include <Arduino.h>
#include <SPI.h>

class MPU_Interface {
  public:
    virtual ~MPU_Interface() {}
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
      //Serial.printf("MPU.readReg(0x%02X)=0x%02X\n", reg, data);
      return data;
    }
};

//================================================================
// SPI
//================================================================
class MPU_InterfaceSPI : public MPU_Interface {
  private:
    SPIClass * _spi;
    int _freq;
    uint8_t _spi_cs;
    uint8_t _spi_mode;

  public:
    MPU_InterfaceSPI(SPIClass *spi, uint8_t cs, uint8_t spi_mode = SPI_MODE3) {
      _spi = spi; 
      _spi_cs = cs;
      _spi_mode = spi_mode;
      pinMode(_spi_cs, OUTPUT);
      digitalWrite(_spi_cs, HIGH);
      setFreq(1000000); //default to 1MHz
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

class MPU_InterfaceI2C : public MPU_Interface{
  private:
    MF_I2C *_i2c;
    uint8_t _i2c_adr;

  public:
    MPU_InterfaceI2C(MF_I2C *i2c, uint8_t i2c_adr) {
      _i2c = i2c;
      _i2c_adr = i2c_adr;
      setFreq(100000); //default to 100kHz
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
