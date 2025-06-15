// madflight https://github.com/qqqlab/madflight
// Interface for MPU and ICM sensors

#pragma once

#include "../../hal/MF_I2C.h"

#include "Arduino.h"
#include <SPI.h>

//Datasheet: spi clock up to 1MHz for register operations, up to 20MHz allowed for reading data.
#define ICM_SPI_FREQ_LOW 1000000
#define ICM_SPI_FREQ_HIGH 20000000

//Datasheet: i2c clock up to 400kHz
#define ICM_I2C_FREQ_LOW 400000
#define ICM_I2C_FREQ_HIGH 1000000 //2.5 times overclocking

class ICM_Interface {
  public:
    virtual ~ICM_Interface() {};

    int freqLow;
    int freqHigh;

    virtual void setFreq(int freq) = 0;

    virtual unsigned int write_register( uint8_t reg, uint8_t data ) = 0;
    
    virtual void read_registers( uint8_t reg, uint8_t *data, uint8_t n ) = 0;

    unsigned int ReadReg(uint8_t reg) {
      uint8_t data = 0;
      read_registers(reg, &data, 1);
      return data;
    }

    inline void setFreqLow() {
      setFreq(freqLow);
    }

    inline void setFreqHigh() {
      setFreq(freqHigh);
    }
};

//================================================================
// SPI
//================================================================
class ICM_InterfaceSPI : public ICM_Interface {
  public:
    ICM_InterfaceSPI(SPIClass *spi, uint8_t cs) {
      _spi = spi; 
      _spi_cs = cs;
      freqLow = ICM_SPI_FREQ_LOW;
      freqHigh = ICM_SPI_FREQ_HIGH;
      setFreq(freqLow);
      pinMode(_spi_cs, OUTPUT);
      digitalWrite(_spi_cs, HIGH);
    }

    void setFreq(int freq) override {
      _freq = freq;
    }

    virtual unsigned int write_register( uint8_t reg, uint8_t data ) override {
      _spi->beginTransaction(SPISettings(_freq, MSBFIRST, SPI_MODE3));
      digitalWrite(_spi_cs, LOW);
      _spi->transfer(reg & 0x7f);
      unsigned int temp_val = _spi->transfer(data);
      digitalWrite(_spi_cs, HIGH);
      _spi->endTransaction();
      return temp_val;
    }

    virtual void read_registers( uint8_t reg, uint8_t *data, uint8_t n ) override {
      _spi->beginTransaction(SPISettings(_freq, MSBFIRST, SPI_MODE3));
      digitalWrite(_spi_cs, LOW);
      _spi->transfer(reg | 0x80);
      for(int i = 0; i < n; i++) {
        data[i] = _spi->transfer(0x00);
      }
      digitalWrite(_spi_cs, HIGH);
      _spi->endTransaction();
    }

private:
    SPIClass * _spi;
    int _freq;  
    uint8_t _spi_cs;
    unsigned int _WriteReg_SPI(uint8_t reg, uint8_t data);
    void _ReadRegs_SPI(uint8_t reg, uint8_t *buf, int n);
};

//================================================================
// I2C
//================================================================

class ICM_InterfaceI2C : public ICM_Interface{
  public:
    ICM_InterfaceI2C(MF_I2C *i2c, uint8_t i2c_adr) {
      _i2c = i2c;
      _i2c_adr = i2c_adr;
      freqLow = ICM_I2C_FREQ_LOW;
      freqHigh = ICM_I2C_FREQ_HIGH;
      setFreq(freqLow);
    }

    void setFreq(int freq) override {
      _i2c->setClock(freq);
    }

    virtual unsigned int write_register( uint8_t reg, uint8_t data ) override {
      _i2c->beginTransmission(_i2c_adr); 
      _i2c->write(reg);
      _i2c->write(data);
      _i2c->endTransmission();
      //Serial.printf("write_register(reg0x%02X, data=0x%02X) --> ", reg, data); ReadReg(reg);
      return 0;
    }

    virtual void read_registers( uint8_t reg, uint8_t *data, uint8_t n ) override {
      _i2c->beginTransmission(_i2c_adr); 
      _i2c->write(reg);
      _i2c->endTransmission(false); //false = repeated start
      uint8_t bytesReceived = _i2c->requestFrom(_i2c_adr, n);
      if(bytesReceived == n) {
        _i2c->readBytes(data, bytesReceived);
      }
      //Serial.printf("ReadRegs(reg=0x%02X, n=%d) --> data[%d]=0x%02X\n", reg, n, bytesReceived, data[0]);
    }

private:
    MF_I2C *_i2c;
    uint8_t _i2c_adr;
    unsigned int _WriteReg_I2C(uint8_t reg, uint8_t data);
    void _ReadRegs_I2C(uint8_t reg, uint8_t *buf, uint8_t n);
};



