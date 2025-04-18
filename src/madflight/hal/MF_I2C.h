#pragma once

#include <stdint.h>

class MF_I2C {
  public:
    virtual void begin() = 0;
    virtual void end() = 0;
    virtual void setClock(uint32_t freq) = 0;
    virtual void beginTransmission(uint8_t address) = 0;
    virtual uint8_t endTransmission(bool stopBit) = 0;
    virtual uint32_t requestFrom(uint8_t address, uint32_t len, bool stopBit) = 0;
    virtual uint32_t read(uint8_t *buf, uint32_t len) = 0;
    virtual uint32_t write(const uint8_t *buf, uint32_t len) = 0;

    uint8_t read() {
      uint8_t data;
      read(&data, 1);
      return data;
    }

    uint32_t readBytes(uint8_t *buf, uint32_t len) {
      return read(buf, len);
    }

    uint32_t write(uint8_t data) {
      return write(&data, 1);
    }

    uint8_t endTransmission(void) {
      return endTransmission(true);
    }
    
    uint32_t requestFrom(uint8_t address, uint32_t len) {
      return requestFrom(address, len, true);
    }

    //virtual void begin(uint8_t address) = 0; 
    //virtual void onReceive(void(*)(int)) = 0;
    //virtual void onRequest(void(*)(void)) = 0;
};


class MF_I2CDevice {
  public:
    MF_I2C *i2c = nullptr;
    uint8_t adr = 0;

    MF_I2CDevice(MF_I2C *i2c, uint8_t adr) {
      this->i2c = i2c;
      this->adr = adr;
    }

    uint32_t writeReg(uint8_t reg, uint8_t *data, uint32_t len) {
      i2c->beginTransmission(adr);
      uint32_t rv = i2c->write(&reg, 1);
      if(len > 0) {
        rv += i2c->write(data, len);
      }
      return rv;
    }

    uint32_t writeReg(uint8_t reg, uint8_t data) {
      return writeReg(reg, &data, 1);
    }

    uint32_t readReg(uint8_t reg, uint8_t *data, uint32_t len) {
      uint32_t rv = 0;
      i2c->beginTransmission(adr);
      i2c->write(&reg, 1);
      if(len > 0) {
        i2c->endTransmission(false);
        i2c->requestFrom(adr, len);
        rv = i2c->read(data, len);
      }
      return rv;
    }

    uint8_t readReg(uint8_t reg) {
      uint8_t data;
      readReg(reg, &data, 1);
      return data;
    }

    uint32_t transceive(uint8_t* wbuf, uint32_t wlen, uint8_t* rbuf, uint32_t rlen) {
      uint32_t rv = 0;
      i2c->beginTransmission(adr);
      if(wlen > 0) rv = i2c->write(wbuf, wlen);
      if(wlen > 0 && rlen > 0) i2c->endTransmission(false);
      if(rlen > 0) {
        i2c->requestFrom(adr, rlen);
        rv = i2c->read(rbuf, rlen);
      }
      return rv;
    }
};

// Wrapper around an arduino TwoWire-like pointer T
template<class T>
class MF_I2CPtrWrapper : public MF_I2C {
  protected:
    T _i2c;

  public:
    MF_I2CPtrWrapper(T i2c) {
      _i2c = i2c; 
    }

    void begin() override {
      _i2c->begin(); 
    }

    void end() override {
      _i2c->end(); 
    }

    void setClock(uint32_t freq) override {
      _i2c->setClock(freq); 
    }

    void beginTransmission(uint8_t address) override {
      _i2c->beginTransmission(address);
    }

    uint8_t endTransmission(bool stopBit) override {
      return _i2c->endTransmission(stopBit);
    }

    uint32_t requestFrom(uint8_t address, uint32_t len, bool stopBit) override {
      return _i2c->requestFrom(address, len, stopBit);
    }

    uint32_t read(uint8_t *buf, uint32_t len) override {
      return _i2c->readBytes(buf, len); 
    }

    uint32_t write(const uint8_t *buf, uint32_t len) override {
      return _i2c->write(buf, len);
    }
};

/*
// Dummy interface
class MF_I2CNone : public MF_I2C {
  public:
    void begin()                            override { }
    void end()                              override { }
    void setClock(uint32_t freq)            override { (void)freq; }
    void beginTransmission(uint8_t address) override { (void)address; }
    uint8_t endTransmission(bool stopBit)   override { (void)stopBit; return 0; }
    uint32_t requestFrom(uint8_t address, uint32_t len, bool stopBit) 
                                            override { (void)address; (void)len; (void)stopBit; return 0; }
    uint32_t read(uint8_t *buf, uint32_t len)   override { (void)buf; (void)len; return 0; }
    uint32_t write(const uint8_t *buf, uint32_t len)  override { (void)buf; (void)len; return 0; }
};
*/

/*
Wire.h on different platforms

## ESP32

https://github.com/espressif/arduino-esp32/blob/master/libraries/Wire/src/Wire.h
class TwoWire : public HardwareI2C

## RP2040

https://github.com/earlephilhower/arduino-pico/blob/master/libraries/Wire/src/Wire.h
class TwoWire : public HardwareI2C


## STM32

https://github.com/rogerclarkmelbourne/Arduino_STM32/blob/master/STM32F1/libraries/Wire/Wire.h
class TwoWire : public WireBase
    https://github.com/rogerclarkmelbourne/Arduino_STM32/blob/master/STM32F1/libraries/Wire/utility/WireBase.h
    class WireBase { // Abstraction is awesome!
*/
