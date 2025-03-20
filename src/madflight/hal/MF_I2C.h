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

    //non "Arduino standard" extension 
    uint32_t transceive(uint8_t address, uint8_t* wbuf, uint32_t wlen, uint8_t* rbuf, uint32_t rlen) {
      beginTransmission(address);
      write(wbuf, wlen);
      endTransmission(false);
      requestFrom(address, rlen);
      return read(rbuf, rlen);
    }

    //virtual void begin(uint8_t address) = 0; 
    //virtual void onReceive(void(*)(int)) = 0;
    //virtual void onRequest(void(*)(void)) = 0;
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
