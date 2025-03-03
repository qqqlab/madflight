#pragma once

class MF_I2C {
  public:
    virtual void begin() = 0;
    virtual void end() = 0;
    virtual void setClock(uint32_t freq) = 0;
    virtual void beginTransmission(uint8_t address) = 0;
    virtual uint8_t endTransmission(bool stopBit) = 0;
    virtual size_t requestFrom(uint8_t address, size_t len, bool stopBit) = 0;
    virtual size_t read(uint8_t *buf, size_t len) = 0;
    virtual size_t write(const uint8_t *buf, size_t len) = 0;

    uint8_t read() {
      uint8_t data;
      read(&data, 1);
      return data;
    }

    void write(uint8_t data) {
      write(&data, 1);
    }

    uint8_t endTransmission(void) {
      return endTransmission(true);
    }
    
    size_t requestFrom(uint8_t address, size_t len) {
      return requestFrom(address, len, true);
    }


    //non "standard" extension 
    size_t transceive(uint8_t address, uint8_t* wbuf, size_t wlen, uint8_t* rbuf, size_t rlen) {
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


// Wrapper around an arduino TwoWire-like pointer
template<class T>
class MF_I2CPtrWrapper : public MF_I2C {
  protected:
    T _i2c;
  public:
    MF_I2CPtrWrapper(T i2c) { _i2c = i2c; }
    void begin()                            override { _i2c->begin(); }
    void end()                              override { _i2c->end(); }
    void setClock(uint32_t freq)            override { _i2c->setClock(freq); }
    void beginTransmission(uint8_t address) override { _i2c->beginTransmission(address); }
    uint8_t endTransmission(bool stopBit)   override { return _i2c->endTransmission(stopBit); }
    size_t requestFrom(uint8_t address, size_t len, bool stopBit) 
                                            override { return _i2c->requestFrom(address, len, stopBit); }
    size_t read(uint8_t *buf, size_t len)   override { return _i2c->readBytes(buf, len); }
    size_t write(const uint8_t *buf, size_t len)  override { return _i2c->write(buf, len); }
};

class MF_I2CNone : public MF_I2C {
  public:
    void begin()                            override { }
    void end()                              override { }
    void setClock(uint32_t freq)            override { }
    void beginTransmission(uint8_t address) override { }
    uint8_t endTransmission(bool stopBit)   override { (void)stopBit; return 0; }
    size_t requestFrom(uint8_t address, size_t len, bool stopBit) 
                                            override { (void)address; (void)len; (void)stopBit; return 0; }
    size_t read(uint8_t *buf, size_t len)   override { (void)buf; (void)len; return 0; }
    size_t write(const uint8_t *buf, size_t len)  override { (void)buf; (void)len; return 0; }
};



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