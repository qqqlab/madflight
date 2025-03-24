#pragma once

#include <stdint.h>

enum class MF_SerialMode {
  mf_SERIAL_8N1, 
  mf_SERIAL_8E2, //for SBUS
};

// Serial Interface
class MF_Serial {
  public:
    virtual void begin(int baud) = 0;
    virtual int available() = 0;
    virtual int availableForWrite() = 0;
    virtual int read(uint8_t *buf, int len) = 0;
    virtual int write(uint8_t *buf, int len) = 0;

    int read() {
      uint8_t data;
      if(read(&data, 1)==0) return -1;
      return data;
    }
};

// Wrapper around an arduino HardwareSerial-like pointer T
template<class T>
class MF_SerialPtrWrapper : public MF_Serial {
  public:
    T _serial;
    MF_SerialPtrWrapper(T serial) { _serial = serial; }
    void begin(int baud)             override { _serial->begin(baud); }
    int available()                  override { return _serial->available(); }
    int availableForWrite()          override { return _serial->availableForWrite(); }
    int read(uint8_t *buf, int len)  override { return _serial->readBytes(buf, len); }
    int write(uint8_t *buf, int len) override { return _serial->write(buf, len); }
};

/*
============ Template for new classes ================

class MF_SerialXXX : public MF_Serial {
  public:
    void begin(int baud)             override { (void)baud; }
    int available()                  override { return 0; }
    int availableForWrite()          override { return 0; }
    int read(uint8_t *buf, int len)  override { (void)buf; (void)len; return 0; }
    int write(uint8_t *buf, int len) override { (void)buf; (void)len; return 0; }
};

*/

/*--------------------------------------------------------------------------------------
# HardwareSerial implementations on different platforms



## RP2040

reading is buffered
writing blocks

https://github.com/earlephilhower/ArduinoCore-API/blob/master/api/HardwareSerial.h
    virtual void begin(unsigned long) = 0;
    virtual int available(void) = 0;
    using Print::write; // pull in write(str) and write(buf, size) from Print

https://github.com/earlephilhower/ArduinoCore-API/blob/master/api/Print.h
    virtual size_t write(const uint8_t *buffer, size_t size);
    // default to zero, meaning "a single write may block"
    // should be overridden by subclasses with buffering
    virtual int availableForWrite() { return 0; }

https://github.com/earlephilhower/ArduinoCore-API/blob/master/api/Stream.h
    size_t readBytes( uint8_t *buffer, size_t length) { return readBytes((char *)buffer, length); }



## ESP32

https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/HardwareSerial.h
    void begin(
      unsigned long baud, uint32_t config = SERIAL_8N1, int8_t rxPin = -1, int8_t txPin = -1, bool invert = false, unsigned long timeout_ms = 20000UL,
      uint8_t rxfifo_full_thrhd = 120
    );
    int available(void);
    int availableForWrite(void);
    size_t read(uint8_t *buffer, size_t size);
    size_t readBytes(uint8_t *buffer, size_t length);
    size_t write(const uint8_t *buffer, size_t size);

    bool setPins(int8_t rxPin, int8_t txPin, int8_t ctsPin = -1, int8_t rtsPin = -1);
    size_t setRxBufferSize(size_t new_size);
    size_t setTxBufferSize(size_t new_size);



## STM32

Buffer size set by defines

https://github.com/stm32duino/Arduino_Core_STM32/blob/main/cores/arduino/HardwareSerial.h

class HardwareSerial : public Stream {
    void begin(unsigned long baud)
    virtual int available(void);
    int availableForWrite(void);
    size_t write(const uint8_t *buffer, size_t size);
    
    void setRx(uint32_t _rx);
    void setTx(uint32_t _tx);
    
https://github.com/stm32duino/Arduino_Core_STM32/blob/main/cores/arduino/Stream.h
    virtual int available() = 0;
    size_t readBytes(uint8_t *buffer, size_t length)
*/