//TODO: Does not compile on Arduino-ESP32 v3.x.x :  error: 'GPIO' was not declared in this scope -  #define SDA_LO() GPIO.out_w1tc = _sda_mask // Actively drive SDA signal low

/*==========================================================================================
ESP32_SoftWire

ESP32 Arduino bit banged fast I2C library, drop in replacement for Wire.h

The library reaches up to 3 MHz I2C clock speed. No fancy bits and bobs: no timeouts, no clock stretching, blocking only... Made for fast IMU sensor reading were an occasional missed read does not matter, but bus hangups do matter.

Limitation: pins 0-31 only

Tested on ESP32, might work on other ESP32 variants.

Background

As of December 2023 the official arduino-esp32 Wire library has some nasty bugs. In particular, there is a 1 second timeout which hangs the bus on a bad read, and this timeout can not be lowered without changing the source code, see espressif/esp-idf#4999 and espressif/arduino-esp32#5934. So occasionally (a couple times per minute) 1000 samples are missed when you are reading a sensor at 1 kHz.

MIT License

Copyright (c) 2023 https://github.com/qqqlab

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

#pragma once

#include <Arduino.h>

class SoftWire : public Stream {
  public:
    SoftWire();
    //TODO ~SoftWire();
    bool setPins(int sda, int scl); //call setPins() first, so that begin() can be called without arguments from libraries
    bool begin(int sda, int scl, uint32_t frequency=0); // returns true, if successful init of i2c bus
    inline bool begin() { return begin(-1, -1, static_cast<uint32_t>(0)); } // Explicit Overload for Arduino MainStream API compatibility
    //TODO bool end();
    //TODO size_t setBufferSize(size_t bSize);
    //TODO void setTimeOut(uint16_t timeOutMillis); // default timeout of i2c transactions is 50ms
    //TODO uint16_t getTimeOut();
    bool setClock(uint32_t frequency);  // Approximate frequency in Hz
    //TODO uint32_t getClock();
    uint8_t beginTransmission(uint8_t address);
    size_t write(uint8_t data);
    uint8_t endTransmission(bool sendStop = true);
    size_t requestFrom(uint8_t address, size_t len, bool stopBit=true);
    int available(void);
    int read(void);
    int peek(void);
    void flush(void);
    size_t write(const uint8_t *data, size_t n);
    
    inline size_t write(const char * s) { return write((uint8_t*) s, strlen(s)); }
    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n) { return write((uint8_t)n); }
    inline size_t write(unsigned int n) { return write((uint8_t)n); }
    inline size_t write(int n) { return write((uint8_t)n); }
    
  private:
    uint8_t _sda;
    uint8_t _scl;
    uint32_t _scl_mask;
    uint32_t _sda_mask;
    bool _started;
    uint8_t _data[256];
    uint8_t _data_len;
    uint8_t _data_i;
    uint8_t _ack;
    uint32_t _delay_cycles; //delay half frequency period
    void _ll_start_cond(void);
    void _ll_stop_cond(void);
    void _ll_write_bit(bool bit);
    uint8_t _ll_read_bit(void);
    uint8_t _ll_write_byte(uint8_t byte);
    uint8_t _ll_read_byte(bool ack); 

    void _delay();
};


#if defined ARDUINO_ARCH_ESP32
/*==========================================================================================
ESP32_SoftWire

ESP32 Arduino bit banged fast I2C library, drop in replacement for Wire.h

The library reaches up to 3 MHz I2C clock speed. No fancy bits and bobs: no timeouts, no clock stretching, blocking only... Made for fast IMU sensor reading were an occasional missed read does not matter, but bus hangups do matter.

Limitation: pins 0-31 only

Tested on ESP32, might work on other ESP32 variants.

Background

As of December 2023 the official arduino-esp32 Wire library has some nasty bugs. In particular, there is a 1 second timeout which hangs the bus on a bad read, and this timeout can not be lowered without changing the source code, see espressif/esp-idf#4999 and espressif/arduino-esp32#5934. So occasionally (a couple times per minute) 1000 samples are missed when you are reading a sensor at 1 kHz.

MIT License

Copyright (c) 2023 https://github.com/qqqlab

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

//-----------------------------------------------------------------------------------------
// I2C cheat sheet: M() master sends, S() slave sends
// Write: M(start) M(adr|1)S(ack) M(data)S(ack) M(data)S(nack/ack) M(stop)
// Read:  M(start) M(adr|0)S(ack) S(data)M(ack) S(data)M(nack) M(stop)
//-----------------------------------------------------------------------------------------

#include "ESP32_SoftWire.h"
#include <soc/gpio_struct.h>

#define SDA_HI() GPIO.out_w1ts = _sda_mask // Do not drive SDA (set pin high-impedance)
#define SDA_LO() GPIO.out_w1tc = _sda_mask // Actively drive SDA signal low
#define SDA_IN() (GPIO.in & _sda_mask ? 1 : 0) // Return current level of SCL line, 0 or 1
#define SCL_HI() GPIO.out_w1ts = _scl_mask  // Do not drive SCL (set pin high-impedance)
#define SCL_LO() GPIO.out_w1tc = _scl_mask // Actively drive SCL signal low
#define SCL_IN() (GPIO.in & _scl_mask ? 1 : 0) // Return current level of SCL line, 0 or 1

#define DELAY_OVERHEAD_CYCLES 26  //26 cycles overhead for PIN_CLR(); _delay();

SoftWire::SoftWire() {
  _sda = -1;
  _scl = -1;
  _scl_mask = 0;
  _sda_mask = 0;
  _started = false; 
  setClock(100000);
  _ll_stop_cond();
}

bool SoftWire::setPins(int sda, int scl) {
  if(sda<0 || sda>31 || scl<0 || scl>31) return false;
  pinMode(sda, OUTPUT_OPEN_DRAIN);
  pinMode(scl, OUTPUT_OPEN_DRAIN);
  _sda_mask = (1<<sda);
  _scl_mask = (1<<scl);
  _sda = sda;
  _scl = scl;
  return true;
}

bool SoftWire::begin(int sda, int scl, uint32_t frequency) {
  setPins(sda, scl);
  setClock(frequency);
  return true;
}

bool SoftWire::setClock(uint32_t frequency) {
  if(frequency == 0) return false;
  _delay_cycles = ESP.getCpuFreqMHz() * 500000 / frequency;
  if(_delay_cycles>DELAY_OVERHEAD_CYCLES) _delay_cycles -= DELAY_OVERHEAD_CYCLES; else _delay_cycles = 0;
  return true;
}

void SoftWire::_delay() {
  uint32_t t = ESP.getCycleCount();
  while(ESP.getCycleCount() - t < _delay_cycles);
}

uint8_t SoftWire::beginTransmission(uint8_t address) {
  _ll_start_cond();
  _ack = _ll_write_byte(address<<1);
  return _ack;
}

size_t SoftWire::write(uint8_t data) {
  _ack = _ll_write_byte(data);
  return _ack;
}

uint8_t SoftWire::endTransmission(bool sendStop) {
  if(sendStop) {
    _ll_stop_cond();
  }
  return _ack;
}

size_t SoftWire::requestFrom(uint8_t address, size_t len, bool stopBit) {
  int i;
  _ll_start_cond();
  _ll_write_byte(address<<1 | 1);
  for(i=0; i<len-1; i++) {
    _data[i] = _ll_read_byte(true);
  }
  _data[i++] = _ll_read_byte(false);
  if(stopBit) _ll_stop_cond();
  _data_len = i;
  _data_i = 0;
  return i;
}

int SoftWire::available(void) {
  return _data_len - _data_i;
}

int SoftWire::read(void) {
  return _data[_data_i++];
}

int SoftWire::peek(void) {
  return _data[_data_i];
} 

void SoftWire::flush(void) { 
  _data_len = 0;
  _data_i = 0;
} 

size_t SoftWire::write(const uint8_t *data, size_t n) {
  size_t cnt = 0;
  for(size_t i=0;i<n;i++) {
    cnt += write(data[i]);
  }
  return cnt;
}

//========================================================================
// Low Level Bit-Bang I2C
//========================================================================

//TODO void arbitration_lost(void);

void SoftWire::_ll_start_cond(void) {
  if (_started) { 
    // if started, do a restart condition
    // set SDA to 1
    SDA_HI();
    _delay();
    SCL_HI();
    
//TODO    while (SCL_IN() == 0) { // Clock stretching
//TODO       // You should add timeout to this loop
//TODO     }

    // Repeated start setup time, minimum 4.7us
    _delay();
  }

//TODO   if (SDA_IN() == 0) {
//TODO     arbitration_lost();
//TODO   }

  // SCL is high, set SDA from 1 to 0.
  SDA_LO();
  _delay();
  SCL_LO();
  _started = true;
}

void SoftWire::_ll_stop_cond(void) {
  // set SDA to 0
  SDA_LO();
  _delay();

  SCL_HI();
  
//TODO   while (SCL_IN() == 0) { // Clock stretching
//TODO     // add timeout to this loop.
//TODO   }

  // Stop bit setup time, minimum 4us
  _delay();

  // SCL is high, set SDA from 0 to 1
  SDA_HI();
  _delay();

//TODO   if (SDA_IN() == 0) {
//TODO     arbitration_lost();
//TODO   }

  _started = false;
}

// Write a bit to I2C bus
void SoftWire::_ll_write_bit(bool bit) {
  if (bit) {
    SDA_HI();
  } else {
    SDA_LO();
  }

  // SDA change propagation delay
  _delay();

  // Set SCL high to indicate a new valid SDA value is available
  SCL_HI();

  // Wait for SDA value to be read by target, minimum of 4us for standard mode
  _delay();

//TODO   while (SCL_IN() == 0) { // Clock stretching
//TODO     // You should add timeout to this loop
//TODO   }

  // SCL is high, now data is valid
  // If SDA is high, check that nobody else is driving SDA
//TODO   if (bit && (SDA_IN() == 0)) {
//TODO     arbitration_lost();
//TODO   }

  // Clear the SCL to low in preparation for next change
  SCL_LO();
}

// Read a bit from I2C bus
uint8_t SoftWire::_ll_read_bit(void) {
  uint8_t bit;

  // Let the target drive data
  SDA_HI();

  // Wait for SDA value to be written by target, minimum of 4us for standard mode
  _delay();

  // Set SCL high to indicate a new valid SDA value is available
  SCL_HI();

//TODO   while (SCL_IN() == 0) { // Clock stretching
//TODO     // You should add timeout to this loop
//TODO   }

  // Wait for SDA value to be written by target, minimum of 4us for standard mode
  _delay();

  // SCL is high, read out bit (0=sda low, 1=sda high)
  bit = (SDA_IN() ? 1 : 0);

  // Set SCL low in preparation for next operation
  SCL_LO();

  return bit;
}

// Write a byte to I2C bus. Return 1 if ack by the target.
uint8_t SoftWire::_ll_write_byte(uint8_t byte) {
  for (uint8_t bit = 0; bit < 8; ++bit) {
    _ll_write_bit((byte & 0x80) != 0);
    byte <<= 1;
  }

  bool nack = _ll_read_bit(); //ack=sda low, nack=sda high

  return (nack ? 1 : 0);
}

// Read a byte from I2C bus
uint8_t SoftWire::_ll_read_byte(bool ack) {
  uint8_t byte = 0;
  uint8_t bit;

  for (bit = 0; bit < 8; ++bit) {
    byte = (byte << 1) | _ll_read_bit();
  }

  _ll_write_bit(!ack);

  return byte;
}
#endif
