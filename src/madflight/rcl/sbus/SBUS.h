#pragma once

#include "Arduino.h"
#include "elapsedMillis.h"
#include "../../hal/MF_Serial.h"

class SBUS{
  public:
    MF_Serial* _bus;

    void begin();
    bool read(uint16_t* channels, bool* failsafe, bool* lostFrame);
  private:
    bool parse();
    const uint32_t _sbusBaud = 100000;
    static const uint8_t _numChannels = 16;
    const uint8_t _sbusHeader = 0x0F;
    const uint8_t _sbusFooter = 0x00;
    const uint8_t _sbus2Footer = 0x04;
    const uint8_t _sbus2Mask = 0x0F;
    const uint32_t SBUS_TIMEOUT_US = 7000;
    uint8_t _parserState, _prevByte = _sbusFooter, _curByte;
    static const uint8_t _payloadSize = 24;
    uint8_t _payload[_payloadSize];
    const uint8_t _sbusLostFrame = 0x04;
    const uint8_t _sbusFailSafe = 0x08;
};
