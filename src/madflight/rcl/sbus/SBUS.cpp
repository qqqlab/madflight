#include "SBUS.h"

/* starts the serial communication */
void SBUS::begin()
{
  // initialize parsing state
  _parserState = 0;

  //_bus->begin(_sbusBaud, SERIAL_8E2); //TODO need SERIAL_8E2
  _bus->begin(_sbusBaud);
}

/* read the SBUS data */
bool SBUS::read(uint16_t* channels, bool* failsafe, bool* lostFrame)
{
  // parse the SBUS packet
  if (parse()) {
    if (channels) {
      // 16 channels of 11 bit data
      channels[0]  = (uint16_t) ((_payload[0]    |_payload[1] <<8)                     & 0x07FF);
      channels[1]  = (uint16_t) ((_payload[1]>>3 |_payload[2] <<5)                     & 0x07FF);
      channels[2]  = (uint16_t) ((_payload[2]>>6 |_payload[3] <<2 |_payload[4]<<10)     & 0x07FF);
      channels[3]  = (uint16_t) ((_payload[4]>>1 |_payload[5] <<7)                     & 0x07FF);
      channels[4]  = (uint16_t) ((_payload[5]>>4 |_payload[6] <<4)                     & 0x07FF);
      channels[5]  = (uint16_t) ((_payload[6]>>7 |_payload[7] <<1 |_payload[8]<<9)      & 0x07FF);
      channels[6]  = (uint16_t) ((_payload[8]>>2 |_payload[9] <<6)                     & 0x07FF);
      channels[7]  = (uint16_t) ((_payload[9]>>5 |_payload[10]<<3)                     & 0x07FF);
      channels[8]  = (uint16_t) ((_payload[11]   |_payload[12]<<8)                     & 0x07FF);
      channels[9]  = (uint16_t) ((_payload[12]>>3|_payload[13]<<5)                     & 0x07FF);
      channels[10] = (uint16_t) ((_payload[13]>>6|_payload[14]<<2 |_payload[15]<<10)    & 0x07FF);
      channels[11] = (uint16_t) ((_payload[15]>>1|_payload[16]<<7)                     & 0x07FF);
      channels[12] = (uint16_t) ((_payload[16]>>4|_payload[17]<<4)                     & 0x07FF);
      channels[13] = (uint16_t) ((_payload[17]>>7|_payload[18]<<1 |_payload[19]<<9)     & 0x07FF);
      channels[14] = (uint16_t) ((_payload[19]>>2|_payload[20]<<6)                     & 0x07FF);
      channels[15] = (uint16_t) ((_payload[20]>>5|_payload[21]<<3)                     & 0x07FF);
    }
    if (lostFrame) {
      // count lost frames
      if (_payload[22] & _sbusLostFrame) {
        *lostFrame = true;
      } else {
        *lostFrame = false;
      }
    }
    if (failsafe) {
      // failsafe state
      if (_payload[22] & _sbusFailSafe) {
          *failsafe = true;
      }
      else{
          *failsafe = false;
      }
    }
    // return true on receiving a full packet
    return true;
    } else {
    // return false if a full packet is not received
    return false;
  }
}

/* parse the SBUS data */
bool SBUS::parse()
{
  // reset the parser state if too much time has passed
  static elapsedMicros _sbusTime = 0;
  if (_sbusTime > SBUS_TIMEOUT_US) {_parserState = 0;}
  // see if serial data is available
  while (_bus->available() > 0) {
    _sbusTime = 0;
    _curByte = _bus->read();
    // find the header
    if (_parserState == 0) {
        if ((_curByte == _sbusHeader) && ((_prevByte == _sbusFooter) || ((_prevByte & _sbus2Mask) == _sbus2Footer))) {
          _parserState++;
        } else {
          _parserState = 0;
        }
    } else {
      // strip off the data
      if ((_parserState-1) < _payloadSize) {
        _payload[_parserState-1] = _curByte;
        _parserState++;
      }
      // check the end byte
      if ((_parserState-1) == _payloadSize) {
        if ((_curByte == _sbusFooter) || ((_curByte & _sbus2Mask) == _sbus2Footer)) {
          _parserState = 0;
          return true;
        } else {
          _parserState = 0;
          return false;
        }
      }
    }
    _prevByte = _curByte;
  }
  // return false if a partial packet
  return false;
}
