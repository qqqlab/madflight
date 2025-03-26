#include "SBUS.h"

/*
sbus is a 25 byte frame:

<header byte 0x0F> <22 bytes 16*11 bit channel data> <flags byte> <footer byte 0x00>

*/


static const uint32_t _sbusBaud = 100000;
static const uint8_t _numChannels = 16;
static const uint8_t _sbusHeader = 0x0F;
static const uint8_t _sbusFooter = 0x00;
static const uint8_t _sbus2Footer = 0x04;
static const uint8_t _sbus2Mask = 0x0F;
static const uint8_t _payloadSize = 24;
static const uint8_t _sbusLostFrame = 0x04;
static const uint8_t _sbusFailSafe = 0x08;

/* starts the serial communication */
void SBUS::begin()
{
  // initialize parsing state
  _parserState = 0;

  //serial->begin(_sbusBaud, SERIAL_8E2); //NOTE: bus needs to be setup by the caller before calling SBUS::begin()
}

// read the SBUS data, return number of packets received
int SBUS::read(uint16_t* channels, bool* failsafe, bool* lostFrame)
{
  int cnt = 0;

  //parse all incoming serial data
  while (parse()) {
    cnt++;

    if (channels) {
      // 16 channels of 11 bit data
      channels[0]  = (uint16_t) ((_payload[0]    |_payload[1] <<8)                     & 0x07FF);
      channels[1]  = (uint16_t) ((_payload[1]>>3 |_payload[2] <<5)                     & 0x07FF);
      channels[2]  = (uint16_t) ((_payload[2]>>6 |_payload[3] <<2 |_payload[4]<<10)    & 0x07FF);
      channels[3]  = (uint16_t) ((_payload[4]>>1 |_payload[5] <<7)                     & 0x07FF);
      channels[4]  = (uint16_t) ((_payload[5]>>4 |_payload[6] <<4)                     & 0x07FF);
      channels[5]  = (uint16_t) ((_payload[6]>>7 |_payload[7] <<1 |_payload[8]<<9)     & 0x07FF);
      channels[6]  = (uint16_t) ((_payload[8]>>2 |_payload[9] <<6)                     & 0x07FF);
      channels[7]  = (uint16_t) ((_payload[9]>>5 |_payload[10]<<3)                     & 0x07FF);
      channels[8]  = (uint16_t) ((_payload[11]   |_payload[12]<<8)                     & 0x07FF);
      channels[9]  = (uint16_t) ((_payload[12]>>3|_payload[13]<<5)                     & 0x07FF);
      channels[10] = (uint16_t) ((_payload[13]>>6|_payload[14]<<2 |_payload[15]<<10)   & 0x07FF);
      channels[11] = (uint16_t) ((_payload[15]>>1|_payload[16]<<7)                     & 0x07FF);
      channels[12] = (uint16_t) ((_payload[16]>>4|_payload[17]<<4)                     & 0x07FF);
      channels[13] = (uint16_t) ((_payload[17]>>7|_payload[18]<<1 |_payload[19]<<9)    & 0x07FF);
      channels[14] = (uint16_t) ((_payload[19]>>2|_payload[20]<<6)                     & 0x07FF);
      channels[15] = (uint16_t) ((_payload[20]>>5|_payload[21]<<3)                     & 0x07FF);
    }

    if (lostFrame) {
      // count lost frames
      *lostFrame = ((_payload[22] & _sbusLostFrame) != 0);
    }

    if (failsafe) {
      // failsafe state
      *failsafe = ((_payload[22] & _sbusFailSafe) != 0);
    }
  }

  return cnt;
}

// parse the SBUS data, return true if packet was received
bool SBUS::parse()
{
  bool rv = false;
  uint8_t b;
  while ( serial->read(&b, 1) ) {
    if (_parserState == 0) {
        // find the header
        if ((b == _sbusHeader) && ((_prevByte == _sbusFooter) || ((_prevByte & _sbus2Mask) == _sbus2Footer))) {
          _parserState++;
        }
    } else if (_parserState < 24) {
        // strip off the data
        _payload[_parserState-1] = b;
        _parserState++;
    } else if (_parserState == 24) {
        // check the end byte
        rv = (b == _sbusFooter) || ((b & _sbus2Mask) == _sbus2Footer);
        _parserState = 0;
    } else {
       //should not get here
      _parserState = 0;
    }
    _prevByte = b;
  }

  return rv;
}
