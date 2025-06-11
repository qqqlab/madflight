/*==========================================================================================
MIT License

Copyright (c) 2023-2025 https://madflight.com

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

#include "RM3100.h"
#include <Arduino.h> //delay()

#define RM3100_BASE_ADDRESS 0x20 // address with Pin 2 and Pin 4 set to LOW

// register addresses
#define RM3100_REG_POLL    0x00 // Polls for single measurement
#define RM3100_REG_CMM     0x01 // Continous mode
#define RM3100_REG_CCX     0x04 // Cycle count X MSB (3*2 bytes)
#define RM3100_REG_TMRC    0x0B // Continous mode data rate
#define RM3100_REG_MX      0x24 // Measurement results X MSB (3*3 bytes)
#define RM3100_REG_BIST    0x33 // Build-In Self Test
#define RM3100_REG_STATUS  0x34 // Status of DRDY
#define RM3100_REG_HSHAKE  0x35 // Handshake
#define RM3100_REG_REVID   0x36 // Revision


uint8_t RM3100::probe(MF_I2C *i2c) {
  MF_I2CDevice dev(i2c, 0);
  for(int i = 0; i < 4; i++) {
    dev.adr = RM3100_BASE_ADDRESS + i;
    uint8_t revid = 0;
    dev.readReg(RM3100_REG_REVID, &revid, 1); //readReg returns 1 when data was received
    //Serial.printf("RM3100.probe(0x%02X) = 0x%02X\n", dev.adr, revid);
    if(revid == 0x22) return dev.adr;
  }
  return 0;
}


RM3100::~RM3100() {
  delete dev;
}


RM3100::RM3100(MF_I2C *i2c, uint8_t i2c_adr, uint16_t cycle_count) {
  dev = new MF_I2CDevice(i2c, i2c_adr);

  //set cycle count
  uint8_t d[6];
  for(int i=0;i<6;i+=2) {
    d[i]= cycle_count >> 8;
    d[i+1] = cycle_count;
  }
  dev->writeReg(RM3100_REG_CCX, d, 6);

  scale_uT = 1 / ((0.3671 * (float)cycle_count) + 1.5); //scale factor in uT/LSB 
  //Note: should be 74LSB/uT at cc=200, but appears to be 10 times less with my sensor...

  // Set delay time between readings - this appears not to work as documented, value is read as 0x71 after power-up
  //dev->writeReg(RM3100_REG_TMRC, 0x92); //1.7 ms

  int tries = 10;
  while(tries) {
    // Continuous measurement
    dev->writeReg(RM3100_REG_CMM, 0x71);
    delay(10);
  
    // Needed to get cmm started.... 
    if(dataReady()) break;
    tries--;
  }
}


bool RM3100::dataReady() {
  return ( (dev->readReg(RM3100_REG_STATUS) & 0x80) == 0x80 );
}


void RM3100::readRaw(int32_t *xyz) {
  uint8_t d[9];
  dev->readReg(RM3100_REG_MX, d, 9);
  //Serial.printf("RM3100.readRaw() %02X%02X%02X %02X%02X%02X %02X%02X%02X\n", d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7], d[8]);

  int j = 0;
  for(int i=0;i<3;i++) {
    xyz[i] = (d[j]<<16) | (d[j+1]<<8) | d[j+2];
    if(xyz[i] & 0x00800000) xyz[i] |= 0xFF000000; //24bit sign expansion
    j+=3;
  }
  //Serial.printf("RM3100.readRaw() %d %d %d %f\n", (int)xyz[0], (int)xyz[1], (int)xyz[2], scale_uT);
}


void RM3100::read(float *x, float *y, float *z) {
  int32_t xyz[3];
  readRaw(xyz);
  *x = xyz[0] * scale_uT;
  *y = xyz[1] * scale_uT;
  *z = xyz[2] * scale_uT;
}
