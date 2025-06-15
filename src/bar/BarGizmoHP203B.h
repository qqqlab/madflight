#pragma once

#include "bar.h"
#include "../hal/MF_I2C.h"

/*
OSR  T[ms]  ADC_CVT
 128   4.1 0b01010100
 256   8.2 0b01010000
 512  16.4 0b01001100
1024  32.8 0b01001000
2048  65.6 0b01000100
4096 131.1 0b01000000 
*/

#define HP203B_SOFT_RST 0x06 //SOFT_RST soft reset
#define HP203B_ANA_CAL 0x28 //ANA_CAL Re-calibrate the Internal analog Blocks
#define HP203B_ADC_CVT_256 0b01010000 //start conversion: ADC_CVT = 010, 100 OSR=256 (8.2ms), 00 press+temp
#define HP203B_READ_PT 0x10 //READ_PT pressure [Pa] + temp [0.01*C]
#define HP203B_READ_AT 0x11 //READ_AT altitude [0.01*m] + temp [0.01*C]

#define HP203B_INT_SRC 0x8D
#define HP203B_INT_SRC_DEV_RDY (1<<6) //only access device when 1
#define HP203B_INT_SRC_PA_RDY (1<<5) //press/alt conversion ready


class BarGizmoHP203B: public BarGizmo {
private:
  MF_I2CDevice *dev;
public:
  BarGizmoHP203B(MF_I2C *i2c, int8_t i2c_adr, uint32_t sampleRate) {
    (void) sampleRate; //TODO
    if(i2c_adr == 0) i2c_adr = 0x76; // fixed: 0x76 or 0x77
    this->dev = new MF_I2CDevice(i2c, i2c_adr);

    dev->writeReg(HP203B_SOFT_RST, nullptr, 0); //SOFT_RST soft reset
    delay(1); //FIXME: should check HP203B_INT_SRC_DEV_RDY
    dev->writeReg(HP203B_ANA_CAL, nullptr, 0); //ANA_CAL Re-calibrate the Internal analog Blocks
    delay(1); //FIXME: should check HP203B_INT_SRC_DEV_RDY
    dev->writeReg(HP203B_ADC_CVT_256, nullptr, 0); //start conversion: ADC_CVT = 010, 100 OSR=256 (8.2ms), 00 press+temp
  }

  ~BarGizmoHP203B() {
    delete dev;
  }

  bool update(float *press, float *temp) override {
    //only read device when dev_rdy and pa_rdy
    //FIXME: put timeout on waiting for pa_rdy
    const uint8_t mask = HP203B_INT_SRC_DEV_RDY | HP203B_INT_SRC_DEV_RDY;
    if( (dev->readReg(HP203B_INT_SRC) & mask) != mask) return false;
   
    uint8_t d[6];
    dev->readReg(HP203B_READ_PT, d, 6); //READ_PT pressure [Pa] + temp [0.01*C]
    dev->writeReg(HP203B_ADC_CVT_256, nullptr, 0); //start conversion: ADC_CVT = 010, 100 OSR=256 (8.2ms), 00 press+temp

    int32_t t = ((d[0] << 16) | (d[1] << 8) | d[2]) & 0x000fffff; //20 bit big-endian signed
    if(t & 0x00080000) t |= 0xfff00000; //20bit sign expansion
    int32_t p = ((d[3] << 16) | (d[4] << 8) | d[5]) & 0x000fffff; //20 bit big-endian unsigned

    *temp = t * 0.01; //temperature in [C]
    *press = p; //pressure in [Pa]

    return true;
  }
};
