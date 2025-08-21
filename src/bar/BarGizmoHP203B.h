/*=======================================================
Driver for HP203B pressure sensor
=======================================================*/

#pragma once

#include "bar.h"
#include "../hal/MF_I2C.h"

#define HP203B_SOFT_RST 0x06 //SOFT_RST soft reset
#define HP203B_ANA_CAL 0x28 //ANA_CAL Re-calibrate the Internal analog Blocks
//ADC_CVT: start conversion: bit7-5 = 010 ADC_CVT, bit4-2 = OSR, bit1-0 = 00 = press+temp
#define HP203B_ADC_CVT_128  0b01010100 //101 OSR =  128   4.1ms
#define HP203B_ADC_CVT_256  0b01010000 //100 OSR =  256   8.2ms
#define HP203B_ADC_CVT_512  0b01001100 //011 OSR =  512  16.4ms
#define HP203B_ADC_CVT_1024 0b01001000 //010 OSR = 1024  32.8ms
#define HP203B_ADC_CVT_2048 0b01000100 //010 OSR = 2048  65.6ms
#define HP203B_ADC_CVT_4096 0b01000000 //010 OSR = 4096 131.1ms
#define HP203B_READ_PT 0x10 //READ_PT pressure [Pa] + temp [0.01*C]
#define HP203B_READ_AT 0x11 //READ_AT altitude [0.01*m] + temp [0.01*C]

#define HP203B_INT_SRC 0x8D //read INT_SRC register
#define HP203B_INT_SRC_DEV_RDY (1<<6) //only access device when this flag is 1 (0x40)
#define HP203B_INT_SRC_PA_RDY (1<<5) //press/alt conversion ready (0x20)

#define HP203B_INT_EN 0xCB //write INT_EN register

class BarGizmoHP203B: public BarGizmo {
private:
  MF_I2CDevice *dev;
  uint32_t sample_ts = 0;

  bool is_pa_ready() {
    uint8_t status = dev->readReg(HP203B_INT_SRC);
    return ( (status & HP203B_INT_SRC_PA_RDY) == HP203B_INT_SRC_PA_RDY );
  }

  bool is_dev_ready() {
    uint8_t status = dev->readReg(HP203B_INT_SRC);
    return ( (status & HP203B_INT_SRC_DEV_RDY) == HP203B_INT_SRC_DEV_RDY);
  }

  //wait for dev_ready, timeout default 2 ms
  bool wait_dev_ready(uint32_t timeout_us = 2000) {
    uint32_t start = micros();
    do{
      if(is_dev_ready()) return true;
    } while(micros() - start <= timeout_us);
    return false; //timeout
  }

public:
  BarGizmoHP203B(MF_I2C *i2c, int8_t i2c_adr, uint32_t sampleRate) {
    (void) sampleRate; //TODO - currently fixed at 120 Hz (8.2ms)
    if(i2c_adr == 0) i2c_adr = 0x76; // fixed: 0x76 or 0x77
    this->dev = new MF_I2CDevice(i2c, i2c_adr);

    int tries = 10;
    while(tries) {
      tries--;
      dev->writeReg(HP203B_SOFT_RST, nullptr, 0); //SOFT_RST soft reset (just send, don't first wait for dev_ready)

      if(!wait_dev_ready()) continue;
      dev->writeReg(HP203B_ANA_CAL, nullptr, 0); //ANA_CAL Re-calibrate the Internal analog Blocks

      //if(!wait_dev_ready()) continue;
      //dev->writeReg(0xCF, 0x00); //disable compensation (don't do this)

      //Datasheet: If the INT_CFG bit is set to 0 while the INT_EN bit is set to 1, the corresponding interrupt flag will appear in the INT_SRC register but the interrupt will not be output to the INT1 pin.
      if(!wait_dev_ready()) continue;
      dev->writeReg(HP203B_INT_EN, 0x30); //enable PA_RDY and T_RDY flags and interrupts

      if(!wait_dev_ready()) continue;
      dev->writeReg(HP203B_ADC_CVT_256, nullptr, 0); //start conversion: ADC_CVT = 010, 100 OSR=256 (8.2ms), 00 press+temp
      break;
    }

    sample_ts = micros();
  }

  ~BarGizmoHP203B() {
    delete dev;
  }

  bool update(float *press, float *temp) override {
    uint32_t now = micros();
    if(now - sample_ts < 10000) return false; //NOTE: gets garbage from sensor if polled quicker.... (PA_RDY is not reliable)

    //check PA_RDY status
    if(!is_pa_ready()) {
      return false; //bail out, try again later
    }

    sample_ts = now;

    //read data
    uint8_t d[6];
    dev->readReg(HP203B_READ_PT, d, 6); //READ_PT pressure [Pa] + temp [0.01*C]

    int32_t t = ((d[0] << 16) | (d[1] << 8) | d[2]) & 0x000fffff; //20 bit big-endian signed
    if(t & 0x00080000) t |= 0xfff00000; //20bit sign expansion
    int32_t p = ((d[3] << 16) | (d[4] << 8) | d[5]) & 0x000fffff; //20 bit big-endian unsigned

    *temp = t * 0.01; //temperature in [C]
    *press = p; //pressure in [Pa]

    //start new conversion
    dev->writeReg(HP203B_ADC_CVT_256, nullptr, 0); //start conversion: ADC_CVT = 010, 100 OSR=256 (8.2ms), 00 press+temp

    return true;
  }
};
