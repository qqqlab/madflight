/*==========================================================================================
MIT License

Copyright (c) 2026 https://madflight.com

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

//Driver for ICM-45686 gyroscope/accelometer

#include "ICM45686.h"
#include <Arduino.h> //delayMicroseconds

#define ICM45686_REG_WHOAMI           0x72
#define ICM45686_REG_PWR_MGMT0        0x10
#define ICM45686_REG_INT1_CONFIG0     0x16
#define ICM45686_REG_INT1_CONFIG1     0x17
#define ICM45686_REG_INT1_CONFIG2     0x18
#define ICM45686_REG_INT1_STATUS0     0x19
#define ICM45686_REG_ACCEL_CONFIG0    0x1B
#define ICM45686_REG_GYRO_CONFIG0     0x1C
#define ICM45686_REG_FIFO_CONFIG0     0x1D
#define ICM45686_REG_FIFO_CONFIG1_0   0x1E
#define ICM45686_REG_FIFO_CONFIG1_1   0x1F
#define ICM45686_REG_FIFO_CONFIG2     0x20
#define ICM45686_REG_FIFO_CONFIG3     0x21
#define ICM45686_REG_FIFO_CONFIG4     0x22
#define ICM45686_REG_RTC_CONFIG       0x26
#define ICM45686_REG_FIFO_COUNTH      0x12
#define ICM45686_REG_FIFO_COUNTL      0x13
#define ICM45686_REG_FIFO_DATA        0x14
#define ICM45686_REG_INTF_CONFIG0     0x2C
#define ICM45686_REG_IOC_PAD_SCENARIO 0x2F
#define ICM45686_REG_IOC_PAD_SCENARIO_AUX_OVRD 0x30
#define ICM45686_REG_IOC_PAD_SCENARIO_OVRD 0x31
#define ICM45686_REG_PWR_MGMT_AUX1    0x54
#define ICM45686_REG_IREG_ADDRH       0x7C
#define ICM45686_REG_IREG_ADDRL       0x7D
#define ICM45686_REG_IREG_DATA        0x7E
#define ICM45686_REG_REG_MISC2        0x7F
#define ICM45686_REG_SREG_CTRL        0x63

#define ICM45686_BANK_IMEM_SRAM_ADDR  0x0000
#define ICM45686_BANK_IPREG_BAR_ADDR  0xA000
#define ICM45686_BANK_IPREG_TOP1_ADDR 0xA200
#define ICM45686_BANK_IPREG_SYS1_ADDR 0xA400
#define ICM45686_BANK_IPREG_SYS2_ADDR 0xA500

#define ICM45686_REG_GYRO_UI_LPF_CFG  0xA4AC

#define ICM45686_WHOAMI_VALUE         0xE9

#define ICM45686_SPI_LOW_SPEED       1000000
#define ICM45686_SPI_HIGH_SPEED     24000000
#define ICM45686_I2C_LOW_SPEED        100000
#define ICM45686_I2C_HIGH_SPEED      1000000

int ICM45686::begin(SensorDevice* dev, int sample_rate, bool use_clkin) {
    sampleRateActual = 0;
    error_code = 0;
    fifobuf.tx_reg = ICM45686_REG_FIFO_COUNTH | 0x80;

    this->dev = dev;
    dev->setLowSpeed();

    // check who-am-i
    uint8_t wai = dev->readReg(0x72);
    if(wai != 0xE9) {
        error_code = -(1000000 + wai);
        return error_code;
    }

    // do soft reset
    dev->writeReg(ICM45686_REG_REG_MISC2, 0x02);
    delay(10);
    // check if reset done
    if (!(dev->readReg(ICM45686_REG_INT1_STATUS0) & 0x80)) {
        error_code = 2000000;
    }

    // setup odr rate
    uint8_t odr_config;
    if (sample_rate >= 6400) {
        odr_config = 0x03;
        sampleRateActual = 6400;
    } else if (sample_rate >= 3200) {
        odr_config = 0x04;
        sampleRateActual = 3200;
    } else if (sample_rate >= 1600) {
        odr_config = 0x05;
        sampleRateActual = 1600;
    } else if (sample_rate >= 800) {
        odr_config = 0x06;
        sampleRateActual = 800;
    } else if (sample_rate >= 400) {
        odr_config = 0x07;
        sampleRateActual = 400;
    } else if (sample_rate >= 200) {
        odr_config = 0x08;
        sampleRateActual = 200;
    } else {
        odr_config = 0x09;
        sampleRateActual = 100;
    }
    write_reg(0x001C, 0xFF, odr_config); //GYRO_CONFIG 4000dps, GYRO_ODR b0-3
    write_reg(0x001B, 0xFF, odr_config); //ACCEL_CONFIG0  32G, ACCEL_ODR b0-3

    // setup FIFO - see AN-000478 ICM-45686 User Guide - 3.5 Setting up FIFO
    // (*) are additions
    write_reg(0x0021, 0b00001111, 0b00000000); //FIFO_CONFIG3 disable all fifo
    write_reg(0x0016, 0b00000010, 0b00000010); //INT1_CONFIG0 enable fifo threshold interrupt
    write_reg(0x0017, 0b11111111, 0b00000000); // (*) INT1_CONFIG1 disable apex interrupts
    write_reg(0x0018, 0b00000111, 0b00000001); // (*) INT1_CONFIG2 interrupt pushpull, pulse, active high    
    write_reg(0x001D, 0b11000000, 0b00000000); //FIFO_CONFIG0 fifo disabled
    write_reg(0xA258, 0b00000010, 0b00000010); //IPREG_TOP1:SMC_CONTROL_0 timestamp enable for fifo
    write_reg(0x001E, 0xff, 1);                //FIFO_CONFIG1_0 fifo watermark lsb
    write_reg(0x001F, 0xff, 0);                //FIFO_CONFIG1_0 fifo watermark msb
    write_reg(0x0020, 0b00001000, 0b00001000); //FIFO_CONFIG2 watermark interrupt on greater than or equal
    write_reg(0x0021, 0b00001110, 0b00001110); //FIFO_CONFIG3 enable HIRES, GYRO, ACC
    write_reg(0x001D, 0b11000000, 0b11000000); //FIFO_CONFIG0 fifo enabled
    write_reg(0x0021, 0b00000001, 0b00000001); //FIFO_CONFIG3 enable interface

    if(use_clkin) {
      // setup CLKIN - see AN-000478 ICM-45686 User Guide - 7 HOW TO USE EXTERNAL INPUT CLOCK
      // (*) are additions
      write_reg(0x0031, 0b00000111, 0b00000110); //IOC_PAD_SCENARIO_OVRD PADS_INT2_CFG_OVRD=1 & PADS_INT2_CFG_OVRD_VAL=2. Note this reg is not documented in the datasheet, only in the user guide
      write_reg(0x0030, 0b00000010, 0b00000010); // (*) IOC_PAD_SCENARIO_AUX_OVRD disable AUX1 
      write_reg(0xA258, 0b00010000, 0b00010000); // (*) SMC_CONTROL_0: ACCEL_LP_CLK_SEL=1
      write_reg(0x0026, 0b00100000, 0b00100000); //RTC_CONFIG RTC_MODE=1. RTC functionality can be enabled only if ACCEL_LP_CLK_SEL is set to 1; otherwise device may not behave as expected.
      write_reg(0xA268, 0b00000100, 0b00000100); //PREG_TOP1:SIFS_I3C_STC_CFG I3C_STC_MODE=0 disable the I3CSM STC
    }

    // enable Interpolator and Anti Aliasing Filter on Gyro and accel
    write_reg(0xA4A6, 0b00110000, 0b00100000); //IPREG_SYS1_REG_166 reg:0xA6 field:GYRO_SRC_CTRL b5-6 -> 2
    write_reg(0xA57B, 0b00000011, 0b00000010); //IPREG_SYS2_REG123 reg:0x7B field:ACCEL_SRC_CTRL b0-1 -> 2

    // low pass filter (default = bypass)
    //write_reg(0xA57B, 0b00000111, 6); //IPREG_SYS1_REG_172 0=bypass, 1=odr/4, 2=/8, 3=/16, 4=/32, 5=/64, 6=/128

    // enable gyro+acc in low noise mode
    write_reg(0x0010, 0b00001111, 0b00001111); //PWR_MGMT0

    if(dev->isSPI()) {
        dev->setFreq(ICM45686_SPI_HIGH_SPEED);
    }else{
        dev->setFreq(ICM45686_I2C_HIGH_SPEED);
    }

    return error_code;
}

static inline int32_t uint20_to_int(uint8_t msb, uint8_t bits, uint8_t lsb) {
    uint32_t value20bit = uint32_t(msb) << 12U | uint32_t(bits) << 4U | lsb;
    // Check the sign bit (MSB)
    if (value20bit & 0x80000) { // MSB is set (negative value)
        // Extend the sign bit to the upper 12 bits of the 32-bit integer
        return (int32_t)(value20bit | 0xFFF00000);
    } else { // MSB is not set (positive value)
        return (int32_t)value20bit;
    }
}

//returns number of samples, or negative on error
int ICM45686::read() {
  if(error_code < 0) return error_code;

  int sample_count = 0;
  do{
    // get sample count and sample in a single SPI transaction
    //dev->write_read_blocking((uint8_t*)&fifobuf.tx_reg, (uint8_t*)&fifobuf.rx_start, sizeof(fifobuf)-1); // FAST: hardware optimized - uses the same buffer for reading and writing with one byte offset to preserve fifobuf.tx_reg between calls
    dev->readRegs(ICM45686_REG_FIFO_COUNTH, (uint8_t*)&fifobuf.n_samples, sizeof(fifobuf)-2); // SLOW: arduino standard
    //for(uint i=0;i<sizeof(fifobuf);i++) Serial.printf("%02X ",((uint8_t*)&fifobuf)[i]); Serial.println();

    // store sample count on first iteration
    if(sample_count == 0) sample_count = fifobuf.n_samples; 

    // check header
    if ((fifobuf.header & 0xFC) != 0x78) { // ACCEL_EN | GYRO_EN | HIRES_EN | TMST_FIELD_EN
        // no or bad data -> flush fifo and bail out
        uint8_t oldval = dev->readReg(ICM45686_REG_FIFO_CONFIG2); //reg 0x20
        dev->writeReg(ICM45686_REG_FIFO_CONFIG2, oldval | 0x80); //reg 0x20
        dev->writeReg(ICM45686_REG_FIFO_CONFIG2, oldval); //reg 0x20
        return -1; //bad data error
    }
  } while(fifobuf.n_samples > 1);

  //convert last sample
  if(fifobuf.n_samples != 0) {
    acc[0] = acc_scale * uint20_to_int(fifobuf.acc[1], fifobuf.acc[0], fifobuf.ax);
    acc[1] = acc_scale * uint20_to_int(fifobuf.acc[3], fifobuf.acc[2], fifobuf.ay);
    acc[2] = acc_scale * uint20_to_int(fifobuf.acc[5], fifobuf.acc[4], fifobuf.az);
    gyr[0] = gyr_scale * uint20_to_int(fifobuf.gyr[1], fifobuf.gyr[0], fifobuf.gx);
    gyr[1] = gyr_scale * uint20_to_int(fifobuf.gyr[3], fifobuf.gyr[2], fifobuf.gy);
    gyr[2] = gyr_scale * uint20_to_int(fifobuf.gyr[5], fifobuf.gyr[4], fifobuf.gz);
  }

  return sample_count;
}

bool ICM45686::write_reg(uint16_t reg, uint8_t mask, uint8_t value) {
  uint8_t old_val, write_val, new_val;
  if(reg<=0xff) {
    old_val = dev->readReg(reg);
    write_val = old_val & ~mask;
    write_val |= value;
    dev->writeReg(reg, write_val);
    new_val = dev->readReg(reg);
  }else{
    old_val = read_bank(reg);
    write_val = old_val & ~mask;
    write_val |= value;
    write_bank(reg, write_val);
    new_val = read_bank(reg);
  }
  bool ok = (write_val == new_val);
  //Serial.printf("write_reg reg:%04X mask:%02X val:%02X -> old:%02X new:%02X ok:%d\n", reg, mask, value, old_val, new_val, ok );
  if(!ok && error_code == 0) error_code = 3000000 + reg;
  return ok;
}

uint8_t ICM45686::read_bank(uint16_t addr)
{
    // set indirect register address
    uint8_t send[] = {(uint8_t)(addr >> 8), (uint8_t)(addr & 0xFF)};
    dev->writeRegs(0x7C, send, sizeof(send)); //ICM45686_REG_IREG_ADDRH

    // try reading IREG_DATA on ready
    for (uint8_t i=0; i<10; i++) {
        if (dev->readReg(0x7F) & 0x01) { //ICM45686_REG_REG_MISC2
            break;
        }
        // minimum wait time-gap between IREG access is 4us
        delayMicroseconds(10);
    }

    // read the data
    return dev->readReg(0x7E); //ICM45686_REG_IREG_DATA
}

void ICM45686::write_bank(uint16_t addr, uint8_t val)
{
    // set indirect register address
    uint8_t send[] = {(uint8_t)(addr >> 8), (uint8_t)(addr & 0xFF), val};
    dev->writeRegs(0x7C, send, sizeof(send)); //ICM45686_REG_IREG_ADDRH

    //check if IREG_DATA ready, we can return immediately if so
    if (dev->readReg(0x7F) & 0x01) { //ICM45686_REG_REG_MISC2
        return;
    }
    // minimum wait time-gap between IREG access is 4us
    delayMicroseconds(10);
}
