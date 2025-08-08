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

#pragma once

#include "mag.h"
#include "../hal/MF_I2C.h"

//#include "../cli/stat.h" //for debugging


// https://www.lcsc.com/datasheet/lcsc_datasheet_2410121623_QST-QMC6309_C5439871.pdf

// Who Am I register
#define QMC6309_WAI_REG 0x00

#define QMC6309_WAI_VALUE 0x90

// Data register
#define QMC6309_OUTX_REG 0x01

// Status register {DRDY:1;OVFL:1;...}
#define QMC6309_STAT_REG 0x09

#define QMC6309_STAT_DRDY 0x01 //1=data ready
#define QMC6309_STAT_OVFL 0x02 //1=overflow
#define QMC6309_STAT_ST_RDY 0x04 //1=self test ready
#define QMC6309_STAT_NVM_RDY 0x08 //1=nvm ready for access
#define QMC6309_STAT_NVM_LOAD_DONE 0x10 //1=nvm loading done

// Control register 1 {MD:2;:1;OSR:2;LPF:3;}
#define QMC6309_CTRL1_REG 0x0A

#define QMC6309_CTRL1_MD_SUSPEND    0b00
#define QMC6309_CTRL1_MD_NORMAL     0b01
#define QMC6309_CTRL1_MD_SINGLE     0b10
#define QMC6309_CTRL1_MD_CONTINUOUS 0b11

#define QMC6309_CTRL1_OSR_1 (0b11<<3)
#define QMC6309_CTRL1_OSR_2 (0b10<<3)
#define QMC6309_CTRL1_OSR_4 (0b01<<3)
#define QMC6309_CTRL1_OSR_8 (0b00<<3)

#define QMC6309_CTRL1_LPF_1  (0b000<<5)
#define QMC6309_CTRL1_LPF_2  (0b001<<5)
#define QMC6309_CTRL1_LPF_4  (0b010<<5)
#define QMC6309_CTRL1_LPF_8  (0b011<<5)
#define QMC6309_CTRL1_LPF_16 (0b100<<5)

// Control register 2 {SET:2;RGN:2;ODR:3;SRT:1;}
#define QMC6309_CTRL2_REG 0x0B

#define QMC6309_CTRL2_SET_RESET_ON  0b00
#define QMC6309_CTRL2_SET_ONLY      0b01
#define QMC6309_CTRL2_SET_RESET_OFF 0b11

#define QMC6309_CTRL2_RNG_32G (0b00<<2)
#define QMC6309_CTRL2_RNG_16G (0b01<<2)
#define QMC6309_CTRL2_RNG_8G  (0b10<<2)

//output data rate for normal mode
#define QMC6309_CTRL2_ODR_1Hz   (0b000<<4)
#define QMC6309_CTRL2_ODR_10Hz  (0b001<<4)
#define QMC6309_CTRL2_ODR_50Hz  (0b010<<4)
#define QMC6309_CTRL2_ODR_100Hz (0b011<<4)
#define QMC6309_CTRL2_ODR_200Hz (0b100<<4)

#define QMC6309_CTRL2_SOFT_RESET_START 0x80
#define QMC6309_CTRL2_SOFT_RESET_STOP  0x00


class MagGizmoQMC6309 : public MagGizmo {
private:
  MF_I2CDevice *dev = nullptr;


public:
  const float scale_uT = 0.025; //scale factor to uT   (at +/-800uT (+/-8G) RNG)
    int16_t mx; //raw adc values
    int16_t my;
    int16_t mz;

  ~MagGizmoQMC6309() {
    delete dev;
  }

  MagGizmoQMC6309(MF_I2C *i2c) {
    i2c->setClock(400000);

    this->dev = new MF_I2CDevice(i2c, 0x7C); //i2c address is always 0x7C

    //soft reset
    //dev->writeReg(QMC6309_CTRL2_REG, QMC6309_CTRL2_SOFT_RESET_START);
    //dev->writeReg(QMC6309_CTRL2_REG, QMC6309_CTRL2_SOFT_RESET_STOP);

    //configure 220Hz update rate with LPF_16
    uint8_t ctrl2_val = QMC6309_CTRL2_SET_RESET_ON | QMC6309_CTRL2_RNG_8G | QMC6309_CTRL2_ODR_100Hz;
    uint8_t ctrl1_val = QMC6309_CTRL1_MD_CONTINUOUS | QMC6309_CTRL1_OSR_8 | QMC6309_CTRL1_LPF_16; //mx	mean:-10.982379	stdev:1.401531	min:-14.000000	max:-8.000000	n:227
    //uint8_t ctrl1_val = QMC6309_CTRL1_MD_CONTINUOUS | QMC6309_CTRL1_OSR_8 | QMC6309_CTRL1_LPF_1; //mx	mean:-10.060345	stdev:6.673316	min:-33.000000	max:+4.000000	n:232
    //uint8_t ctrl1_val = QMC6309_CTRL1_MD_CONTINUOUS | QMC6309_CTRL1_OSR_1 | QMC6309_CTRL1_LPF_1; //mx	mean:-11.925335	stdev:38.645042	min:-119.000000	max:+106.000000	n:1125

    int tries = 20;
    uint8_t wai = 0;
    uint8_t stat = 0;
    uint8_t ctrl1 = 0;
    uint8_t ctrl2 = 0;
    while(tries) {
      ctrl1 = dev->readReg(QMC6309_CTRL1_REG);
      ctrl2 = dev->readReg(QMC6309_CTRL2_REG);
      wai = dev->readReg(QMC6309_WAI_REG);

      //setup
      if(ctrl2 != ctrl2_val) dev->writeReg(QMC6309_CTRL2_REG, ctrl2_val);
      if(ctrl1 != ctrl1_val) dev->writeReg(QMC6309_CTRL1_REG, ctrl1_val);
      delay(1);

      stat = dev->readReg(QMC6309_STAT_REG);
      if(stat & QMC6309_STAT_DRDY) {
        break;
      }

      tries--;
    }
    if(!tries) {
      Serial.printf("MAG: ERROR could not configure QMC6309. wai:0x%02X(expected 0x%02X), status:0x%02X(0x%02X), ctrl2:0x%02X(0x%02X), ctrl1:0x%02X(0x%02X)\n", wai, QMC6309_WAI_VALUE, stat, QMC6309_STAT_DRDY, ctrl2, ctrl2_val, ctrl1, ctrl1_val);
    }

    /* //DEBUG 
    while(1) {
      Stat mx,my,mz;
      uint32_t start_ts = micros();
      while(micros() - start_ts < 1000000) {
        float x,y,z;
        if(update(&x,&y,&z)) {
          mx.append(x);
          my.append(y);
          mz.append(z);
        }
      }
      
      Serial.println("=== Magnetometer (external) ===");
      mx.print("mx");
      my.print("my");
      mz.print("mz");
    }
    */
  }

  bool update_raw() {
    uint8_t stat = dev->readReg(QMC6309_STAT_REG);
    if((stat & QMC6309_STAT_DRDY) == 0 || (stat & QMC6309_STAT_OVFL) != 0 ) return false;

    uint8_t d[6];
    dev->readReg(QMC6309_OUTX_REG, d, 6);

    mx = d[0] | (d[1] << 8); //16 bit litte-endian signed - 40LSB/uT at +/-800uT scale
    my = d[2] | (d[3] << 8);
    mz = d[4] | (d[5] << 8);

    return true;
  }

  bool update(float *x, float *y, float *z) override {
    if(!update_raw()) return false;

    *x =  scale_uT * mx; 
    *y =  scale_uT * my;
    *z =  scale_uT * mz;
    return true;
  }
};
