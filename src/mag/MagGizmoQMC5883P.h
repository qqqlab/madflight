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

/*
Results polling sensor at full speed (by setting  mag.config.sample_rate = 10000 in madflight.h)

CONTINOUS, RNG_2G

OSR8 DSR1
=== Magnetometer (external) - Sample rate: 338 Hz===
mx[uT]        	mean:-20.084320	stdev:0.366410	min:-21.300001	max:-18.700001	n:1014
my[uT]        	mean:+6.349901	stdev:0.315687	min:+5.500000	max:+7.400000	n:1014
mz[uT]        	mean:-53.373520	stdev:0.569645	min:-55.266666	max:-51.246670	n:1014

OSR8 DSR2 --> output values x 4
=== Magnetometer (external) - Sample rate: 340 Hz===
mx[uT]        	mean:-77.972786	stdev:0.221217	min:-78.700005	max:-77.053337	n:1020
my[uT]        	mean:+24.959414	stdev:0.226411	min:+24.246668	max:+25.700001	n:1020
mz[uT]        	mean:-214.020432	stdev:0.385796	min:-215.486679	max:-212.820007	n:1020

OSR8 DSR4 --> output values x 2
=== Magnetometer (external) - Sample rate: 339 Hz===
mx[uT]        	mean:-38.907703	stdev:0.177179	min:-39.526669	max:-38.400002	n:1017
my[uT]        	mean:+12.298276	stdev:0.165770	min:+11.773334	max:+12.820001	n:1017
mz[uT]        	mean:-107.098076	stdev:0.238308	min:-107.906670	max:-106.246666	n:1017

OSR8 DSR8
=== Magnetometer (external) - Sample rate: 335 Hz===
mx[uT]        	mean:-19.063702	stdev:0.122097	min:-19.440001	max:-18.726667	n:1005
my[uT]        	mean:+6.025944	stdev:0.129753	min:+5.600000	max:+6.406667	n:1005
mz[uT]        	mean:-53.729515	stdev:0.191111	min:-54.260002	max:-53.260002	n:1005



OSR4 DSR1
=== Magnetometer (external) - Sample rate: 606 Hz===
mx[uT]        	mean:-20.081915	stdev:0.405473	min:-21.300001	max:-18.800001	n:1819
my[uT]        	mean:+6.345520	stdev:0.399842	min:+5.100000	max:+7.800000	n:1819
mz[uT]        	mean:-53.421970	stdev:0.708677	min:-55.700001	max:-51.046669	n:1819

OSR4 DSR2
=== Magnetometer (external) - Sample rate: 608 Hz===
mx[uT]        	mean:-75.867149	stdev:0.297883	min:-76.853333	max:-74.853333	n:1824
my[uT]        	mean:+24.006199	stdev:0.303903	min:+23.000000	max:+25.100000	n:1824
mz[uT]        	mean:-214.846542	stdev:0.496762	min:-216.453339	max:-212.700012	n:1824

OSR4 DSR4
=== Magnetometer (external) - Sample rate: 607 Hz===
mx[uT]        	mean:-37.546970	stdev:0.203176	min:-38.299999	max:-36.953335	n:1822
my[uT]        	mean:+11.735551	stdev:0.219780	min:+11.120000	max:+12.373334	n:1822
mz[uT]        	mean:-107.442589	stdev:0.348475	min:-108.666672	max:-106.260002	n:1822

OSR4 DSR4
mx[uT]        	mean:-18.759823	stdev:0.152427	min:-19.340000	max:-18.293333	n:1797
my[uT]        	mean:+5.851523	stdev:0.159706	min:+5.373333	max:+6.400000	n:1797
mz[uT]        	mean:-53.754726	stdev:0.253526	min:-54.666668	max:-53.006668	n:1797



OSR2 DSR1
=== Magnetometer (external) - Sample rate: 1000 Hz===
mx[uT]        	mean:-20.158100	stdev:0.543661	min:-22.400000	max:-18.400000	n:3000
my[uT]        	mean:+6.380333	stdev:0.537862	min:+4.700000	max:+8.400001	n:3000
mz[uT]        	mean:-53.290108	stdev:0.965145	min:-56.713333	max:-50.139999	n:3000

OSR1 DSR1
=== Magnetometer (external) - Sample rate: 1487 Hz===
mx[uT]        	mean:-19.809683	stdev:0.924344	min:-22.600000	max:-16.800001	n:4462
my[uT]        	mean:+6.563603	stdev:0.805155	min:+3.800000	max:+9.000000	n:4462
mz[uT]        	mean:-53.405621	stdev:1.468447	min:-58.960003	max:-46.886669	n:4462
*/

#pragma once

#include "mag.h"
#include "../hal/MF_I2C.h"

//#include "../cli/stat.h" //for debugging

#define QMC5883P_I2C_ADR 0x2C 

#define QMC5883P_I2C_MAX_CLK 400000 

#define QMC5883P_uT_per_LSB ((float)6.666666666e-3f) //2 Gauss range 15000 LSB/G

// Who Am I register
#define QMC5883P_WAI_REG 0x00

#define QMC5883P_WAI_VALUE 0x90

// Data register
#define QMC5883P_OUTX_REG 0x01

// Status register {DRDY:1;OVFL:1;RFU:6}
#define QMC5883P_STAT_REG 0x09

#define QMC5883P_STAT_DRDY 0x01 //1=data ready
#define QMC5883P_STAT_OVFL 0x02 //1=overflow

// Control register 1 {MODE:2;ODR:2;OSR:2;DSR:2;}
#define QMC5883P_CTRL1_REG 0x0A

#define QMC5883P_CTRL1_MD_SUSPEND    0b00
#define QMC5883P_CTRL1_MD_NORMAL     0b01
#define QMC5883P_CTRL1_MD_SINGLE     0b10
#define QMC5883P_CTRL1_MD_CONTINUOUS 0b11

//output data rate for normal mode
#define QMC5883P_CTRL1_ODR_200HZ (0b11<<2)
#define QMC5883P_CTRL1_ODR_100HZ (0b10<<2)
#define QMC5883P_CTRL1_ODR_50HZ  (0b01<<2)
#define QMC5883P_CTRL1_ODR_10HZ  (0b00<<2)

//datasheet: over sample ratio1

#define QMC5883P_CTRL1_OSR_8 (0b00<<4) //338 Hz
#define QMC5883P_CTRL1_OSR_4 (0b01<<4) //606 Hz
#define QMC5883P_CTRL1_OSR_2 (0b10<<4) //1000 Hz
#define QMC5883P_CTRL1_OSR_1 (0b11<<4) //1487 Hz

//datasheet: down sampling rate
#define QMC5883P_CTRL1_DSR_1  (0b00<<6)
#define QMC5883P_CTRL1_DSR_2  (0b01<<6)
#define QMC5883P_CTRL1_DSR_4  (0b10<<6)
#define QMC5883P_CTRL1_DSR_8  (0b11<<6)

// Control register 2 {SET:2;RGN:2;RFU:2;SELF_TEST:1;SOFT_RST;}
#define QMC5883P_CTRL2_REG 0x0B

#define QMC5883P_CTRL2_SET_RESET_ON  0b00
#define QMC5883P_CTRL2_SET_ONLY      0b01
#define QMC5883P_CTRL2_SET_RESET_OFF 0b11

#define QMC5883P_CTRL2_RNG_30G (0b00<<2)
#define QMC5883P_CTRL2_RNG_12G (0b01<<2)
#define QMC5883P_CTRL2_RNG_8G  (0b10<<2)
#define QMC5883P_CTRL2_RNG_2G  (0b11<<2)

#define QMC5883P_CTRL2_SELF_TEST_START (1<<6)
#define QMC5883P_CTRL2_SELF_TEST_STOP  0x00

#define QMC5883P_CTRL2_SOFT_RESET_START (1<<7)
#define QMC5883P_CTRL2_SOFT_RESET_STOP  0x00


class MagGizmoQMC5883P : public MagGizmo {
private:
  MF_I2CDevice *dev = nullptr;


public:
  const char* name() override {return "QMC5883P";}
  const float scale_uT = QMC5883P_uT_per_LSB; //scale factor to uT (at +/-200uT (+/-2G) RNG)
    int16_t mx; //raw adc values
    int16_t my;
    int16_t mz;

  ~MagGizmoQMC5883P() {
    delete dev;
  }

  MagGizmoQMC5883P(MF_I2C *i2c) {
    i2c->setClock(QMC5883P_I2C_MAX_CLK);

    this->dev = new MF_I2CDevice(i2c, QMC5883P_I2C_ADR); //i2c address is fixed

    //soft reset
    //dev->writeReg(QMC5883P_CTRL2_REG, QMC5883P_CTRL2_SOFT_RESET_START);
    //dev->writeReg(QMC5883P_CTRL2_REG, QMC5883P_CTRL2_SOFT_RESET_STOP);

    // datasheet: Define the sign for X Y and Z axis - reg 0x29 mentioned in examples, not documented in register section
    //dev->writeReg(0x29, 0x06);

    //configure
    uint8_t ctrl2_val = QMC5883P_CTRL2_SET_RESET_ON | QMC5883P_CTRL2_RNG_2G;
    uint8_t ctrl1_val = QMC5883P_CTRL1_MD_CONTINUOUS | QMC5883P_CTRL1_OSR_8 | QMC5883P_CTRL1_DSR_1;
    
    int tries = 20;
    uint8_t wai = 0;
    uint8_t stat = 0;
    uint8_t ctrl1 = 0;
    uint8_t ctrl2 = 0;
    while(tries) {
      ctrl1 = dev->readReg(QMC5883P_CTRL1_REG);
      ctrl2 = dev->readReg(QMC5883P_CTRL2_REG);
      wai = dev->readReg(QMC5883P_WAI_REG);

      //setup
      if(ctrl2 != ctrl2_val) dev->writeReg(QMC5883P_CTRL2_REG, ctrl2_val);
      if(ctrl1 != ctrl1_val) dev->writeReg(QMC5883P_CTRL1_REG, ctrl1_val);
      delay(1);

      stat = dev->readReg(QMC5883P_STAT_REG);
      if(stat & QMC5883P_STAT_DRDY) {
        break;
      }

      tries--;
    }
    if(!tries) {
      Serial.printf("MAG: ERROR could not configure QMC5883P. wai:0x%02X(expected 0x%02X), status:0x%02X(0x%02X), ctrl2:0x%02X(0x%02X), ctrl1:0x%02X(0x%02X)\n", wai, QMC5883P_WAI_VALUE, stat, QMC5883P_STAT_DRDY, ctrl2, ctrl2_val, ctrl1, ctrl1_val);
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
    uint8_t stat = dev->readReg(QMC5883P_STAT_REG);
    if((stat & QMC5883P_STAT_DRDY) == 0 || (stat & QMC5883P_STAT_OVFL) != 0 ) return false;

    uint8_t d[6];
    dev->readReg(QMC5883P_OUTX_REG, d, 6);

    mx = d[0] | (d[1] << 8); //16 bit litte-endian signed
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
