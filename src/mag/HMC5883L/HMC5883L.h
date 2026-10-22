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
==========================================================================================*/

// Driver for the Honeywell HMC5883L 3-axis magnetometer
//
// Datasheet: https://www.sparkfun.com/datasheets/SensorsCompass/HMC5883L.pdf
//
// This sensor is fitted on the GY-87 10DOF board (MPU6050 + HMC5883L + BMP280).
//
// Note: the HMC5883L is NOT the same as the QMC5883L. They use different I2C
// addresses (0x1E vs 0x0D) and completely different register maps.

#pragma once

#include <Arduino.h> //delay()
#include "../../hal/MF_I2C.h"

//------------------------------------------------------------------
// Register map
//------------------------------------------------------------------
#define HMC5883L_REG_X_MSB     0x00
#define HMC5883L_REG_X_LSB     0x01
#define HMC5883L_REG_Y_MSB     0x02
#define HMC5883L_REG_Y_LSB     0x03
#define HMC5883L_REG_Z_MSB     0x04
#define HMC5883L_REG_Z_LSB     0x05
#define HMC5883L_REG_CTRL_A    0x09
#define HMC5883L_REG_CTRL_B    0x0A
#define HMC5883L_REG_CTRL_C    0x0B
#define HMC5883L_REG_STATUS    0x0D
#define HMC5883L_REG_CHIP_ID_A 0x0E
#define HMC5883L_REG_CHIP_ID_B 0x0F

//bit 7 of the register address is the auto-increment bit: when set, the register
//address is incremented after every byte, so a burst read returns consecutive registers
#define HMC5883L_AUTO_INC      0x80

//------------------------------------------------------------------
// Control Register A (0x09)
//------------------------------------------------------------------
#define HMC5883L_CTRL_A_SOFT_RST 0x80 //self clearing soft reset

//output data rate [6:5]
#define HMC5883L_ODR_0P75HZ     0x00
#define HMC5883L_ODR_1P5HZ      0x20
#define HMC5883L_ODR_15HZ       0x40
#define HMC5883L_ODR_75HZ       0x60 //maximum rate of the HMC5883L

//gain [4:2]
#define HMC5883L_GAIN_0P5GA     0x00 //1370 LSB/Gauss
#define HMC5883L_GAIN_1GA       0x04 //1090 LSB/Gauss
#define HMC5883L_GAIN_2GA       0x08 // 549 LSB/Gauss
#define HMC5883L_GAIN_4GA       0x0C // 438 LSB/Gauss
#define HMC5883L_GAIN_8GA       0x10 // 390 LSB/Gauss

//measurement mode [1:0]
#define HMC5883L_MODE_NORMAL    0x00
#define HMC5883L_MODE_POS_BIAS  0x01
#define HMC5883L_MODE_NEG_BIAS  0x02

//------------------------------------------------------------------
// Status Register (0x0D)
//------------------------------------------------------------------
#define HMC5883L_STATUS_DRDY    0x01 //data ready
#define HMC5883L_STATUS_DOR      0x02 //data overrun
#define HMC5883L_STATUS_LOCK     0x04 //measurement cycle complete

//------------------------------------------------------------------
// I2C addresses
//------------------------------------------------------------------
#define HMC5883L_ADDRESS        0x1E //ADDR pin low (default on the GY-87)
#define HMC5883L_ADDRESS_ALT    0x1D //ADDR pin high

//------------------------------------------------------------------
// Scale factor: 1 Gauss = 100 uT
//------------------------------------------------------------------
#define HMC5883L_uT_per_LSB_0P5GA (100.0f/1370.0f)
#define HMC5883L_uT_per_LSB_1GA   (100.0f/1090.0f)
#define HMC5883L_uT_per_LSB_2GA   (100.0f/ 549.0f)
#define HMC5883L_uT_per_LSB_4GA   (100.0f/ 438.0f)
#define HMC5883L_uT_per_LSB_8GA   (100.0f/ 390.0f)

//------------------------------------------------------------------
// Axis mapping
//------------------------------------------------------------------
// The physical axes of the HMC5883L are documented as X=East, Y=North, Z=Down
// (component side up). madflight expects NWU (North, West, Up) values.
//
//   HMC5883L_AXIS_NWU : n=+y, w=-x, u=-z   (datasheet physical orientation)
//   HMC5883L_AXIS_RAW : n=+x, w=+y, u=+z   (raw chip axes, same convention as
//                                              the other madflight magnetometers)
//
// Use "mag_align" to compensate for the rotation of the sensor on the airframe.
#ifndef HMC5883L_AXIS_MAP
  #define HMC5883L_AXIS_MAP HMC5883L_AXIS_NWU
#endif
#define HMC5883L_AXIS_NWU 1
#define HMC5883L_AXIS_RAW 2

class HMC5883L {
  public:
    HMC5883L() {}

    void setAdr(uint8_t adr = 0) {
      if(adr) {
        _adr = adr;
      }else{
        _adr = HMC5883L_ADDRESS;
      }
    }

    uint8_t getAdr() { return _adr; }

    //chip id register A, 0x48 ('H') for a HMC5883L
    uint8_t chipIdA() { return _readReg(HMC5883L_REG_CHIP_ID_A); }
    uint8_t chipIdB() { return _readReg(HMC5883L_REG_CHIP_ID_B); }

    bool detect() {
      return (chipIdA() == 0x48);
    }

    void begin(MF_I2C* i2c, uint8_t adr = 0) {
      _i2c = i2c;
      setAdr(adr);
      _uT_per_LSB = HMC5883L_uT_per_LSB_2GA;
      _gain = HMC5883L_GAIN_2GA;

      //soft reset, then wait for the chip to start up
      _writeReg(HMC5883L_REG_CTRL_A, HMC5883L_CTRL_A_SOFT_RST);
      delay(10);

      //normal mode, 75Hz output data rate, +/-2 Gauss
      setMode(HMC5883L_ODR_75HZ, _gain, HMC5883L_MODE_NORMAL);

      //discard the first samples, they are not valid after power up
      for(int i = 0; i < 10; i++) {
        readRaw(&_x, &_y, &_z);
        delay(2);
      }
    }

    //output data rate, gain and measurement mode
    void setMode(uint8_t odr, uint8_t gain, uint8_t mode) {
      _gain = gain;
      switch(gain) {
        case HMC5883L_GAIN_0P5GA: _uT_per_LSB = HMC5883L_uT_per_LSB_0P5GA; break;
        case HMC5883L_GAIN_1GA:   _uT_per_LSB = HMC5883L_uT_per_LSB_1GA;   break;
        case HMC5883L_GAIN_2GA:   _uT_per_LSB = HMC5883L_uT_per_LSB_2GA;   break;
        case HMC5883L_GAIN_4GA:   _uT_per_LSB = HMC5883L_uT_per_LSB_4GA;   break;
        case HMC5883L_GAIN_8GA:   _uT_per_LSB = HMC5883L_uT_per_LSB_8GA;   break;
        default: _uT_per_LSB = HMC5883L_uT_per_LSB_2GA; break;
      }
      _writeReg(HMC5883L_REG_CTRL_A, odr | gain | mode);
    }

    void reset() {
      _writeReg(HMC5883L_REG_CTRL_A, HMC5883L_CTRL_A_SOFT_RST);
    }

    //status register, bit0=data ready, bit1=data overrun, bit2=cycle complete
    uint8_t status() { return _readReg(HMC5883L_REG_STATUS); }

    //true when the chip has a new measurement available
    bool dataReady() { return (status() & HMC5883L_STATUS_DRDY) != 0; }

    //read the raw sensor values (chip axes, in LSB)
    bool readRaw(int16_t* mx, int16_t* my, int16_t* mz) {
      uint8_t d[6];
      //auto increment: read X_MSB, X_LSB, Y_MSB, Y_LSB, Z_MSB, Z_LSB in one transaction
      if(_readRegs(HMC5883L_REG_X_MSB | HMC5883L_AUTO_INC, d, 6) != 6) return false;
      *mx = (int16_t)(((uint16_t)d[0] << 8) | d[1]);
      *my = (int16_t)(((uint16_t)d[2] << 8) | d[3]);
      *mz = (int16_t)(((uint16_t)d[4] << 8) | d[5]);
      return true;
    }

    //read the sensor values in uT, using the NWU convention expected by madflight
    //
    //The HMC5883L measures at most 75Hz, while madflight polls the magnetometer
    //at 100Hz. The data output registers always hold the most recent measurement,
    //so reading them faster than the output data rate simply returns the same
    //sample twice. That is harmless, and it saves an extra i2c transaction for
    //the status register.
    bool read_uT(float* n, float* w, float* u) {
      int16_t x, y, z;
      if(!readRaw(&x, &y, &z)) {
        //i2c error: repeat the previous measurement
        *n = _n; *w = _w; *u = _u;
        return false;
      }
      const float fx = _uT_per_LSB * x;
      const float fy = _uT_per_LSB * y;
      const float fz = _uT_per_LSB * z;
      #if HMC5883L_AXIS_MAP == HMC5883L_AXIS_NWU
        _n = +fy; _w = -fx; _u = -fz;
      #else
        _n = +fx; _w = +fy; _u = +fz;
      #endif
      *n = _n; *w = _w; *u = _u;
      return true;
    }

  private:
    MF_I2C *_i2c = nullptr;
    uint8_t _adr = HMC5883L_ADDRESS;
    uint8_t _gain = HMC5883L_GAIN_2GA;
    float _uT_per_LSB = HMC5883L_uT_per_LSB_2GA;

    int16_t _x = 0, _y = 0, _z = 0; //raw values
    float _n = 0, _w = 0, _u = 0;   //last valid NWU values

    void _writeReg(uint8_t reg, uint8_t val) {
      _i2c->beginTransmission(_adr);
      _i2c->write(reg);
      _i2c->write(val);
      _i2c->endTransmission();
    }

    uint8_t _readReg(uint8_t reg) {
      uint8_t d = 0;
      _readRegs(reg, &d, 1);
      return d;
    }

    //read len consecutive registers, returns the number of bytes read
    uint8_t _readRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
      _i2c->beginTransmission(_adr);
      _i2c->write(reg);
      _i2c->endTransmission(false); //false = repeated start
      uint32_t n = _i2c->requestFrom(_adr, len);
      if(n == len) {
        _i2c->readBytes(buf, len);
        return len;
      }
      return (uint8_t)n;
    }
};
