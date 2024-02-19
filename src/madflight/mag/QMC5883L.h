#pragma once

//#include "Arduino.h"


/*

FROM QST QMC5883L Datasheet
---------------------------
MODE CONTROL (MODE)
    Standby         0x00
    Continuous      0x01

OUTPUT DATA RATE (ODR)
    10Hz            0x00
    50Hz            0x04
    100Hz           0x08
    200Hz           0x0C

FULL SCALE (RNG)
    2G              0x00  //range = +/-2G (200uT)  LSB = 1/12000 G = 1/120 uT 
    8G              0x10  //range = +/-8G (800uT)  LSB = 1/ 3000 G = 1/ 30 uT 

OVER SAMPLE RATIO (OSR)
    512             0x00
    256             0x40
    128             0x80
    64              0xC0 
*/

#define QMC5883L_uT_per_LSB ((float)8.333333333e-3f) //2G range
//#define QMC5883L_uT_per_LSB ((float)3.333333333e-2f) //8G range

template<typename WireType>
class QMC5883L{
	
  public:
    QMC5883L(WireType* i2c, uint8_t adr = 0) {
        _i2c = i2c;
        setAdr(adr);
    }
    
    void setAdr(uint8_t adr = 0) {
        if(adr) {
            _adr = adr;
        }else{
            _adr = 0x0D;
        }
    }

    bool detect() {
        return ( _readReg(0x0D) == 0xFF );
    }

    void begin(){
        _writeReg(0x0B,0x01);
        setMode(0x01,0x08,0x00,0X00); //Continuous, 100Hz, 2G, 512x OSR
    }

    void setMode(uint8_t mode, uint8_t odr, uint8_t rng, uint8_t osr){
        _writeReg(0x09,mode|odr|rng|osr);
    }

    void reset(){
        _writeReg(0x0A,0x80);
    }

    void readRaw(int16_t* mx, int16_t* my, int16_t* mz){
        _i2c->beginTransmission(_adr);
        _i2c->write(0x00);
        _i2c->endTransmission(false);
        _i2c->requestFrom(_adr, (uint8_t)0x06);
        *mx = (int16_t)(_i2c->read() | _i2c->read() << 8);
        *my = (int16_t)(_i2c->read() | _i2c->read() << 8);
        *mz = (int16_t)(_i2c->read() | _i2c->read() << 8);
    }

    void read_uT(float* mx, float* my, float* mz) {
        int16_t rawmx, rawmy, rawmz;
        readRaw(&rawmx, &rawmy, &rawmz);
        *mx = QMC5883L_uT_per_LSB * rawmx;
        *my = QMC5883L_uT_per_LSB * rawmy;
        *mz = QMC5883L_uT_per_LSB * rawmz;
    }

  private:
    WireType *_i2c;
    uint8_t _adr;
    
    void _writeReg(uint8_t reg, uint8_t val){
        _i2c->beginTransmission(_adr);
        _i2c->write(reg);
        _i2c->write(val);
        _i2c->endTransmission();
    }

    int _readReg(uint8_t reg){
        _i2c->beginTransmission(_adr);
        _i2c->write(reg);
        _i2c->endTransmission(false);
        _i2c->requestFrom(_adr, (uint8_t)6);
        return _i2c->read();
    }
};