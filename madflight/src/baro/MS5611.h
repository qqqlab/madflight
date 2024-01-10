//based on: https://github.com/jarzebski/Arduino-MS5611/tree/dev

/*

The MIT License

Copyright (c) 2014-2023 Korneliusz Jarzębski

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

*/

#pragma once

#include "Arduino.h"

#define MS5611_ADDRESS                (0x77)

#define MS5611_CMD_ADC_READ           (0x00)
#define MS5611_CMD_RESET              (0x1E)
#define MS5611_CMD_CONV_D1            (0x40)
#define MS5611_CMD_CONV_D2            (0x50)
#define MS5611_CMD_READ_PROM          (0xA2)

typedef enum
{
    MS5611_ULTRA_HIGH_RES   = 0x08,
    MS5611_HIGH_RES         = 0x06,
    MS5611_STANDARD         = 0x04,
    MS5611_LOW_POWER        = 0x02,
    MS5611_ULTRA_LOW_POWER  = 0x00
} ms5611_osr_t;

class MS5611
{
public:
    bool compensation;
    MS5611() {
        compensation = true;
    }
    bool begin(ms5611_osr_t osr = MS5611_HIGH_RES);
    uint32_t readRawTemperature();
    uint32_t readRawPressure();
    float readTemperature();
    float readPressure();
    double getAltitude(double pressure, double seaLevelPressure = 101325);
    double getSeaLevel(double pressure, double altitude);
    void setOversampling(ms5611_osr_t osr);
    ms5611_osr_t getOversampling();

    void startTemperature() {
        i2c->beginTransmission(MS5611_ADDRESS);
        i2c->write(MS5611_CMD_CONV_D2 + uosr);
        i2c->endTransmission();
    }

    void startPressure() {
        i2c->beginTransmission(MS5611_ADDRESS);
        i2c->write(MS5611_CMD_CONV_D1 + uosr);
        i2c->endTransmission();
    }

    int getDelayUs() {
        return sample_delay_us;
    }
    
    int getMeasurements(float *press, float* temp) {
        int rv;
        if(micros() - sample_micros < sample_delay_us) {
            rv = 0;
        }else if(sample_cnt == 0) {
            readTemperature();
            startPressure();
            sample_micros = micros();
            sample_cnt++;
            rv = 2;
        }else{
            readPressure();
            if(sample_cnt < sample_interval) {
                startPressure();
                sample_micros = micros();
                sample_cnt++;
            }else{
                startTemperature();
                sample_micros = micros();
                sample_cnt = 0;
            }
            rv = 1;
        }
        *press = _press;
        *temp = _temp;
        return rv;
    }

private:
    uint16_t fc[6];

    uint8_t uosr;
    int32_t _TEMP;
    int32_t _dT;
    float _temp;
    float _press;
    

    void reset(void);
    void readPROM(void);

    uint32_t sample_delay_us;
    uint32_t sample_micros;
    uint32_t sample_cnt;
    uint32_t sample_interval;

    uint16_t readRegister16(uint8_t reg);
    uint32_t readRegister24(uint8_t reg);
};


#include <math.h>

bool MS5611::begin(ms5611_osr_t osr)
{
    i2c->begin();
    reset();
    setOversampling(osr);
    delay(100);
    readPROM();
    
    //get initial readings
    startTemperature();
    delayMicroseconds(sample_delay_us);
    readTemperature();

    startPressure();
    delayMicroseconds(sample_delay_us);
    readPressure();

    //start automatic measurements
    startPressure();
    sample_micros = micros();
    sample_cnt = 1;
    sample_interval = 20; //1 temp sample, 19 press samples

    return true;
}



// Set oversampling value
void MS5611::setOversampling(ms5611_osr_t osr)
{
    switch (osr)
    {
    case MS5611_ULTRA_LOW_POWER:
        sample_delay_us = 1000;
        break;
    case MS5611_LOW_POWER:
        sample_delay_us = 2000;
        break;
    case MS5611_STANDARD:
        sample_delay_us = 3000;
        break;
    case MS5611_HIGH_RES:
        sample_delay_us = 5000;
        break;
    case MS5611_ULTRA_HIGH_RES:
        sample_delay_us = 10000;
        break;
    }

    uosr = osr;
}

// Get oversampling value
ms5611_osr_t MS5611::getOversampling()
{
    return (ms5611_osr_t)uosr;
}

void MS5611::reset(void)
{
    i2c->beginTransmission(MS5611_ADDRESS);
    i2c->write(MS5611_CMD_RESET);
    i2c->endTransmission();
}

void MS5611::readPROM()
{
    for (uint8_t offset = 0; offset < 6; offset++)
    {
        fc[offset] = readRegister16(MS5611_CMD_READ_PROM + (offset * 2));
    }
}

uint32_t MS5611::readRawTemperature()
{
    return readRegister24(MS5611_CMD_ADC_READ);
}

uint32_t MS5611::readRawPressure()
{
    return readRegister24(MS5611_CMD_ADC_READ);
}


float MS5611::readPressure()
{
    uint32_t D1 = readRawPressure();
    int64_t OFF = (int64_t)fc[1] * 65536 + (int64_t)fc[3] * _dT / 128;
    int64_t SENS = (int64_t)fc[0] * 32768 + (int64_t)fc[2] * _dT / 256;
    if (compensation)
    {
        int64_t OFF2 = 0;
        int64_t SENS2 = 0;
        if (_TEMP < 2000)
        {
            OFF2 = 5 * ((_TEMP - 2000) * (_TEMP - 2000)) / 2;
            SENS2 = 5 * ((_TEMP - 2000) * (_TEMP - 2000)) / 4;
        }
        if (_TEMP < -1500)
        {
            OFF2 = OFF2 + 7 * ((_TEMP + 1500) * (_TEMP + 1500));
            SENS2 = SENS2 + 11 * ((_TEMP + 1500) * (_TEMP + 1500)) / 2;
        }
        OFF = OFF - OFF2;
        SENS = SENS - SENS2;
    }

    _press = (D1 * SENS / 2097152 - OFF) / 32768.f;
    return _press;
}

float MS5611::readTemperature()
{
    uint32_t D2 = readRawTemperature();
    int32_t dT = D2 - (uint32_t)fc[4] * 256;
    int32_t TEMP = 2000 + ((int64_t) dT * fc[5]) / 8388608;
    int32_t TEMP2 = 0;
    if (compensation)
    {
        if (TEMP < 2000)
        {
            TEMP2 = (dT * dT) / pow(2, 31);
        }
    }
    TEMP = TEMP - TEMP2;
    _dT = dT;
    _TEMP = TEMP;
    _temp = _TEMP / 100.f ;
    return _temp;
}

// Calculate altitude from Pressure & Sea level pressure
double MS5611::getAltitude(double pressure, double seaLevelPressure)
{
    return (44330.0f * (1.0f - pow((double)pressure / (double)seaLevelPressure, 0.1902949f)));
}

// Calculate sea level from Pressure given on specific altitude
double MS5611::getSeaLevel(double pressure, double altitude)
{
    return ((double)pressure / pow(1.0f - ((double)altitude / 44330.0f), 5.255f));
}

// Read 16-bit from register (oops MSB, LSB)
uint16_t MS5611::readRegister16(uint8_t reg)
{
    uint16_t value;
    i2c->beginTransmission(MS5611_ADDRESS);
    i2c->write(reg);
    i2c->endTransmission();
    i2c->requestFrom(MS5611_ADDRESS, 2);
    uint8_t vha = i2c->read();
    uint8_t vla = i2c->read();
    value = vha << 8 | vla;
    return value;
}

// Read 24-bit from register (oops XSB, MSB, LSB)
uint32_t MS5611::readRegister24(uint8_t reg)
{
    uint32_t value;
    i2c->beginTransmission(MS5611_ADDRESS);
    i2c->write(reg);
    i2c->endTransmission();
    i2c->requestFrom(MS5611_ADDRESS, 3);
    uint8_t vxa = i2c->read();
    uint8_t vha = i2c->read();
    uint8_t vla = i2c->read();
    value = ((int32_t)vxa << 16) | ((int32_t)vha << 8) | vla;
    return value;
}