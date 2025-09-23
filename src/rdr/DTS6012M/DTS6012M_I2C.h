/*==========================================================================================
MIT License

Copyright (c) 2025 https://madflight.com

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

/* DTS6012M I2C wiring
1 3.3V
2 3.3V
3 SDA (has internal 2.2K pullup)
4 SCL (has internal 2.2K pullup)
5 floating or 10K pullup to 3.3V (I2C selection and interrupt output)
6 GND
*/

#include "../../hal/hal.h"

class DTS6012M_I2C {
private:
     MF_I2CDevice *i2c_sensor = nullptr;
public:
    const uint32_t interval = 10000; //100 Hz update rate
    uint32_t update_ts = 0;
    int16_t distance = -1;

    bool begin(MF_I2C* i2c_bus, uint8_t adr) {
        if(adr == 0) adr = 0x51; //default address
        i2c_sensor = new MF_I2CDevice(i2c_bus, adr);
        int tries = 5;
        do {
            i2c_sensor->writeReg(0x02, 0x01); //start sensor
            update_ts = micros();
            uint32_t ts = millis();
            while(millis() - ts <= 100) {
                if(update()) return true;
            }
            tries--;
       }while(tries);
       return false;
    }

    bool update() {
        //keep exact 100 Hz timing
        uint32_t now = micros();
        if(now - update_ts < interval) return false; 
        if(now - update_ts < 2 * interval) {
            update_ts += interval;
        }else{
            update_ts = now; //resync
        }

        uint8_t data[2];
        if(i2c_sensor->readReg(0x00, data, 2) != 2 ) return false;
        int16_t dist = (int16_t)((data[0] << 8) | data[1]);
        if((dist > 0 && dist <= 20000) || dist == -1) {
            distance = dist;
            return true;
        }
        return false;
    }
};