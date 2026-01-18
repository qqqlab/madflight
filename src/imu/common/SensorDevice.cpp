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

// Interface for SPI/I2C sensors

#include "SensorDevice.h"

SensorDevice* SensorDevice::createImuDevice(ImuConfig *config) {
    if(!config) return nullptr;

    // Detect sensor
    SensorDevice *dev = nullptr;
    if(config->spi_bus) {
        if(config->spi_cs >= 0) {
            dev = new SensorDeviceSPI(config->spi_bus, config->spi_cs, SPI_MODE3);
        }
    }else if(config->i2c_bus) {
        dev = new SensorDeviceI2C(config->i2c_bus, config->i2c_adr);
    }
    if(!dev) Serial.println("asdfasdf");
    return dev;
}