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

//Driver for LSM6DSV16B, LSM6DSV16BX gyroscope/accelometer

#include "LSM6DSV16B.h"

//returns negative error code, positive warning code, or 0 on success
int LSM6DSV16B::begin(SensorDevice* dev) { 
    this->dev = dev;
    dev->setLowSpeed(); // Use low speed for setup

    // Check who-am-i
    if (!detect(dev)) return -1;

    // Perform a software reset
    dev->writeReg(0x12, 0x01); //CTRL3, SW_RESET

    // Wait for the device to be ready
    while (dev->readReg(0x12) & 0x01) {} //CTRL3, SW_RESET

    // Autoincrement register address when doing block SPI reads and update continuously
    dev->writeReg(0x12, 0x44);  //BDU bit need to be set - CTRL3, IF_INC | BDU

    // Acc 16G sensitivity, LPF1 ODR/2, LPF2 ODR/4 (in high-performance mode)
    dev->writeReg(0x17, 0x03); //CTRL8, FS_XL, HP_LPF2_XL_BW

    // Enable acc LPF2 filter
    dev->writeReg(0x16, 0x08); //CTRL9, LPF2_XL_EN

    // Gyro 2000 deg/s sensitivity, LPF1 240Hz (@960kHz ODR)
    dev->writeReg(0x15, 0x04); //CTRL6, FS_G, LPF1_G_BW

    // Enable gyro LPF1 filter
    dev->writeReg(0x16, 0x01); //CTRL7, LPF1_G_EN

    // Acc ODR 960Hz in high-performance mode
    dev->writeReg(0x10, 0x09); //CTRL1, OP_MODE_XL, ODR_XL

    // Gyro ODR 960Hz in high-performance mode
    dev->writeReg(0x11, 0x09); //CTRL2, OP_MODE_G, ODR_G

    // Generate pulse on interrupt line, not requiring a read to clear
    dev->writeReg(0x13, 0x02); //CTRL4, DRDY_PULSED

    // Enable the INT1 output to interrupt when new gyro data is ready
    dev->writeReg(0x0D, 0x02); //INT1_CTRL, INT1_DRDY_G

    // Set full speed
    dev->setFreq(10000000); //10MHz

    return 0;
}

void LSM6DSV16B::readraw(int16_t *raw) {
  dev->readRegs(0x22, (uint8_t*)raw, 12); //LSM6DSV_OUTX_L_G - gx,gy,gz,az,ay,ax little endian, no byte juggling needed for RP2,ESP32,STM32
}
