/*==========================================================================================
Driver for LSM6DSV gyroscope/accelometer

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

//based on https://raw.githubusercontent.com/betaflight/betaflight/refs/heads/master/src/main/drivers/accgyro/accgyro_spi_lsm6dsv16x.c

#include "LSM6DSV.h"
#include "LSM6DSV_regs.h"
#include "../MPUxxxx/MPU_interface.h"


bool LSM6DSV::detect(MPU_Interface* dev) {
    return (dev->readReg(LSM6DSV_WHO_AM_I) == 0x70);
}

//returns negative error code, positive warning code, or 0 on success
int LSM6DSV::begin(SPIClass *spi, int cs_pin, int sample_rate) { 
    // Use low speed for setup
    dev = new MPU_InterfaceSPI(spi, cs_pin);
    dev->setFreq(1000000);

    // Check who-am-i
    if (!detect(dev)) return -1;

    // Perform a software reset
    dev->writeReg(LSM6DSV_CTRL3, LSM6DSV_CTRL3_SW_RESET);

    // Wait for the device to be ready
    while (dev->readReg(LSM6DSV_CTRL3) & LSM6DSV_CTRL3_SW_RESET) {}

    // Autoincrement register address when doing block SPI reads and update continuously
    dev->writeReg(LSM6DSV_CTRL3, LSM6DSV_CTRL3_IF_INC | LSM6DSV_CTRL3_BDU);      /*BDU bit need to be set*/

    // Select high-accuracy ODR mode 1
    dev->writeReg(LSM6DSV_HAODR_CFG,
                LSM6DSV_ENCODE_BITS(LSM6DSV_HAODR_MODE1,
                                    LSM6DSV_HAODR_CFG_HAODR_SEL_MASK,
                                    LSM6DSV_HAODR_CFG_HAODR_SEL_SHIFT));

    // Enable the accelerometer in high accuracy
    dev->writeReg(LSM6DSV_CTRL1,
                LSM6DSV_ENCODE_BITS(LSM6DSV_CTRL1_OP_MODE_XL_HIGH_ACCURACY,
                                    LSM6DSV_CTRL1_OP_MODE_XL_MASK,
                                    LSM6DSV_CTRL1_OP_MODE_XL_SHIFT));

    // Enable the gyro in high accuracy
    dev->writeReg(LSM6DSV_CTRL2,
                LSM6DSV_ENCODE_BITS(LSM6DSV_CTRL2_OP_MODE_G_HIGH_ACCURACY,
                                    LSM6DSV_CTRL2_OP_MODE_G_MASK,
                                    LSM6DSV_CTRL2_OP_MODE_G_SHIFT));

    // Enable 16G sensitivity
    dev->writeReg(LSM6DSV_CTRL8,
                LSM6DSV_ENCODE_BITS(LSM6DSV_CTRL8_FS_XL_16G,
                                    LSM6DSV_CTRL8_FS_XL_MASK,
                                    LSM6DSV_CTRL8_FS_XL_SHIFT));

    // Enable the accelerometer odr at 1kHz
    dev->writeReg(LSM6DSV_CTRL1,
                LSM6DSV_ENCODE_BITS(LSM6DSV_CTRL1_ODR_XL_1000HZ,
                                    LSM6DSV_CTRL1_ODR_XL_MASK,
                                    LSM6DSV_CTRL1_ODR_XL_SHIFT));

    // Enable the gyro odr at 1kHz
    dev->writeReg(LSM6DSV_CTRL2,
                LSM6DSV_ENCODE_BITS(LSM6DSV_CTRL2_ODR_G_1000HZ,
                                    LSM6DSV_CTRL2_ODR_G_MASK,
                                    LSM6DSV_CTRL2_ODR_G_SHIFT));

    // Set the LPF1 filter bandwidth - Enable 2000 deg/s sensitivity and selected LPF1 filter setting
/*  filter options
    [LPF_NORMAL] = LSM6DSV_CTRL6_FS_G_BW_288HZ,
    [LPF_OPTION_1] = LSM6DSV_CTRL6_FS_G_BW_157HZ,
    [LPF_OPTION_2] = LSM6DSV_CTRL6_FS_G_BW_215HZ,
    [LPF_EXPERIMENTAL] = LSM6DSV_CTRL6_FS_G_BW_455HZ
*/    
    dev->writeReg(LSM6DSV_CTRL6,
                LSM6DSV_ENCODE_BITS(LSM6DSV_CTRL6_FS_G_BW_288HZ,
                                    LSM6DSV_CTRL6_LPF1_G_BW_MASK,
                                    LSM6DSV_CTRL6_LPF1_G_BW_SHIFT) |
                LSM6DSV_ENCODE_BITS(LSM6DSV_CTRL6_FS_G_2000DPS,
                                    LSM6DSV_CTRL6_FS_G_MASK,
                                    LSM6DSV_CTRL6_FS_G_SHIFT));

    // Enable the gyro digital LPF1 filter
    dev->writeReg(LSM6DSV_CTRL7, LSM6DSV_CTRL7_LPF1_G_EN);

    // Generate pulse on interrupt line, not requiring a read to clear
    dev->writeReg(LSM6DSV_CTRL4, LSM6DSV_CTRL4_DRDY_PULSED);

    // Enable the INT1 output to interrupt when new gyro data is ready
    dev->writeReg(LSM6DSV_INT1_CTRL, LSM6DSV_INT1_CTRL_INT1_DRDY_G);

    // Set full speed
    dev->setFreq(LSM6DSV16X_MAX_SPI_CLK_HZ);

    return 0;
}

void LSM6DSV::readraw(int16_t *raw) {
  dev->readRegs(LSM6DSV_OUTX_L_G, (uint8_t*)raw, 12); //ax,ay,az,gx,gy,gz little endian, no byte juggling needed for RP2,ESP32,STM32
}
