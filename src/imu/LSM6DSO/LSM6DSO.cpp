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

//Driver for LSM6DSO, LSM6DSOX gyroscope/accelometer

#define USE_BF_CONFIG 0

#include "LSM6DSO.h"
#include "LSM6DSO_regs.h"

//returns negative error code, positive warning code, or 0 on success
int LSM6DSO::begin(SensorDevice* dev) { 
    this->dev = dev;
    dev->setLowSpeed(); // Use low speed for setup

    if(!detect(dev)) return -1;
    delay(10);

#if USE_BF_CONFIG
    //based on https://github.com/betaflight/betaflight/blob/master/src/main/drivers/accgyro/accgyro_spi_lsm6dso_init.c

    // Reset the device - default 00000100
    dev->writeReg(LSM6DSO_REG_CTRL3_C, LSM6DSO_MASK_CTRL3_C_RESET);
    delay(100); //wait 100ms before continuing config

    // Configure interrupt pin 1 for gyro data ready only
    ///dev->writeReg(LSM6DSO_REG_INT1_CTRL, LSM6DSO_VAL_INT1_CTRL);

    // pulsed interrupt - COUNTER_BDR_REG1 default = 00000000
    ///dev->writeReg(0x0B, 0b1000000); //DATAREADY_PULSED

    // Disable interrupt pin 2
    dev->writeReg(LSM6DSO_REG_INT2_CTRL, LSM6DSO_VAL_INT2_CTRL);

    // Configure the accelerometer
    // 833hz ODR, 16G scale, use LPF1 output
    dev->writeReg(LSM6DSO_REG_CTRL1_XL, (LSM6DSO_VAL_CTRL1_XL_ODR833 << 4) | (LSM6DSO_VAL_CTRL1_XL_16G << 2) | (LSM6DSO_VAL_CTRL1_XL_LPF1 << 1));

    // Configure the gyro
    // 833hz ODR, 2000dps scale
    dev->writeReg( LSM6DSO_REG_CTRL2_G, (LSM6DSO_VAL_CTRL2_G_ODR833 << 4) | (LSM6DSO_VAL_CTRL2_G_2000DPS << 2));

    // Configure control register 3 - default 00000100
    // latch LSB/MSB during reads; set interrupt pins active high; set interrupt pins push/pull; set 4-wire SPI; enable auto-increment burst reads
    dev->writeReg(LSM6DSO_REG_CTRL3_C, (LSM6DSO_VAL_CTRL3_C_BDU | LSM6DSO_VAL_CTRL3_C_H_LACTIVE | LSM6DSO_VAL_CTRL3_C_PP_OD | LSM6DSO_VAL_CTRL3_C_SIM | LSM6DSO_VAL_CTRL3_C_IF_INC));

    // Configure control register 4 - default 00000000
    // enable accelerometer high performane mode; set gyro LPF1 cutoff to 335.5hz
    dev->writeReg(LSM6DSO_REG_CTRL4_C, (LSM6DSO_VAL_CTRL4_C_I2C_DISABLE | LSM6DSO_VAL_CTRL4_C_LPF1_SEL_G));

    // Configure control register 6 - default 00000000
    // disable I2C interface; enable gyro LPF1
    dev->writeReg(LSM6DSO_REG_CTRL6_C, (LSM6DSO_VAL_CTRL6_C_XL_HM_MODE | LSM6DSO_VAL_CTRL6_C_FTYPE_335HZ));

    // Configure control register 9 - default 00011000
    // disable I3C interface
    dev->writeReg(LSM6DSO_REG_CTRL9_XL, LSM6DSO_VAL_CTRL9_XL_I3C_DISABLE);

    // MF addition: pulsed gyro interrupt
    // INT1_CTRL Configure interrupt pin 1 for gyro data ready only
    dev->writeReg(0x0D, (1<<1));

    // COUNTER_BDR_REG1 pulsed interrupt -  default = 00000000
    dev->writeReg(0x0B, (1<<7)); //DATAREADY_PULSED

#else

    // LSM6DSO_REG_CTRL3_C - Reset the device - default 00000100
    dev->writeReg(0x12, 0x01);
    delay(100); //wait 100ms before continuing config

    // CTRL2_G - set the gyroscope control register to work at 833 Hz, 2000 dps and in bypass mode
    dev->writeReg(0x11, 0x7C);

    // CTRL1_XL - Accelerometer control register 1
    // [7:4] ODR_XL      7 = Accelerometer 833 Hz
    // [3:2] S[1:0]_XL   1 = 16G scale
    // [1]   LPF2_XL_EN  enable LPF2 low pass filter
    dev->writeReg(0x10, (7<<4) | (1<<2) | (1<<1));

    // CTRL7_G - set gyroscope power mode to high performance and high pass cutoff to m16 MHz
    dev->writeReg(0x16, 0x00);

    // CTRL8_XL - Set Acc LPF2 to ODR/4 - default 00000000
    // [7:5] HPCF_XL_ Accelerometer LPF2 and HP filter configuration and cutoff setting. Refer to Table 72.
    // [4]   HP_REF_MODE_XL Enables accelerometer high-pass filter reference mode (valid for high-pass path - HP_SLOPE_XL_EN bit must be 1). Default value: 0(1) (0: disabled, 1: enabled)
    // [3]   FASTSETTL_MODE_XL Enables accelerometer LPF2 and HPF fast-settling mode. The filter sets the second samples after writing this bit. Active only during device exit from power-down mode. Default value: 0 (0: disabled, 1: enabled)
    // [2]   HP_SLOPE_XL_EN Accelerometer slope filter / high-pass filter selection. Refer to Figure 27. 
    // [1]   XL_FS_MODE Accelerometer full-scale management between UI chain and OIS chain (0: old full-scale mode. When the accelerometer UI is on, the full scale is the same between UI/OIS and is chosen by the UI CTRL registers; when the accelerometer UI is in PD, the OIS can choose the FS. 1: new full-scale mode. Full scales are independent between the UI/OIS chain but both bound to ±8 g.)
    // [0]   LOW_PASS_ON_6D LPF2 on 6D function selection. Refer to Figure 27. Default value: 0 (0: ODR/2 low-pass filtered data sent to 6D interrupt function; 1: LPF2 output data sent to 6D interrupt function) 
    dev->writeReg(0x17, 0x08);

    // INT1_CTRL Configure interrupt pin 1 for gyro data ready only
    dev->writeReg(0x0D, (1<<1));

    // COUNTER_BDR_REG1 pulsed interrupt -  default = 00000000
    dev->writeReg(0x0B, (1<<7)); //DATAREADY_PULSED

#endif

    // Set full speed
    dev->setFreq(LSM6DSO_MAX_SPI_CLK_HZ/10);

    return 0;
}

void LSM6DSO::readraw(int16_t *raw) {
    dev->readRegs(LSM6DSO_REG_OUTX_L_G, (uint8_t*)raw, 12); //ax,ay,az,gx,gy,gz little endian, no byte juggling needed for RP2,ESP32,STM32
}
