
/*==========================================================================================
Driver for LSM6DSO gyroscope/accelometer

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

//based on https://github.com/betaflight/betaflight/blob/master/src/main/drivers/accgyro/accgyro_spi_lsm6dso_init.c

// 10 MHz max SPI frequency
#define LSM6DSO_MAX_SPI_CLK_HZ 10000000

#define LSM6DSO_CHIP_ID 0x6C

// equivalent to 70 mdps/LSB, as specified in LSM6DSO datasheet section 4.1, symbol G_So
#define LSM6DSO_GYRO_SCALE_2000DPS 0.070f


// LSM6DSO registers (not the complete list)
typedef enum {
    LSM6DSO_REG_INT1_CTRL = 0x0D,  // int pin 1 control
    LSM6DSO_REG_INT2_CTRL = 0x0E,  // int pin 2 control
    LSM6DSO_REG_WHO_AM_I = 0x0F,   // chip ID
    LSM6DSO_REG_CTRL1_XL = 0x10,   // accelerometer control
    LSM6DSO_REG_CTRL2_G = 0x11,    // gyro control
    LSM6DSO_REG_CTRL3_C = 0x12,    // control register 3
    LSM6DSO_REG_CTRL4_C = 0x13,    // control register 4
    LSM6DSO_REG_CTRL5_C = 0x14,    // control register 5
    LSM6DSO_REG_CTRL6_C = 0x15,    // control register 6
    LSM6DSO_REG_CTRL7_G = 0x16,    // control register 7
    LSM6DSO_REG_CTRL8_XL = 0x17,   // control register 8
    LSM6DSO_REG_CTRL9_XL = 0x18,   // control register 9
    LSM6DSO_REG_CTRL10_C = 0x19,   // control register 10
    LSM6DSO_REG_STATUS = 0x1E,     // status register
    LSM6DSO_REG_OUT_TEMP_L = 0x20, // temperature LSB
    LSM6DSO_REG_OUT_TEMP_H = 0x21, // temperature MSB
    LSM6DSO_REG_OUTX_L_G = 0x22,   // gyro X axis LSB
    LSM6DSO_REG_OUTX_H_G = 0x23,   // gyro X axis MSB
    LSM6DSO_REG_OUTY_L_G = 0x24,   // gyro Y axis LSB
    LSM6DSO_REG_OUTY_H_G = 0x25,   // gyro Y axis MSB
    LSM6DSO_REG_OUTZ_L_G = 0x26,   // gyro Z axis LSB
    LSM6DSO_REG_OUTZ_H_G = 0x27,   // gyro Z axis MSB
    LSM6DSO_REG_OUTX_L_A = 0x28,   // acc X axis LSB
    LSM6DSO_REG_OUTX_H_A = 0x29,   // acc X axis MSB
    LSM6DSO_REG_OUTY_L_A = 0x2A,   // acc Y axis LSB
    LSM6DSO_REG_OUTY_H_A = 0x2B,   // acc Y axis MSB
    LSM6DSO_REG_OUTZ_L_A = 0x2C,   // acc Z axis LSB
    LSM6DSO_REG_OUTZ_H_A = 0x2D,   // acc Z axis MSB
} lsm6dsoRegister_e;


// LSM6DSO register configuration values
typedef enum {
    LSM6DSO_VAL_INT1_CTRL = 0x02,             // enable gyro data ready interrupt pin 1
    LSM6DSO_VAL_INT2_CTRL = 0x02,             // enable gyro data ready interrupt pin 2
    LSM6DSO_VAL_CTRL1_XL_ODR833 = 0x07,       // accelerometer 833hz output data rate (gyro/8)
    LSM6DSO_VAL_CTRL1_XL_ODR1667 = 0x08,      // accelerometer 1666hz output data rate (gyro/4)
    LSM6DSO_VAL_CTRL1_XL_ODR3332 = 0x09,      // accelerometer 3332hz output data rate (gyro/2)
    LSM6DSO_VAL_CTRL1_XL_ODR3333 = 0x0A,      // accelerometer 6664hz output data rate (gyro/1)
    LSM6DSO_VAL_CTRL1_XL_8G = 0x03,           // accelerometer 8G scale
    LSM6DSO_VAL_CTRL1_XL_16G = 0x01,          // accelerometer 16G scale
    LSM6DSO_VAL_CTRL1_XL_LPF1 = 0x00,         // accelerometer output from LPF1
    LSM6DSO_VAL_CTRL1_XL_LPF2 = 0x01,         // accelerometer output from LPF2
    LSM6DSO_VAL_CTRL2_G_ODR833 = 0x07,        // gyro 833hz output data rate
    LSM6DSO_VAL_CTRL2_G_ODR6664 = 0x0A,       // gyro 6664hz output data rate
    LSM6DSO_VAL_CTRL2_G_2000DPS = 0x03,       // gyro 2000dps scale
    LSM6DSO_VAL_CTRL3_C_BDU = (1<<6),         // (bit 6) output registers are not updated until MSB and LSB have been read (prevents MSB from being updated while burst reading LSB/MSB)
    LSM6DSO_VAL_CTRL3_C_H_LACTIVE = 0,        // (bit 5) interrupt pins active high
    LSM6DSO_VAL_CTRL3_C_PP_OD = 0,            // (bit 4) interrupt pins push/pull
    LSM6DSO_VAL_CTRL3_C_SIM = 0,              // (bit 3) SPI 4-wire interface mode
    LSM6DSO_VAL_CTRL3_C_IF_INC = (1<<2),      // (bit 2) auto-increment address for burst reads
    LSM6DSO_VAL_CTRL4_C_I2C_DISABLE = (1<<2), // (bit 2) disable I2C interface
    LSM6DSO_VAL_CTRL4_C_LPF1_SEL_G = (1<<1),  // (bit 1) enable gyro LPF1
    LSM6DSO_VAL_CTRL6_C_XL_HM_MODE = 0,       // (bit 4) enable accelerometer high performance mode
    LSM6DSO_VAL_CTRL6_C_FTYPE_335HZ = 0x00,   // (bits 2:0) gyro LPF1 cutoff 335.5hz
    LSM6DSO_VAL_CTRL6_C_FTYPE_232HZ = 0x01,   // (bits 2:0) gyro LPF1 cutoff 232.0hz
    LSM6DSO_VAL_CTRL6_C_FTYPE_171HZ = 0x02,   // (bits 2:0) gyro LPF1 cutoff 171.1hz
    LSM6DSO_VAL_CTRL6_C_FTYPE_609HZ = 0x03,   // (bits 2:0) gyro LPF1 cutoff 609.0hz
    LSM6DSO_VAL_CTRL9_XL_I3C_DISABLE = (1<<1),// (bit 1) disable I3C interface
} lsm6dsoConfigValues_e;

// LSM6DSO register configuration bit masks
typedef enum {
    LSM6DSO_MASK_CTRL3_C = 0x7C,         // 0b01111100
    LSM6DSO_MASK_CTRL3_C_RESET = (1<<0), // 0b00000001
    LSM6DSO_MASK_CTRL4_C = 0x06,         // 0b00000110
    LSM6DSO_MASK_CTRL6_C = 0x17,         // 0b00010111
    LSM6DSO_MASK_CTRL9_XL = 0x02,        // 0b00000010
} lsm6dsoConfigMasks_e;