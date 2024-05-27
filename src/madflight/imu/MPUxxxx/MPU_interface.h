// madflight https://github.com/qqqlab/madflight
// Interface for MPU sensors

#pragma once

#include "Arduino.h"
#include <SPI.h>

//Datasheet: spi clock up to 1MHz for register operations, up to 20MHz allowed for reading data.
#define MPU_SPI_FREQ_SLOW 1000000
#define MPU_SPI_FREQ_FAST 20000000

//Datasheet: i2c clock up to 400kHz
#define MPU_I2C_FREQ_SLOW 400000
#define MPU_I2C_FREQ_FAST 1000000 //2.5 times overclocking

class MPU_Interface {
  public:
    int freqSlow;
    int freqFast;
  
    virtual void begin() = 0;

    virtual void setFreq(int freq) = 0;

    virtual unsigned int WriteReg( uint8_t reg, uint8_t data ) = 0;
    
    virtual void ReadRegs( uint8_t reg, uint8_t *data, uint8_t n ) = 0;
    
    unsigned int ReadReg(uint8_t reg) {
      uint8_t data = 0;
      ReadRegs(reg, &data, 1);
      return data;
    }
    
    inline void setFreqSlow() {
      setFreq(freqSlow);
    }

    inline void setFreqFast() {
      setFreq(freqFast);
    }
};

//================================================================
// SPI
//================================================================
class MPU_InterfaceSPI : public MPU_Interface {
  public:
    MPU_InterfaceSPI(SPIClass *spi, uint8_t cs) {
      _spi = spi; 
      _spi_cs = cs;
      freqSlow = MPU_SPI_FREQ_SLOW;
      freqFast = MPU_SPI_FREQ_FAST;
      setFreq(freqSlow);
    }

    virtual void begin() {
      pinMode(_spi_cs, OUTPUT);
      digitalWrite(_spi_cs, HIGH);
    }

    void setFreq(int freq) {
      _freq = freq;
    }

    virtual unsigned int WriteReg( uint8_t reg, uint8_t data ) {
      _spi->beginTransaction(SPISettings(_freq, MSBFIRST, SPI_MODE3));
      digitalWrite(_spi_cs, LOW);
      _spi->transfer(reg & 0x7f);
      unsigned int temp_val = _spi->transfer(data);
      digitalWrite(_spi_cs, HIGH);
      _spi->endTransaction();
      return temp_val;
    }

    virtual void ReadRegs( uint8_t reg, uint8_t *data, uint8_t n ) {
      _spi->beginTransaction(SPISettings(_freq, MSBFIRST, SPI_MODE3));
      digitalWrite(_spi_cs, LOW);
      _spi->transfer(reg | 0x80);
      for(int i = 0; i < n; i++) {
        data[i] = _spi->transfer(0x00);
      }
      digitalWrite(_spi_cs, HIGH);
      _spi->endTransaction();
    }

private:
    SPIClass * _spi;
    int _freq;  
    uint8_t _spi_cs;
    unsigned int _WriteReg_SPI(uint8_t reg, uint8_t data);
    void _ReadRegs_SPI(uint8_t reg, uint8_t *buf, int n);
};

//================================================================
// I2C
//================================================================

template <typename WireType>
class MPU_InterfaceI2C : public MPU_Interface{
  public:
    MPU_InterfaceI2C(WireType *i2c, uint8_t i2c_adr) {
      _i2c = i2c;
      _i2c_adr = i2c_adr;
      freqSlow = MPU_I2C_FREQ_SLOW;
      freqFast = MPU_I2C_FREQ_FAST;
    }

    virtual void begin() {
      setFreq(freqSlow);
    }

    void setFreq(int freq) {
      _i2c->setClock(freq);
    }

    virtual unsigned int WriteReg( uint8_t reg, uint8_t data ) {
      _i2c->beginTransmission(_i2c_adr); 
      _i2c->write(reg);
      _i2c->write(data);
      _i2c->endTransmission();
      //Serial.printf("WriteReg(reg0x%02X, data=0x%02X) --> ", reg, data); ReadReg(reg);
      return 0;
    }

    virtual void ReadRegs( uint8_t reg, uint8_t *data, uint8_t n ) {
      _i2c->beginTransmission(_i2c_adr); 
      _i2c->write(reg);
      _i2c->endTransmission(false); //false = repeated start
      uint8_t bytesReceived = _i2c->requestFrom(_i2c_adr, n);
      if(bytesReceived == n) {
        _i2c->readBytes(data, bytesReceived);
      }
      //Serial.printf("ReadRegs(reg=0x%02X, n=%d) --> data[%d]=0x%02X\n", reg, n, bytesReceived, data[0]);
    }

private:
    WireType *_i2c;
    uint8_t _i2c_adr;
    unsigned int _WriteReg_I2C(uint8_t reg, uint8_t data);
    void _ReadRegs_I2C(uint8_t reg, uint8_t *buf, uint8_t n);
};





// MPU9250 registers (NOTE: used MPU6050 and MPU9150 registers are the same)
#define MPUREG_XG_OFFS_TC 0x00
#define MPUREG_YG_OFFS_TC 0x01
#define MPUREG_ZG_OFFS_TC 0x02
#define MPUREG_X_FINE_GAIN 0x03
#define MPUREG_Y_FINE_GAIN 0x04
#define MPUREG_Z_FINE_GAIN 0x05
#define MPUREG_XA_OFFS_H 0x06
#define MPUREG_XA_OFFS_L 0x07
#define MPUREG_YA_OFFS_H 0x08
#define MPUREG_YA_OFFS_L 0x09
#define MPUREG_ZA_OFFS_H 0x0A
#define MPUREG_ZA_OFFS_L 0x0B
#define MPUREG_PRODUCT_ID 0x0C
#define MPUREG_SELF_TEST_X 0x0D
#define MPUREG_SELF_TEST_Y 0x0E
#define MPUREG_SELF_TEST_Z 0x0F
#define MPUREG_SELF_TEST_A 0x10
#define MPUREG_XG_OFFS_USRH 0x13
#define MPUREG_XG_OFFS_USRL 0x14
#define MPUREG_YG_OFFS_USRH 0x15
#define MPUREG_YG_OFFS_USRL 0x16
#define MPUREG_ZG_OFFS_USRH 0x17
#define MPUREG_ZG_OFFS_USRL 0x18
#define MPUREG_SMPLRT_DIV 0x19
#define MPUREG_CONFIG 0x1A
#define MPUREG_GYRO_CONFIG 0x1B
#define MPUREG_ACCEL_CONFIG 0x1C
#define MPUREG_ACCEL_CONFIG_2      0x1D
#define MPUREG_LP_ACCEL_ODR        0x1E
#define MPUREG_MOT_THR             0x1F
#define MPUREG_FIFO_EN             0x23
#define MPUREG_I2C_MST_CTRL        0x24
#define MPUREG_I2C_SLV0_ADDR       0x25
#define MPUREG_I2C_SLV0_REG        0x26
#define MPUREG_I2C_SLV0_CTRL       0x27
#define MPUREG_I2C_SLV1_ADDR       0x28
#define MPUREG_I2C_SLV1_REG        0x29
#define MPUREG_I2C_SLV1_CTRL       0x2A
#define MPUREG_I2C_SLV2_ADDR       0x2B
#define MPUREG_I2C_SLV2_REG        0x2C
#define MPUREG_I2C_SLV2_CTRL       0x2D
#define MPUREG_I2C_SLV3_ADDR       0x2E
#define MPUREG_I2C_SLV3_REG        0x2F
#define MPUREG_I2C_SLV3_CTRL       0x30
#define MPUREG_I2C_SLV4_ADDR       0x31
#define MPUREG_I2C_SLV4_REG        0x32
#define MPUREG_I2C_SLV4_DO         0x33
#define MPUREG_I2C_SLV4_CTRL       0x34
#define MPUREG_I2C_SLV4_DI         0x35
#define MPUREG_I2C_MST_STATUS      0x36
#define MPUREG_INT_PIN_CFG 0x37
#define MPUREG_INT_ENABLE 0x38
#define MPUREG_ACCEL_XOUT_H 0x3B
#define MPUREG_ACCEL_XOUT_L 0x3C
#define MPUREG_ACCEL_YOUT_H 0x3D
#define MPUREG_ACCEL_YOUT_L 0x3E
#define MPUREG_ACCEL_ZOUT_H 0x3F
#define MPUREG_ACCEL_ZOUT_L 0x40
#define MPUREG_TEMP_OUT_H 0x41
#define MPUREG_TEMP_OUT_L 0x42
#define MPUREG_GYRO_XOUT_H 0x43
#define MPUREG_GYRO_XOUT_L 0x44
#define MPUREG_GYRO_YOUT_H 0x45
#define MPUREG_GYRO_YOUT_L 0x46
#define MPUREG_GYRO_ZOUT_H 0x47
#define MPUREG_GYRO_ZOUT_L 0x48
#define MPUREG_EXT_SENS_DATA_00    0x49
#define MPUREG_EXT_SENS_DATA_01    0x4A
#define MPUREG_EXT_SENS_DATA_02    0x4B
#define MPUREG_EXT_SENS_DATA_03    0x4C
#define MPUREG_EXT_SENS_DATA_04    0x4D
#define MPUREG_EXT_SENS_DATA_05    0x4E
#define MPUREG_EXT_SENS_DATA_06    0x4F
#define MPUREG_EXT_SENS_DATA_07    0x50
#define MPUREG_EXT_SENS_DATA_08    0x51
#define MPUREG_EXT_SENS_DATA_09    0x52
#define MPUREG_EXT_SENS_DATA_10    0x53
#define MPUREG_EXT_SENS_DATA_11    0x54
#define MPUREG_EXT_SENS_DATA_12    0x55
#define MPUREG_EXT_SENS_DATA_13    0x56
#define MPUREG_EXT_SENS_DATA_14    0x57
#define MPUREG_EXT_SENS_DATA_15    0x58
#define MPUREG_EXT_SENS_DATA_16    0x59
#define MPUREG_EXT_SENS_DATA_17    0x5A
#define MPUREG_EXT_SENS_DATA_18    0x5B
#define MPUREG_EXT_SENS_DATA_19    0x5C
#define MPUREG_EXT_SENS_DATA_20    0x5D
#define MPUREG_EXT_SENS_DATA_21    0x5E
#define MPUREG_EXT_SENS_DATA_22    0x5F
#define MPUREG_EXT_SENS_DATA_23    0x60
#define MPUREG_I2C_SLV0_DO         0x63
#define MPUREG_I2C_SLV1_DO         0x64
#define MPUREG_I2C_SLV2_DO         0x65
#define MPUREG_I2C_SLV3_DO         0x66
#define MPUREG_I2C_MST_DELAY_CTRL  0x67
#define MPUREG_SIGNAL_PATH_RESET   0x68
#define MPUREG_MOT_DETECT_CTRL     0x69
#define MPUREG_USER_CTRL           0x6A
#define MPUREG_PWR_MGMT_1          0x6B
#define MPUREG_PWR_MGMT_2          0x6C
#define MPUREG_BANK_SEL            0x6D
#define MPUREG_MEM_START_ADDR      0x6E
#define MPUREG_MEM_R_W             0x6F
#define MPUREG_DMP_CFG_1           0x70
#define MPUREG_DMP_CFG_2           0x71
#define MPUREG_FIFO_COUNTH         0x72
#define MPUREG_FIFO_COUNTL         0x73
#define MPUREG_FIFO_R_W            0x74
#define MPUREG_WHOAMI              0x75
#define MPUREG_XA_OFFSET_H         0x77
#define MPUREG_XA_OFFSET_L         0x78
#define MPUREG_YA_OFFSET_H         0x7A
#define MPUREG_YA_OFFSET_L         0x7B
#define MPUREG_ZA_OFFSET_H         0x7D
#define MPUREG_ZA_OFFSET_L         0x7E
 
// Configuration bits mpu9250
#define BIT_SLEEP 0x40
#define BIT_H_RESET 0x80
#define BITS_CLKSEL 0x07
#define MPU_CLK_SEL_PLLGYROX 0x01
#define MPU_CLK_SEL_PLLGYROZ 0x03
#define MPU_EXT_SYNC_GYROX 0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_2G                  0x00
#define BITS_FS_4G                  0x08
#define BITS_FS_8G                  0x10
#define BITS_FS_16G                 0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN              0x01
#define BIT_I2C_IF_DIS              0x10
 
#define READ_FLAG   0x80
 
/* ---- Sensitivity --------------------------------------------------------- */
 
#define MPU9250A_2g       ((float)0.000061035156f) // 0.000061035156 g/LSB
#define MPU9250A_4g       ((float)0.000122070312f) // 0.000122070312 g/LSB
#define MPU9250A_8g       ((float)0.000244140625f) // 0.000244140625 g/LSB
#define MPU9250A_16g      ((float)0.000488281250f) // 0.000488281250 g/LSB
 
#define MPU9250G_250dps   ((float)0.007633587786f) // 0.007633587786 dps/LSB
#define MPU9250G_500dps   ((float)0.015267175572f) // 0.015267175572 dps/LSB
#define MPU9250G_1000dps  ((float)0.030487804878f) // 0.030487804878 dps/LSB
#define MPU9250G_2000dps  ((float)0.060975609756f) // 0.060975609756 dps/LSB
 
 
#define MPU9250T_85degC   ((float)0.002995177763f) // 0.002995177763 degC/LSB
