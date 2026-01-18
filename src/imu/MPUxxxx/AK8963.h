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

// Driver for AK8963
 
#include "MPU_regs.h"

//================================================================
// AK8963 Magnetometer - internal to MPU9250
//================================================================

#define AK8963_uT_per_LSB ((float)0.15f)

/* ---- AK8963 Reg In MPU9250 ----------------------------------------------- */
#define AK8963_I2C_ADDR             0x0c//0x18
#define AK8963_Device_ID            0x48

// Read-only Reg
#define AK8963_WIA                  0x00
#define AK8963_INFO                 0x01
#define AK8963_ST1                  0x02
#define AK8963_HXL                  0x03
#define AK8963_HXH                  0x04
#define AK8963_HYL                  0x05
#define AK8963_HYH                  0x06
#define AK8963_HZL                  0x07
#define AK8963_HZH                  0x08
#define AK8963_ST2                  0x09
// Write/Read Reg
#define AK8963_CNTL1                0x0A
#define AK8963_CNTL2                0x0B
#define AK8963_ASTC                 0x0C
#define AK8963_TS1                  0x0D
#define AK8963_TS2                  0x0E
#define AK8963_I2CDIS               0x0F
// Read-only Reg ( ROM )
#define AK8963_ASAX                 0x10
#define AK8963_ASAY                 0x11
#define AK8963_ASAZ                 0x12

class AK8963 {

  public:
    float mag_multiplier[3];

    AK8963(SensorDevice *dev) {
        _dev = dev;
    }

    //error codes 2000, warning codes -2000
    int begin()
    {
      _dev->writeReg(MPUREG_I2C_MST_CTRL, 0x0D);     // I2C master clock speed 400KHz
      //_dev->writeReg(MPUREG_USER_CTRL, 0x30);        // Enable I2C master mode, disable slave mode I2C bus --> ONLY DO THIS IN SPI MODE, disables the external I2C interface....
      _dev->writeReg(MPUREG_USER_CTRL, 0x20);        // Enable I2C master mode 

      int rv = 0;

      if(!AK8963_WriteReg(AK8963_CNTL2, 0x01)) return 2001;  // Reset AK8963
      delay(10);

      int id = AK8963_ReadReg(AK8963_WIA);
      if(id!=0x48) rv = -(2000 + id);

      AK8963_getASA();

      if(!AK8963_WriteReg(AK8963_CNTL1, 0x16)) return 2002;  // Set 100Hz continuous measurement in 16bit
      //if(!AK8963_WriteReg(AK8963_CNTL1, 0x12)) return -1;  // Set 8Hz continuous measurement in 16bit

      // Send I2C command to get Mag data, this command is repeated with the sample rate (1kHz) 
      // as long as these MPU registers are not overwritten with an another command
      _dev->writeReg(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR|READ_FLAG); // Set the I2C slave addres of AK8963 and set for read.
      _dev->writeReg(MPUREG_I2C_SLV0_REG, AK8963_HXL); // I2C slave 0 register address from where to begin data transfer
      _dev->writeReg(MPUREG_I2C_SLV0_CTRL, 0x87); // Read 7 bytes from the magnetometer
      // must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.

      return rv;
    }

  private:

    SensorDevice *_dev;

    void AK8963_getASA()
    {
        AK8963_WriteReg(AK8963_CNTL1, 0x00); // set AK8963 to Power Down
        delayMicroseconds(1000); // datasheet: 100us
        AK8963_WriteReg(AK8963_CNTL1, 0x0F); // set AK8963 to FUSE ROM access
        delayMicroseconds(1000); // datasheet: 0us

        for(int i = 0; i < 3; i++) {
            int ASA = AK8963_ReadReg(AK8963_ASAX + i);
            //Serial.printf("ASA%d=%d\n",i,ASA);
            mag_multiplier[i] = 1.0;
            if(ASA>=0) mag_multiplier[i] = (((float)ASA+128)/256) * AK8963_uT_per_LSB;
            //Serial.printf("mag_multiplier%d=%f\n",i,mag_multiplier[i]);
        }
        AK8963_WriteReg(AK8963_CNTL1, 0x00); // set AK8963 to Power Down
        delayMicroseconds(1000); // datasheet: 100us
    }

    uint8_t AK8963_whoami(){
        return AK8963_ReadReg(AK8963_WIA);
    }

    int AK8963_ReadReg(uint8_t reg)
    {
      _dev->writeReg(MPUREG_I2C_SLV4_ADDR, AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for reading.
      _dev->writeReg(MPUREG_I2C_SLV4_REG, reg); //I2C slave 0 register address from where to begin data transfer
      _dev->writeReg(MPUREG_I2C_SLV4_CTRL, 0x80); //Enable I2C transfer
      uint32_t now = micros();
      while(micros() - now < 2000) {
        //wait for I2C_SLV4_DONE
        if( _dev->readReg(MPUREG_I2C_MST_STATUS) & 0x40 ) {
          return _dev->readReg(MPUREG_I2C_SLV4_DI);
        }
      }
      return -1;
    }

    bool AK8963_WriteReg(uint8_t reg, uint8_t data) 
    {
      _dev->writeReg(MPUREG_I2C_SLV4_ADDR, AK8963_I2C_ADDR); //Set the I2C slave addres of AK8963 and set for writing.
      _dev->writeReg(MPUREG_I2C_SLV4_REG, reg); //I2C slave 0 register address from where to begin data transfer
      _dev->writeReg(MPUREG_I2C_SLV4_DO, data); //Reset AK8963
      _dev->writeReg(MPUREG_I2C_SLV4_CTRL, 0x81); //Enable I2C transfer
      uint32_t now = micros();
      while(micros() - now < 2000) {
        //wait for I2C_SLV4_DONE 
        if( _dev->readReg(MPUREG_I2C_MST_STATUS) & 0x40 ) {
          return true;
        }
      }
      return false;
    }
};