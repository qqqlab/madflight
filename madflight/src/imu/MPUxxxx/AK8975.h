// madflight https://github.com/qqqlab/madflight
// 2023-12-27 Invensense MPU-9150 I2C library
// sampling rate acc+gyro 1000Hz, mag 100Hz
// DON'T access AK8963 after begin() - the mag will stop sending data...
 
//================================================================
// AK8975 Magnetometer - internal to MPU9150
//================================================================

#define AK8975_uT_per_LSB ((float)0.3f)

/* ---- AK8975 Reg In MPU9150 ----------------------------------------------- */
#define AK8975_I2C_ADDR             0x0C
#define AK8975_Device_ID            0x48

// Read-only Reg
#define AK8975_WIA                  0x00
#define AK8975_INFO                 0x01
#define AK8975_ST1                  0x02
#define AK8975_HXL                  0x03
#define AK8975_HXH                  0x04
#define AK8975_HYL                  0x05
#define AK8975_HYH                  0x06
#define AK8975_HZL                  0x07
#define AK8975_HZH                  0x08
#define AK8975_ST2                  0x09
// Write/Read Reg
#define AK8975_CNTL                 0x0A
#define AK8975_RSV                  0x0B
#define AK8975_ASTC                 0x0C
#define AK8975_TS1                  0x0D
#define AK8975_TS2                  0x0E
#define AK8975_I2CDIS               0x0F
// Read-only Reg ( ROM )
#define AK8975_ASAX                 0x10
#define AK8975_ASAY                 0x11
#define AK8975_ASAZ                 0x12

class AK8975 {

  public:
    float mag_multiplier[3];

    AK8975(MPU_Interface *iface) {
        _iface = iface;
    }
    
    int begin()
    {
      _iface->WriteReg(MPUREG_I2C_MST_CTRL, 0x0D);  // MPU9150I2C master clock speed 400KHz
      _iface->WriteReg(MPUREG_USER_CTRL, 0x20);     // MPU9150 Enable I2C master mode 

      //warm up AK8975
      for(int i=0;i<10;i++) {
          int wai = AK8975_whoami();
          //Serial.printf("AK8975_whoami()=0x%x\n",wai);
          if(wai == 0x48) break;
          delay(10);
      }

      AK8975_getASA();

      _iface->WriteReg(MPUREG_USER_CTRL, 0x00); //disable master I2C

      // Let I2C slave get Mag data, these commands repeated with the sample rate (1kHz) 
      // slave0: get 7 bytes of data
      _iface->WriteReg(MPUREG_I2C_SLV0_ADDR, AK8975_I2C_ADDR|READ_FLAG); //write operation
      _iface->WriteReg(MPUREG_I2C_SLV0_REG, AK8975_HXL); //mag data register
      _iface->WriteReg(MPUREG_I2C_SLV0_CTRL, 0x87);   //enable, read 7 bytes (3*2 mag + status)
      // slave1: restart mag sampling
      _iface->WriteReg(MPUREG_I2C_SLV1_ADDR, AK8975_I2C_ADDR);  //write operation
      _iface->WriteReg(MPUREG_I2C_SLV1_REG, AK8975_CNTL);  //write to mode register
      _iface->WriteReg(MPUREG_I2C_SLV1_DO, 0x01);  //data to write (set single conversion mode)
      _iface->WriteReg(MPUREG_I2C_SLV1_CTRL, 0x81); //enable, read 1 bytes (need to read at least one byte!!!!)

      _iface->WriteReg(MPUREG_I2C_MST_DELAY_CTRL, 0x03+0x80); //wait for SLV0+SLV1, shadow
      _iface->WriteReg(MPUREG_I2C_SLV4_CTRL, 9); //read slaves every n+1 samples -> 100Hz

      _iface->WriteReg(MPUREG_USER_CTRL, 0x00); //clear 
      _iface->WriteReg(MPUREG_USER_CTRL, 0x02); //I2C_MST_RESET i2c reset
      _iface->WriteReg(MPUREG_USER_CTRL, 0x20); //enable master I2C

      return 0;
    }

  private:

    MPU_Interface *_iface;

    void AK8975_getASA()
    {
        AK8975_WriteReg(AK8975_CNTL, 0x00);                               // set AK8975 to Power Down
        delayMicroseconds(1000);                                    // datasheet: 100us
        AK8975_WriteReg(AK8975_CNTL, 0x0F);                               // set AK8975 to FUSE ROM access
        delayMicroseconds(1000);                                    // datasheet: 0us

        for(int i = 0; i < 3; i++) {
            int ASA = AK8975_ReadReg(AK8975_ASAX + i);
            //Serial.printf("AK8975_ASA%d=%d ",i,ASA);
            mag_multiplier[i] = 1.0;
            if(ASA>=0) mag_multiplier[i] = (((float)ASA+128)/256) * AK8975_uT_per_LSB;
            //Serial.printf("mag_multiplier%d=%f\n",i,mag_multiplier[i]);
        }
        AK8975_WriteReg(AK8975_CNTL, 0x00);                               // set AK8975 to Power Down
        delayMicroseconds(1000);                                    // datasheet: 100us
    }

    uint8_t AK8975_whoami(){
        return AK8975_ReadReg(AK8975_WIA);
    }

    int AK8975_ReadReg(uint8_t reg)
    {
      _iface->WriteReg(MPUREG_I2C_SLV4_ADDR, AK8975_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8975 and set for reading.
      _iface->WriteReg(MPUREG_I2C_SLV4_REG, reg); //I2C slave 0 register address from where to begin data transfer
      _iface->WriteReg(MPUREG_I2C_SLV4_CTRL, 0x80); //Enable I2C transfer
      uint32_t now = micros();
      while(micros() - now < 4000) {
        //wait for I2C_SLV4_DONE
        if( _iface->ReadReg(MPUREG_I2C_MST_STATUS) & 0x40 ) {
          return _iface->ReadReg(MPUREG_I2C_SLV4_DI);
        }
      }
      return -1;
    }

    bool AK8975_WriteReg(uint8_t reg, uint8_t data) 
    {
      _iface->WriteReg(MPUREG_I2C_SLV4_ADDR, AK8975_I2C_ADDR); //Set the I2C slave addres of AK8975 and set for writing.
      _iface->WriteReg(MPUREG_I2C_SLV4_REG, reg); //I2C slave 0 register address from where to begin data transfer
      _iface->WriteReg(MPUREG_I2C_SLV4_DO, data);   // Reset AK8975
      _iface->WriteReg(MPUREG_I2C_SLV4_CTRL, 0x81); //Enable I2C transfer
      uint32_t now = micros();
      while(micros() - now < 2000) {
        //wait for I2C_SLV4_DONE 
        if( _iface->ReadReg(MPUREG_I2C_MST_STATUS) & 0x40 ) {
          return true;
        }
      }
      return false;
    }
};
