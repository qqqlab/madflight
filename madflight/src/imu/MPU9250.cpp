//2023-11-03 Invensense MPU-9250 SPI/I2C library
 
 #include "MPU9250.h"

MPU9250::MPU9250(MPU_Interface *iface, uint8_t low_pass_filter, uint8_t low_pass_filter_acc) {
  _iface = iface;
  _init(low_pass_filter, low_pass_filter_acc);
}

void MPU9250::_init(uint8_t low_pass_filter, uint8_t low_pass_filter_acc) {
  _low_pass_filter = low_pass_filter;
  _low_pass_filter_acc = low_pass_filter_acc;
  acc_multiplier = 1.0 / 2048.0; //default 2G after reset
  gyro_multiplier = 1.0 / 131.0; //default 250 deg/s after reset
  mag_multiplier[0] = AK8963_uT_per_LSB;
  mag_multiplier[1] = AK8963_uT_per_LSB;
  mag_multiplier[2] = AK8963_uT_per_LSB;    
}

int MPU9250::begin() {
  _iface->begin();

  _iface->WriteReg(MPUREG_PWR_MGMT_1, BIT_H_RESET);        // Reset MPU9250
  delay(10);
  
  int rv = 0;
  int id = MPU9250::whoami();
  if(id!=0x71) rv = -(1000 + id);  //0x70 (112) = MPU6500, 0x71 (113) = MPU9250
  
  _iface->WriteReg(MPUREG_PWR_MGMT_1, 0x01);               // Clock Source
  _iface->WriteReg(MPUREG_PWR_MGMT_2, 0x00);               // Enable Acc & Gyro
  _iface->WriteReg(MPUREG_CONFIG, _low_pass_filter);       // Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz
  _iface->WriteReg(MPUREG_GYRO_CONFIG, BITS_FS_250DPS);    // +-250dps
  _iface->WriteReg(MPUREG_ACCEL_CONFIG, BITS_FS_2G);       // +-2G        
  _iface->WriteReg(MPUREG_ACCEL_CONFIG_2, _low_pass_filter_acc); // Set Acc Data Rates, Enable Acc LPF , Bandwidth 184Hz
  _iface->WriteReg(MPUREG_I2C_MST_CTRL, 0x0D);     // I2C master clock speed 400KHz
  _iface->WriteReg(MPUREG_USER_CTRL, 0x30);        // Enable I2C master mode, disable slave mode I2C bus  

  //enable 50us data ready pulse on int pin
  _iface->WriteReg(MPUREG_INT_PIN_CFG, 0x00);
  _iface->WriteReg(MPUREG_INT_ENABLE, 0x01);

  set_acc_scale(BITS_FS_2G);
  set_gyro_scale(BITS_FS_250DPS);

  //setup AK8963
  int rv2 = AK8963_begin();
  return (rv==0 ? rv2 : rv);
}

/*                                ACCELEROMETER SCALE
 * usage: call this function at startup, after initialization, to set the right range for the
 * accelerometers. Suitable ranges are:
 * BITS_FS_2G
 * BITS_FS_4G
 * BITS_FS_8G
 * BITS_FS_16G
 * returns the range set (2,4,8 or 16)
 */
int MPU9250::set_acc_scale(int scale) {
    int temp_scale;
    _iface->WriteReg(MPUREG_ACCEL_CONFIG, scale);
    
    switch (scale){
        case BITS_FS_2G:
            acc_multiplier = 1.0 / 16384.0;
        break;
        case BITS_FS_4G:
            acc_multiplier = 1.0 / 8192.0;
        break;
        case BITS_FS_8G:
            acc_multiplier = 1.0 / 4096.0;
        break;
        case BITS_FS_16G:
            acc_multiplier = 1.0 / 2048.0;
        break;   
    }
    temp_scale = _iface->ReadReg(MPUREG_ACCEL_CONFIG);
    
    switch (temp_scale){
        case BITS_FS_2G:
            temp_scale=2;
        break;
        case BITS_FS_4G:
            temp_scale=4;
        break;
        case BITS_FS_8G:
            temp_scale=8;
        break;
        case BITS_FS_16G:
            temp_scale=16;
        break;   
    }
    return temp_scale;
}

int MPU9250::set_acc_scale_g(int scale_in_g) {
    if(scale_in_g <= 2) {
      return set_acc_scale(BITS_FS_2G);
    }else if(scale_in_g <= 4) { 
      return set_acc_scale(BITS_FS_4G);
    }else if(scale_in_g <= 8) { 
      return set_acc_scale(BITS_FS_8G);
    }else{ 
      return set_acc_scale(BITS_FS_16G);
    }
}


/*                                 GYROSCOPE SCALE
 * usage: call this function at startup, after initialization, to set the right range for the
 * gyroscopes. Suitable ranges are:
 * BITS_FS_250DPS
 * BITS_FS_500DPS
 * BITS_FS_1000DPS
 * BITS_FS_2000DPS
 * returns the range set (250,500,1000 or 2000)
 */
int MPU9250::set_gyro_scale(int scale) {
    int temp_scale;
    _iface->WriteReg(MPUREG_GYRO_CONFIG, scale);

    switch (scale){
        case BITS_FS_250DPS:   gyro_multiplier = 1.0 / 131.0; break;
        case BITS_FS_500DPS:   gyro_multiplier = 1.0 / 65.5; break;
        case BITS_FS_1000DPS:  gyro_multiplier = 1.0 / 32.8; break;
        case BITS_FS_2000DPS:  gyro_multiplier = 1.0 / 16.4; break;   
    }

    temp_scale = _iface->ReadReg(MPUREG_GYRO_CONFIG);

    switch (temp_scale){
        case BITS_FS_250DPS:   temp_scale = 250;    break;
        case BITS_FS_500DPS:   temp_scale = 500;    break;
        case BITS_FS_1000DPS:  temp_scale = 1000;   break;
        case BITS_FS_2000DPS:  temp_scale = 2000;   break;   
    }
    return temp_scale;
}

int MPU9250::set_gyro_scale_dps(int scale_in_dps)
{
    if(scale_in_dps <= 250) {
      return set_gyro_scale(BITS_FS_250DPS);
    }else if(scale_in_dps <= 500) { 
      return set_gyro_scale(BITS_FS_500DPS);
    }else if(scale_in_dps <= 1000) { 
      return set_gyro_scale(BITS_FS_1000DPS);
    }else{ 
      return set_gyro_scale(BITS_FS_2000DPS);
    }
}

// MPU9250 should return 0x71
unsigned int MPU9250::whoami() {
    return _iface->ReadReg(MPUREG_WHOAMI|READ_FLAG);
}


//================================================================
// Read sensor data
//================================================================

//Get sensor data in NED frame
//x=North (forward), y=East (right), z=Down 
//acc: gravitation force is positive in axis direction (sensor reports negative)
//gyro: direction of positive rotation by right hand rule, i.e. positive is: yaw right, roll right, pitch up
void MPU9250::getMotion9NED(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *mx, float *my, float *mz)
{
  read();
  //sensor orientation for acc/gyro is NWU
  *ax = -accel[0]; //-N
  *ay = accel[1];  //-E
  *az = accel[2];  //-D
  *gx = gyro[0];   //N
  *gy = -gyro[1];  //E
  *gz = -gyro[2];  //D
  //sensor orientation for mag is WND
  *mx = mag[1];    //N
  *my = -mag[0];   //E
  *mz = mag[2];    //D
}

void MPU9250::getMotion6NED(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
  read();
  //sensor orientation for acc/gyro is NWU
  *ax = -accel[0]; //-N
  *ay = accel[1];  //-E
  *az = accel[2];  //-D
  *gx = gyro[0];   //N
  *gy = -gyro[1];  //E
  *gz = -gyro[2];  //D
}

//read sensor data (axis as defined by sensor)
void MPU9250::read()
{
    uint8_t response[20];
    int16_t bit_data;
    float data;
    int i,pos;

    _iface->ReadRegs(MPUREG_ACCEL_XOUT_H,response,20);
    // Get accelerometer value (6 bytes)
    pos = 0;
    for(i = 0; i < 3; i++) {
        bit_data = ((int16_t)response[pos]<<8) | response[pos+1];
        data = (float)bit_data;
        accel[i] = data * acc_multiplier;
        pos += 2;;
    }
    // Get temperature (2 bytes)
    bit_data = ((int16_t)response[pos]<<8) | response[pos+1];
    data = (float)bit_data;
    temperature = ((data-21)/333.87)+21;
    pos += 2;
    // Get gyroscope value (6 bytes)
    for(i=0; i < 3; i++) {
        bit_data = ((int16_t)response[pos]<<8) | response[pos+1];
        data = (float)bit_data;
        gyro[i] = data * gyro_multiplier;
        pos += 2;
    }
    // Get Magnetometer value (6 bytes)
    for(i=0; i < 3; i++) {
        bit_data = ((int16_t)response[pos+1]<<8) | response[pos];
        data = (float)bit_data;
        mag[i] = data * mag_multiplier[i];
        pos += 2;
    }
}

//================================================================
// AK8963 Magnetometer
//================================================================

//error codes 2000, warning codes -2000
int MPU9250::AK8963_begin()
{
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
  _iface->WriteReg(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR|READ_FLAG); // Set the I2C slave addres of AK8963 and set for read.
  _iface->WriteReg(MPUREG_I2C_SLV0_REG, AK8963_HXL); // I2C slave 0 register address from where to begin data transfer
  _iface->WriteReg(MPUREG_I2C_SLV0_CTRL, 0x87); // Read 7 bytes from the magnetometer
  // must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.

  return rv;
}

void MPU9250::AK8963_getASA()
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

uint8_t MPU9250::AK8963_whoami(){
    return AK8963_ReadReg(AK8963_WIA);
}

int MPU9250::AK8963_ReadReg(uint8_t reg)
{
  _iface->WriteReg(MPUREG_I2C_SLV4_ADDR, AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for reading.
  _iface->WriteReg(MPUREG_I2C_SLV4_REG, reg); //I2C slave 0 register address from where to begin data transfer
  _iface->WriteReg(MPUREG_I2C_SLV4_CTRL, 0x80); //Enable I2C transfer
  uint32_t now = micros();
  while(micros() - now < 2000) {
    //wait for I2C_SLV4_DONE
    if( _iface->ReadReg(MPUREG_I2C_MST_STATUS) & 0x40 ) {
      return _iface->ReadReg(MPUREG_I2C_SLV4_DI);
    }
  }
  return -1;
}

bool MPU9250::AK8963_WriteReg(uint8_t reg, uint8_t data) 
{
  _iface->WriteReg(MPUREG_I2C_SLV4_ADDR, AK8963_I2C_ADDR); //Set the I2C slave addres of AK8963 and set for writing.
  _iface->WriteReg(MPUREG_I2C_SLV4_REG, reg); //I2C slave 0 register address from where to begin data transfer
  _iface->WriteReg(MPUREG_I2C_SLV4_DO, data); //Reset AK8963
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
