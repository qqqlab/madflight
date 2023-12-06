//2023-11-02 Invensense MPU-6050 I2C library
//sampling rate acc+gyro 1000Hz
 
 #include "MPU6050.h"

MPU6050::MPU6050(uint8_t low_pass_filter, uint8_t low_pass_filter_acc)
{
  _low_pass_filter = low_pass_filter;
  _low_pass_filter_acc = low_pass_filter_acc;
  acc_multiplier = 1.0 / 2048.0; //default 2G after reset
  gyro_multiplier = 1.0 / 131.0; //default 250 deg/s after reset
}

bool MPU6050::begin()
{
  WriteReg(MPUREG_PWR_MGMT_1, BIT_H_RESET);        // MPU6050 Reset
  delay(10);
  WriteReg(MPUREG_PWR_MGMT_1, 0x01);               // MPU6050 Clock Source XGyro
  WriteReg(MPUREG_PWR_MGMT_2, 0x00);               // MPU6050 Enable Acc & Gyro
  WriteReg(MPUREG_CONFIG, _low_pass_filter);       // MPU6050 Use DLPF set Gyroscope bandwidth 184Hz, acc bandwidth 188Hz
  WriteReg(MPUREG_GYRO_CONFIG, BITS_FS_250DPS);    // MPU6050 +-250dps
  WriteReg(MPUREG_ACCEL_CONFIG, BITS_FS_2G);       // MPU6050 +-2G        

  //enable 50us data ready pulse on int pin
  WriteReg(MPUREG_INT_PIN_CFG, 0x00);
  WriteReg(MPUREG_INT_ENABLE, 0x01);
  
  return 0;
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
int MPU6050::set_acc_scale(int scale)
{
    int temp_scale;
    WriteReg(MPUREG_ACCEL_CONFIG, scale);
    
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
    temp_scale = ReadReg(MPUREG_ACCEL_CONFIG);
    
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

int MPU6050::set_acc_scale_g(int scale_in_g)
{
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
int MPU6050::set_gyro_scale(int scale)
{
    int temp_scale;
    WriteReg(MPUREG_GYRO_CONFIG, scale);

    switch (scale){
        case BITS_FS_250DPS:   gyro_multiplier = 1.0 / 131.0; break;
        case BITS_FS_500DPS:   gyro_multiplier = 1.0 / 65.5; break;
        case BITS_FS_1000DPS:  gyro_multiplier = 1.0 / 32.8; break;
        case BITS_FS_2000DPS:  gyro_multiplier = 1.0 / 16.4; break;   
    }

    temp_scale = ReadReg(MPUREG_GYRO_CONFIG);

    switch (temp_scale){
        case BITS_FS_250DPS:   temp_scale = 250;    break;
        case BITS_FS_500DPS:   temp_scale = 500;    break;
        case BITS_FS_1000DPS:  temp_scale = 1000;   break;
        case BITS_FS_2000DPS:  temp_scale = 2000;   break;   
    }
    return temp_scale;
}

int MPU6050::set_gyro_scale_dps(int scale_in_dps)
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

// MPU6050 should return 0x68
unsigned int MPU6050::whoami()
{
    return ReadReg(MPUREG_WHOAMI);
}


//================================================================
// Read sensor data
//================================================================

//Get sensor data in NED frame
//x=North (forward), y=East (right), z=Down 
//acc: gravitation force is positive in axis direction
//gyro: direction of positive rotation by right hand rule, i.e. positive is: yaw right, roll right, pitch up
void MPU6050::getMotion6NED(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
  read();
  //sensor orientation for acc/gyro is NWU
  *ax = -accel[0];
  *ay = accel[1];
  *az = accel[2];
  *gx = gyro[0];
  *gy = -gyro[1];
  *gz = -gyro[2];
}

void MPU6050::read()
{
    uint8_t response[14];
    int16_t bit_data;
    float data;
    int i,pos;

    ReadRegs(MPUREG_ACCEL_XOUT_H,response,14);
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
}
