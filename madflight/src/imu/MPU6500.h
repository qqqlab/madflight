//2023-12-27 Invensense MPU-6500 library
//sampling rate acc+gyro 1000Hz

#pragma once

#include "MPU_Interface.h"

class MPU6500 {
private:
    MPU_Interface *_iface;

public:
    float accel[3];
    float gyro[3];
    float temperature;
    float acc_multiplier;
    float gyro_multiplier;


MPU60X0(MPU_Interface *iface) {
  _iface = iface;
}

bool begin(int gyro_scale_dps=250, int acc_scale_g=2) {
  //start interface
  _iface->begin();

  //config
  _iface->WriteReg(MPUREG_PWR_MGMT_1, BIT_H_RESET);        // Reset
  delay(20);
  _iface->WriteReg(MPUREG_PWR_MGMT_1, 0x01);               // Clock Source XGyro
  _iface->WriteReg(MPUREG_PWR_MGMT_2, 0x00);               // Enable Acc & Gyro
  _iface->WriteReg(MPUREG_CONFIG, BITS_DLPF_CFG_188HZ);    // Use DLPF set Gyroscope bandwidth 184Hz, acc bandwidth 188Hz
  
  //set scale
  set_gyro_scale_dps(gyro_scale_dps);
  set_acc_scale_g(acc_scale_g);

  //enable 50us data ready pulse on int pin
  _iface->WriteReg(MPUREG_INT_PIN_CFG, 0x00);
  _iface->WriteReg(MPUREG_INT_ENABLE, 0x01);

  //check whoami
  int wai = whoami();
  if(wai != 0x70) Serial.printf("WARNING: MPU6500 whoami mismatch, got:0x%02X expected:0x70\n",wai);

  return 0;
}

// MPU6500 should return 0x70
unsigned int whoami()
{
    _iface->setFreqSlow();
    return _iface->ReadReg(MPUREG_WHOAMI);
}

void set_acc_scale_g(int scale_in_g)
{
    _iface->setFreqSlow();
    if(scale_in_g <= 2) {
      _iface->WriteReg(MPUREG_ACCEL_CONFIG, BITS_FS_2G);
      acc_multiplier = 2.0 / 32786.0;
    }else if(scale_in_g <= 4) { 
      _iface->WriteReg(MPUREG_ACCEL_CONFIG, BITS_FS_4G);
      acc_multiplier = 4.0 / 32786.0;
    }else if(scale_in_g <= 8) { 
      _iface->WriteReg(MPUREG_ACCEL_CONFIG, BITS_FS_8G);
      acc_multiplier = 8.0 / 32786.0;
    }else{ 
      _iface->WriteReg(MPUREG_ACCEL_CONFIG, BITS_FS_16G);
      acc_multiplier = 16.0 / 32786.0;
    }
}

void set_gyro_scale_dps(int scale_in_dps)
{
    _iface->setFreqSlow();
    if(scale_in_dps <= 250) {
      _iface->WriteReg(MPUREG_GYRO_CONFIG, BITS_FS_250DPS);
      gyro_multiplier = 250.0 / 32768.0;
    }else if(scale_in_dps <= 500) { 
      _iface->WriteReg(MPUREG_GYRO_CONFIG, BITS_FS_500DPS);
      gyro_multiplier = 500.0 / 32768.0;
    }else if(scale_in_dps <= 1000) { 
      _iface->WriteReg(MPUREG_GYRO_CONFIG, BITS_FS_1000DPS);
      gyro_multiplier = 1000.0 / 32768.0;
    }else{ 
      _iface->WriteReg(MPUREG_GYRO_CONFIG, BITS_FS_2000DPS);
      gyro_multiplier = 2000.0 / 32768.0;
    }
}

//Get sensor data in NED frame
//x=North (forward), y=East (right), z=Down 
//acc: gravitation force is positive in axis direction
//gyro: direction of positive rotation by right hand rule, i.e. positive is: yaw right, roll right, pitch up
void getMotion6NED(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
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

void read()
{
    uint8_t response[14];
    int16_t bit_data;
    float data;
    int i,pos;

    _iface->setFreqFast();
    _iface->ReadRegs(MPUREG_ACCEL_XOUT_H,response,14);
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

};
