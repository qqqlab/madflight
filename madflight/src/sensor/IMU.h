/*========================================================================================================================
This file contains all necessary functions and code used for IMU sensors to avoid cluttering the main code

Each USE_IMU_xxx section in this file defines:
 - int imu_Setup() - init, returns 0 on success.
 - void imu_Read() - returns a sample: acc (g), gyro (deg/s), and when has_mag=true magneto (uT) 

MPU-6XXX and MPU-9XXX sensor family
===================================
These are 6 or 9 axis sensors, with maximum sample rates: gyro 8 kHz, accel 4 kHz, and mag 100 Hz. The driver 
configures gyro and accel with 1000 Hz sample rate (with on sensor 200 Hz low pass filter), and mag 100 Hz.
========================================================================================================================*/


//========================================================================================================================
// MPU6050 I2C gyro/acc
//========================================================================================================================
#if defined USE_IMU_MPU6050_I2C

#include "MPU6050.h"

MPU6050 mpu;

int imu_Setup() {
  Serial.println("USE_IMU_MPU6050_I2C");
  mpu.setI2C(i2c, IMU_I2C_ADR);
  mpu.set_gyro_scale_dps(IMU_GYRO_DPS);
  mpu.set_acc_scale_g(IMU_ACCEL_G); 
  int status = mpu.begin();
  return status;
}

void imu_Read(float *AcX, float *AcY, float *AcZ, float *GyX, float *GyY, float *GyZ, float *MgX, float *MgY, float *MgZ) {
  (void)(MgX); (void)(MgY); (void)(MgZ); //suppress compiler warnings
  mpu.getMotion6NED(AcX, AcY, AcZ, GyX, GyY, GyZ);
}


//========================================================================================================================
// MPU9150 I2C gyro/acc/mag
//========================================================================================================================
#elif defined USE_IMU_MPU9150_I2C

#include "MPU9150.h"

MPU9150 mpu;

int imu_Setup() {
  Serial.println("USE_IMU_MPU9150_I2C");
  mpu.setI2C(i2c, IMU_I2C_ADR);  
  mpu.set_gyro_scale_dps(IMU_GYRO_DPS);
  mpu.set_acc_scale_g(IMU_ACCEL_G); 
  int status = mpu.begin();
  return status;
}

void imu_Read(float *AcX, float *AcY, float *AcZ, float *GyX, float *GyY, float *GyZ, float *MgX, float *MgY, float *MgZ) {
  mpu.getMotion9NED(AcX, AcY, AcZ, GyX, GyY, GyZ, MgX, MgY, MgZ);
}


//========================================================================================================================
// MPU9250 I2C gyro/acc/mag or MPU6500 I2C gyro/acc
//========================================================================================================================
#elif defined USE_IMU_MPU9250_I2C or defined USE_IMU_MPU6500_I2C

#include "MPU9250.h"

MPU9250_I2C mpu;

int imu_Setup() {
  Serial.println("USE_IMU_MPU9250_I2C");
  mpu.setI2C(i2c, IMU_I2C_ADR);
  mpu.set_gyro_scale_dps(IMU_GYRO_DPS);
  mpu.set_acc_scale_g(IMU_ACCEL_G);
  int status = mpu.begin();
#if defined USE_IMU_MPU9250_I2C
  if(status == -1112) {
    Serial.println("WARNING: MPU9250 is actually MPU6500 without magnetometer");
    return 0;
  }
#endif
  return status;
}

void imu_Read(float *AcX, float *AcY, float *AcZ, float *GyX, float *GyY, float *GyZ, float *MgX, float *MgY, float *MgZ) {
  mpu.getMotion9NED(AcX, AcY, AcZ, GyX, GyY, GyZ, MgX, MgY, MgZ); 
}


//========================================================================================================================
// MPU9250 SPI gyro/acc/mag or MPU6500 SPI gyro/acc
//========================================================================================================================
#elif defined USE_IMU_MPU9250_SPI or defined USE_IMU_MPU6500_SPI

#include "MPU9250.h"

MPU9250 mpu();

int imu_Setup() {
  Serial.println("USE_IMU_MPU9250_SPI");
  mpu.setSPI(spi, spi_CS_PIN);
  mpu.set_gyro_scale_dps(IMU_GYRO_DPS);
  mpu.set_acc_scale_g(IMU_ACCEL_G);
  int status = mpu.begin();  
#if defined USE_IMU_MPU9250_SPI
  if(status == -1112) {
    Serial.println("WARNING: MPU9250 is actually MPU6500 without magnetometer");
    return 0;
  }
#endif  
  return status;
}

void imu_Read(float *AcX, float *AcY, float *AcZ, float *GyX, float *GyY, float *GyZ, float *MgX, float *MgY, float *MgZ) {
  mpu.getMotion9NED(AcX, AcY, AcZ, GyX, GyY, GyZ, MgX, MgY, MgZ); 
}

#else
  #error "uncomment one USE_IMU_xxx"
#endif