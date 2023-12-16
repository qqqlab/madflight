/*========================================================================================================================
This file contains all necessary functions and code used for IMU sensors to avoid cluttering the main code

Each USE_IMU_xxx section in this file defines:
 - int imu_Setup() - init, returns 0 on success.
 - void imu_Read() - returns a sample: acc (g), gyro (deg/s), and when has_mag=true magneto (uT)

Body frame is NED: x-axis North(front), y-axis East(right), z-axis Down

MPU-6XXX and MPU-9XXX sensor family
===================================
These are 6 or 9 axis sensors, with maximum sample rates: gyro 8 kHz, accel 4 kHz, and mag 100 Hz. The driver 
configures gyro and accel with 1000 Hz sample rate (with on sensor 200 Hz low pass filter), and mag 100 Hz.
========================================================================================================================*/

//make sure exactly one bus type is defined, default to I2C
#ifdef USE_IMU_BUS_SPI
  #define IMU_BUS_TYPE "SPI"
  #ifdef USE_IMU_BUS_I2C
    #undefine USE_IMU_BUS_I2C
  #endif
#elif defined USE_IMU_BUS_I2C
  #define IMU_BUS_TYPE "I2C"
#else
  #error "define USE_IMU_BUS_SPI or USE_IMU_BUS_I2C"
#endif

//handle rotation for different mounting positions
#if defined IMU_ROTATE_YAW90
  #define IMO_ROTATE() do{ float tmp; tmp=*ax; *ax=-*ay; *ay=tmp;   tmp=*gx; *gx=-*gy; *gy=tmp;   tmp=*mx; *mx=-*my; *my=tmp; }while(0)
#elif defined IMU_ROTATE_YAW180
  #define IMO_ROTATE() do{ *ax=-*ax; *ay=-*ay;   *gx=-*gx; *gy=-*gy;   *mx=-*mx; *my=-*my; }while(0)
#elif defined IMU_ROTATE_YAW270
  #define IMO_ROTATE() do{ float tmp; tmp=*ax; *ax=*ay; *ay=-tmp;   tmp=*gx; *gx=*gy; *gy=-tmp;   tmp=*mx; *mx=*my; *my=-tmp; }while(0)
#elif defined IMU_ROTATE_ROLL180
  #define IMO_ROTATE() do{ *ay=-*ay; *az=-*az;   *gy=-*gy; *gz=-*gz;   *my=-*my; *mz=-*mz; }while(0)
#elif defined IMU_ROTATE_YAW90_ROLL180
  #define IMO_ROTATE() do{ float tmp; tmp=*ax; *ax=*ay; *ay=tmp; *az=-*az;   tmp=*gx; *gx=*gy; *gy=tmp; *gz=-*gz;   tmp=*mx; *mx=*my; *my=tmp; *mz=-*mz; }while(0)
#elif defined IMU_ROTATE_YAW180_ROLL180
  #define IMO_ROTATE() do{ *ax=-*ax; *az=-*az;   *gx=-*gx; *gz=-*gz;   *mx=-*mx; *mz=-*mz; }while(0)
#elif defined IMU_ROTATE_YAW270_ROLL180
  #define IMO_ROTATE() do{ float tmp; tmp=*ax; *ax=-*ay; *ay=-tmp; *az=-*az;   tmp=*gx; *gx=-*gy; *gy=-tmp; *gz=-*gz;   tmp=*mx; *mx=-*my; *my=-tmp; *mz=-*mz; }while(0)
#elif 
  #define IMO_ROTATE()
#endif

//========================================================================================================================
// MPU6050 I2C gyro/acc
//========================================================================================================================
#if defined USE_IMU_BUS_I2C && defined USE_IMU_MPU6050

#include "MPU6050.h"

MPU_InterfaceI2C<HW_WIRETYPE> mpu_iface(i2c, IMU_I2C_ADR);
MPU6050 mpu(&mpu_iface);

int imu_Setup() {
  Serial.println("USE_IMU_MPU6050 " IMU_BUS_TYPE);
  mpu.set_gyro_scale_dps(IMU_GYRO_DPS);
  mpu.set_acc_scale_g(IMU_ACCEL_G); 
  int status = mpu.begin();
  return status;
}

void imu_Read(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *mx, float *my, float *mz) {
  (void)(mx); (void)(my); (void)(mz); //suppress compiler warnings
  mpu.getMotion6NED(ax, ay, az, gx, gy, gz);
  IMO_ROTATE();
}

//========================================================================================================================
// MPU9150 I2C gyro/acc/mag
//========================================================================================================================
#elif defined USE_IMU_BUS_I2C && defined USE_IMU_MPU9150

#include "MPU9150.h"

MPU_InterfaceI2C<HW_WIRETYPE> mpu_iface(i2c, IMU_I2C_ADR);
MPU9150 mpu(&mpu_iface);

int imu_Setup() {
  Serial.println("USE_IMU_MPU9150 " IMU_BUS_TYPE);
  mpu.set_gyro_scale_dps(IMU_GYRO_DPS);
  mpu.set_acc_scale_g(IMU_ACCEL_G);
  int status = mpu.begin();
  return status;
}

void imu_Read(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *mx, float *my, float *mz) {
  mpu.getMotion9NED(ax, ay, az, gx, gy, gz, mx, my, mz);
  IMO_ROTATE();
}

//========================================================================================================================
// MPU9250 SPI/I2C gyro/acc/mag or MPU6500 I2C gyro/acc
//========================================================================================================================
#elif defined USE_IMU_MPU9250 || defined USE_IMU_MPU6500

#include "MPU9250.h"

#ifdef USE_IMU_BUS_SPI
  MPU_InterfaceSPI mpu_iface(spi, HW_PIN_SPI_CS);
#else
  MPU_InterfaceI2C<HW_WIRETYPE> mpu_iface(i2c, IMU_I2C_ADR);
#endif

MPU9250 mpu(&mpu_iface);

int imu_Setup() {
#ifdef USE_IMU_MPU6500
  Serial.println("USE_IMU_MPU6500 " IMU_BUS_TYPE);
#else
  Serial.println("USE_IMU_MPU9250 " IMU_BUS_TYPE);
#endif
  mpu.set_gyro_scale_dps(IMU_GYRO_DPS);
  mpu.set_acc_scale_g(IMU_ACCEL_G);
  int status = mpu.begin();
  if(status == -1112) {
#if defined USE_IMU_MPU9250
    Serial.println("WARNING: MPU9250 is actually MPU6500 without magnetometer");
#endif
    return 0;
  }
  return status;
}

void imu_Read(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *mx, float *my, float *mz) {
  mpu.getMotion9NED(ax, ay, az, gx, gy, gz, mx, my, mz);
  IMO_ROTATE();
}

#else
  #error "uncomment a correct combination of USE_IMU_xxx and USE_IMU_BUS_xxx"
#endif
