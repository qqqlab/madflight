/*========================================================================================================================
This file contains all necessary functions and code used for IMU sensors to avoid cluttering the main code

Each USE_IMU_xxx section in this file defines:
 - int imu_Setup() - init, returns 0 on success.
 - void imu_Read() - reads acc,gyro,mag sample from sensor and returns: acc (g), gyro (deg/s)

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
  #define IMU_ROTATE() do{ float tmp; tmp=*ax; *ax=-*ay; *ay=tmp;   tmp=*gx; *gx=-*gy; *gy=tmp;   tmp=*mx; *mx=-*my; *my=tmp; }while(0)
#elif defined IMU_ROTATE_YAW180
  #define IMU_ROTATE() do{ *ax=-*ax; *ay=-*ay;   *gx=-*gx; *gy=-*gy;   *mx=-*mx; *my=-*my; }while(0)
#elif defined IMU_ROTATE_YAW270
  #define IMU_ROTATE() do{ float tmp; tmp=*ax; *ax=*ay; *ay=-tmp;   tmp=*gx; *gx=*gy; *gy=-tmp;   tmp=*mx; *mx=*my; *my=-tmp; }while(0)
#elif defined IMU_ROTATE_ROLL180
  #define IMU_ROTATE() do{ *ay=-*ay; *az=-*az;   *gy=-*gy; *gz=-*gz;   *my=-*my; *mz=-*mz; }while(0)
#elif defined IMU_ROTATE_YAW90_ROLL180
  #define IMU_ROTATE() do{ float tmp; tmp=*ax; *ax=*ay; *ay=tmp; *az=-*az;   tmp=*gx; *gx=*gy; *gy=tmp; *gz=-*gz;   tmp=*mx; *mx=*my; *my=tmp; *mz=-*mz; }while(0)
#elif defined IMU_ROTATE_YAW180_ROLL180
  #define IMU_ROTATE() do{ *ax=-*ax; *az=-*az;   *gx=-*gx; *gz=-*gz;   *mx=-*mx; *mz=-*mz; }while(0)
#elif defined IMU_ROTATE_YAW270_ROLL180
  #define IMU_ROTATE() do{ float tmp; tmp=*ax; *ax=-*ay; *ay=-tmp; *az=-*az;   tmp=*gx; *gx=-*gy; *gy=-tmp; *gz=-*gz;   tmp=*mx; *mx=-*my; *my=-tmp; *mz=-*mz; }while(0)
#else
  #define IMU_ROTATE()
#endif


#include "MPU_interface.h"
#ifdef USE_IMU_BUS_SPI
  MPU_InterfaceSPI mpu_iface(spi, HW_PIN_SPI_CS);
#else
  MPU_InterfaceI2C<HW_WIRETYPE> mpu_iface(i2c, IMU_I2C_ADR);
#endif



//========================================================================================================================
// MPU6000 SPI/I2C or MPU6000 I2C [gyro/acc]
//========================================================================================================================
#if defined USE_IMU_MPU6000 || (defined USE_IMU_BUS_I2C && defined USE_IMU_MPU6050)

#if defined USE_IMU_MPU6000 
  #define IMU_TYPE "MPU6000"
#elif defined USE_IMU_MPU6050
  #define IMU_TYPE "MPU6050"
#endif

#include "MPU60X0.h"

MPU60X0 mpu(&mpu_iface);

int imu_Setup() {
  Serial.println("USE_IMU_" IMU_TYPE " " IMU_BUS_TYPE);
  return mpu.begin(IMU_GYRO_DPS, IMU_ACCEL_G);
}

void imu_Read(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *mx, float *my, float *mz) {
  (void)(mx); (void)(my); (void)(mz); //suppress compiler warnings
  mpu.getMotion6NED(ax, ay, az, gx, gy, gz);
  IMU_ROTATE();
}


//========================================================================================================================
// MPU9150 I2C gyro/acc/mag
//========================================================================================================================
#elif defined USE_IMU_BUS_I2C && defined USE_IMU_MPU9150

#include "MPU9150.h"

MPU9150 mpu(&mpu_iface);

int imu_Setup() {
  Serial.printf("USE_IMU_MPU9150 " IMU_BUS_TYPE);
  int status = mpu.begin();
  mpu.set_gyro_scale_dps(IMU_GYRO_DPS);
  mpu.set_acc_scale_g(IMU_ACCEL_G); 
  return status;
}

void imu_Read(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *mx, float *my, float *mz) {
  mpu.getMotion9NED(ax, ay, az, gx, gy, gz, mx, my, mz);
  IMU_ROTATE();
}

//========================================================================================================================
// MPU6500 SPI/I2C gyro/acc
//========================================================================================================================
#elif defined USE_IMU_MPU6500

#include "MPU9250.h"

MPU9250 mpu(&mpu_iface);

int imu_Setup() {
  Serial.printf("USE_IMU_MPU6500 " IMU_BUS_TYPE);
  int status = mpu.begin();
  mpu.set_gyro_scale_dps(IMU_GYRO_DPS);
  mpu.set_acc_scale_g(IMU_ACCEL_G); 
  if(status == -1112) { //expected return value for MPU6500
    return 0;
  }  
  return status;
}

void imu_Read(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *mx, float *my, float *mz) {
  (void)(mx); (void)(my); (void)(mz); //suppress compiler warnings
  mpu.getMotion6NED(ax, ay, az, gx, gy, gz);
  IMU_ROTATE();
}

//========================================================================================================================
// MPU9250 SPI/I2C gyro/acc/mag
//========================================================================================================================
#elif defined USE_IMU_MPU9250

#include "MPU9250.h"

MPU9250 mpu(&mpu_iface);

int imu_Setup() {
  Serial.printf("USE_IMU_MPU9250 " IMU_BUS_TYPE);
  int status = mpu.begin();
  mpu.set_gyro_scale_dps(IMU_GYRO_DPS);
  mpu.set_acc_scale_g(IMU_ACCEL_G); 
  if(status == -1112) {
    Serial.println("WARNING: MPU9250 is actually MPU6500 without magnetometer");
    return 0;
  }
  return status;
}

void imu_Read(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *mx, float *my, float *mz) {
  mpu.getMotion9NED(ax, ay, az, gx, gy, gz, mx, my, mz);
  IMU_ROTATE();
}

#else
  #error "uncomment a correct combination of USE_IMU_xxx and USE_IMU_BUS_xxx"
#endif
