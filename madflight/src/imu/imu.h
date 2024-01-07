/*========================================================================================================================
This file contains all necessary functions and code used for IMU sensors to avoid cluttering the main code

Each USE_IMU_xxx section in this file defines:
 - SensorType imu_Sensor
 - #define IMU_HAS_MAG 0|1
 - int imu_Setup() - init, return 0 on success, positive on error, negative on warning
 - void imu_Read() - reads acc,gyro,mag sample from sensor and returns: acc (g), gyro (deg/s)
 - and sets imu_rate_hz to the actual sensor data rate

Body frame is NED: x-axis North(front), y-axis East(right), z-axis Down

MPU-6XXX and MPU-9XXX sensor family
===================================
These are 6 or 9 axis sensors, with maximum sample rates: gyro 8 kHz, accel 4 kHz, and mag 100 Hz. The driver 
configures gyro and accel with 1000 Hz sample rate (with on sensor 200 Hz low pass filter), and mag 100 Hz.
========================================================================================================================*/

extern uint32_t loop_freq;// = 1000; //The main loop frequency in Hz. imu.h might lower this depending on the sensor used. Do not touch unless you know what you are doing.

//handle rotation for different mounting positions
#if defined IMU_ROTATE_CW90
  #define IMU_ROTATE() do{ float tmp; tmp=*ax; *ax=-*ay; *ay=tmp;   tmp=*gx; *gx=-*gy; *gy=tmp;   tmp=*mx; *mx=-*my; *my=tmp; }while(0)
#elif defined IMU_ROTATE_CW180
  #define IMU_ROTATE() do{ *ax=-*ax; *ay=-*ay;   *gx=-*gx; *gy=-*gy;   *mx=-*mx; *my=-*my; }while(0)
#elif defined IMU_ROTATE_CW270
  #define IMU_ROTATE() do{ float tmp; tmp=*ax; *ax=*ay; *ay=-tmp;   tmp=*gx; *gx=*gy; *gy=-tmp;   tmp=*mx; *mx=*my; *my=-tmp; }while(0)
#elif defined IMU_ROTATE_CW0FLIP
  #define IMU_ROTATE() do{ *ay=-*ay; *az=-*az;   *gy=-*gy; *gz=-*gz;   *my=-*my; *mz=-*mz; }while(0)
#elif defined IMU_ROTATE_CW90FLIP
  #define IMU_ROTATE() do{ float tmp; tmp=*ax; *ax=*ay; *ay=tmp; *az=-*az;   tmp=*gx; *gx=*gy; *gy=tmp; *gz=-*gz;   tmp=*mx; *mx=*my; *my=tmp; *mz=-*mz; }while(0)
#elif defined IMU_ROTATE_CW180FLIP
  #define IMU_ROTATE() do{ *ax=-*ax; *az=-*az;   *gx=-*gx; *gz=-*gz;   *mx=-*mx; *mz=-*mz; }while(0)
#elif defined IMU_ROTATE_CW270FLIP
  #define IMU_ROTATE() do{ float tmp; tmp=*ax; *ax=-*ay; *ay=-tmp; *az=-*az;   tmp=*gx; *gx=-*gy; *gy=-tmp; *gz=-*gz;   tmp=*mx; *mx=-*my; *my=-tmp; *mz=-*mz; }while(0)
#else
  #define IMU_ROTATE()
#endif

//setup the imu_Sensor object
#if defined USE_IMU_SPI_BMI270
  #define IMU_TYPE "USE_IMU_SPI_BMI270"
  #define USE_IMU_BUS_SPI
  #define IMU_HAS_MAG 0
  #include "BMI270/BMI270.h"
  BMI270 imu_Sensor(spi, HW_PIN_IMU_CS);
#elif defined USE_IMU_SPI_MPU9250
  #define IMU_TYPE "USE_IMU_SPI_MPU9250"
  #define USE_IMU_BUS_SPI
  #define IMU_HAS_MAG 1
  #include "MPUxxxx/MPU_interface.h"
  #include "MPUxxxx/MPUxxxx.h"
  MPU_InterfaceSPI mpu_iface(spi, HW_PIN_IMU_CS);
  MPUXXXX imu_Sensor(MPUXXXX::MPU9250, &mpu_iface);
#elif defined USE_IMU_SPI_MPU6500
  #define IMU_TYPE "USE_IMU_SPI_MPU6500"
  #define USE_IMU_BUS_SPI
  #define IMU_HAS_MAG 0
  #include "MPUxxxx/MPU_interface.h"
  #include "MPUxxxx/MPUxxxx.h"
  MPU_InterfaceSPI mpu_iface(spi, HW_PIN_IMU_CS);
  MPUXXXX imu_Sensor(MPUXXXX::MPU6500, &mpu_iface);
#elif defined USE_IMU_SPI_MPU6000
  #define IMU_TYPE "USE_IMU_SPI_MPU6000"
  #define USE_IMU_BUS_SPI
  #define IMU_HAS_MAG 0
  #include "MPUxxxx/MPU_interface.h"
  #include "MPUxxxx/MPUxxxx.h"
  MPU_InterfaceSPI mpu_iface(spi, HW_PIN_IMU_CS);
  MPUXXXX imu_Sensor(MPUXXXX::MPU6000, &mpu_iface);
#elif defined USE_IMU_I2C_MPU9250
  #define IMU_TYPE "USE_IMU_I2C_MPU9250"
  #define USE_IMU_BUS_I2C
  #define IMU_HAS_MAG 1
  #include "MPUxxxx/MPU_interface.h"
  #include "MPUxxxx/MPUxxxx.h"
  MPU_InterfaceI2C<HW_WIRETYPE> mpu_iface(i2c, IMU_I2C_ADR);
  MPUXXXX imu_Sensor(MPU9250, &mpu_iface);
#elif defined USE_IMU_I2C_MPU9150
  #define IMU_TYPE "USE_IMU_I2C_MPU9150"
  #define USE_IMU_BUS_I2C
  #define IMU_HAS_MAG 1
  #include "MPUxxxx/MPU_interface.h"
  #include "MPUxxxx/MPUxxxx.h"
  MPU_InterfaceI2C<HW_WIRETYPE> mpu_iface(i2c, IMU_I2C_ADR);
  MPUXXXX imu_Sensor(MPUXXXX::MPU9150, &mpu_iface);
#elif defined USE_IMU_I2C_MPU6500
  #define IMU_TYPE "USE_IMU_I2C_MPU6500"
  #define USE_IMU_BUS_I2C
  #define IMU_HAS_MAG 0
  #include "MPUxxxx/MPU_interface.h"
  #include "MPUxxxx/MPUxxxx.h"
  MPU_InterfaceI2C<HW_WIRETYPE> mpu_iface(i2c, IMU_I2C_ADR);
  MPUXXXX imu_Sensor(MPUXXXX::MPU6500, &mpu_iface);
#elif defined USE_IMU_I2C_MPU6050
  #define IMU_TYPE "USE_IMU_I2C_MPU6050"
  #define USE_IMU_BUS_I2C
  #define IMU_HAS_MAG 0
  #include "MPUxxxx/MPU_interface.h"
  #include "MPUxxxx/MPUxxxx.h"
  MPU_InterfaceI2C<HW_WIRETYPE> mpu_iface(i2c, IMU_I2C_ADR);
  MPUXXXX imu_Sensor(MPUXXXX::MPU6050, &mpu_iface);
#elif defined USE_IMU_I2C_MPU6000
  #define IMU_TYPE "USE_IMU_I2C_MPU6000"
  #define USE_IMU_BUS_I2C
  #define IMU_HAS_MAG 0
  MPU_InterfaceI2C<HW_WIRETYPE> mpu_iface(i2c, IMU_I2C_ADR);
  MPUXXXX imu_Sensor(MPUXXXX::MPU6000, &mpu_iface);
#else
  #error "uncomment a one USE_IMU_xxx in madflight.ino"
#endif

//returns 0 on success, positive on error, negative on warning
int imu_Setup() {
  int rv = imu_Sensor.begin(IMU_GYRO_DPS, IMU_ACCEL_G, loop_freq);
  loop_freq = imu_Sensor.get_rate();
  Serial.printf(IMU_TYPE " loop_freq=%dHz rv=%d\n", loop_freq, rv);
  return rv;
}

void imu_Read(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *mx, float *my, float *mz) {
#if IMU_HAS_MAG 
  imu_Sensor.getMotion9NED(ax, ay, az, gx, gy, gz, mx, my, mz);
#else
  (void)(mx); (void)(my); (void)(mz); //suppress compiler warnings
  imu_Sensor.getMotion6NED(ax, ay, az, gx, gy, gz);
#endif
  IMU_ROTATE();
}
