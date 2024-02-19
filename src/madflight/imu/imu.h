/*========================================================================================================================
This file contains all necessary functions and code used for IMU sensors to avoid cluttering the main code

Each IMU_USE_xxx section in this file defines:
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

//Available sensors
//#define IMU_USE_NONE 0 //always need an IMU
#define IMU_USE_SPI_BMI270 1
#define IMU_USE_SPI_MPU9250 2
#define IMU_USE_SPI_MPU6500 3
#define IMU_USE_SPI_MPU6000 4
#define IMU_USE_I2C_MPU9250 5
#define IMU_USE_I2C_MPU9150 6
#define IMU_USE_I2C_MPU6500 7
#define IMU_USE_I2C_MPU6050 8
#define IMU_USE_I2C_MPU6000 9

#include "../interface.h"

/* INTERFACE
class Imu {
  public:
    float ax = 0; //"North" acceleration in G
    float ay = 0; //"East" acceleration in G
    float az = 0; //"Down" acceleration in G
    float gx = 0; //"North" rotation speed in deg/s
    float gy = 0; //"East" rotation speed in deg/s
    float gz = 0; //"Down" rotation speed in deg/s
    float mx = 0; //"North" magnetic flux in uT
    float my = 0; //"East" magnetic flux in uT
    float mz = 0; //"Down" magnetic flux in uT
    virtual bool hasMag() = 0; //returns true if IMU has a magnetometer
    virtual int setup(uint32_t sampleRate) = 0;
    virtual void update() = 0;
    uint32_t getSampleRate() {return _sampleRate;}
  protected:
    uint32_t _sampleRate = 0; //sensor sample rate in Hz
};

extern Imu &imu;
*/

//Available aligns
#define IMU_ALIGN_CW0 1
#define IMU_ALIGN_CW90 2
#define IMU_ALIGN_CW180 3
#define IMU_ALIGN_CW270 4
#define IMU_ALIGN_CW0FLIP 5
#define IMU_ALIGN_CW90FLIP 6
#define IMU_ALIGN_CW180FLIP 7
#define IMU_ALIGN_CW270FLIP 8

//default settings
#ifndef IMU_GYRO_DPS
  #define IMU_GYRO_DPS 2000 //Full scale gyro range in deg/sec. Most IMUs support 250,500,1000,2000. Can use any value here, driver will pick next greater setting.
#endif
#ifndef IMU_ACCEL_G
  #define IMU_ACCEL_G 16 //Full scale gyro accelerometer in G's. Most IMUs support 2,4,8,16. Can use any value here, driver will pick next greater setting.
#endif

//handle rotation for different mounting positions
#if IMU_ALIGN == IMU_ALIGN_CW90
  #define IMU_ROTATE() do{ float tmp; tmp=ax; ax=-ay; ay=tmp;   tmp=gx; gx=-gy; gy=tmp;   tmp=mx; mx=-my; my=tmp; }while(0)
#elif IMU_ALIGN == IMU_ALIGN_CW180
  #define IMU_ROTATE() do{ ax=-ax; ay=-ay;   gx=-gx; gy=-gy;   mx=-mx; my=-my; }while(0)
#elif IMU_ALIGN == IMU_ALIGN_CW270
  #define IMU_ROTATE() do{ float tmp; tmp=ax; ax=ay; ay=-tmp;   tmp=gx; gx=gy; gy=-tmp;   tmp=mx; mx=my; my=-tmp; }while(0)
#elif IMU_ALIGN == IMU_ALIGN_CW0FLIP
  #define IMU_ROTATE() do{ ay=-ay; az=-az;   gy=-gy; gz=-gz;   my=-my; mz=-mz; }while(0)
#elif IMU_ALIGN == IMU_ALIGN_CW90FLIP
  #define IMU_ROTATE() do{ float tmp; tmp=ax; ax=ay; ay=tmp; az=-az;   tmp=gx; gx=gy; gy=tmp; gz=-gz;   tmp=mx; mx=my; my=tmp; mz=-mz; }while(0)
#elif IMU_ALIGN == IMU_ALIGN_CW180FLIP
  #define IMU_ROTATE() do{ ax=-ax; az=-az;   gx=-gx; gz=-gz;   mx=-mx; mz=-mz; }while(0)
#elif IMU_ALIGN == IMU_ALIGN_CW270FLIP
  #define IMU_ROTATE() do{ float tmp; tmp=ax; ax=-ay; ay=-tmp; az=-az;   tmp=gx; gx=-gy; gy=-tmp; gz=-gz;   tmp=mx; mx=-my; my=-tmp; mz=-mz; }while(0)
#else
  #define IMU_ROTATE()
#endif


//setup the imu_Sensor object
//=====================================================================================================================
// IMU_USE_SPI_BMI270
//=====================================================================================================================
#if IMU_USE == IMU_USE_SPI_BMI270
  #define IMU_TYPE "IMU_USE_SPI_BMI270"
  #define IMU_USE_BUS_SPI
  #define IMU_HAS_MAG 0
  #include "BMI270/BMI270.h"
  BMI270 imu_Sensor(spi, HW_PIN_IMU_CS);

//=====================================================================================================================
// IMU_USE_SPI_MPU9250
//=====================================================================================================================
#elif IMU_USE == IMU_USE_SPI_MPU9250
  #define IMU_TYPE "IMU_USE_SPI_MPU9250"
  #define IMU_USE_BUS_SPI
  #define IMU_HAS_MAG 1
  #include "MPUxxxx/MPU_interface.h"
  #include "MPUxxxx/MPUxxxx.h"
  MPU_InterfaceSPI mpu_iface(spi, HW_PIN_IMU_CS);
  MPUXXXX imu_Sensor(MPUXXXX::MPU9250, &mpu_iface);

//=====================================================================================================================
// IMU_USE_SPI_MPU6500
//=====================================================================================================================
#elif IMU_USE == IMU_USE_SPI_MPU6500
  #define IMU_TYPE "IMU_USE_SPI_MPU6500"
  #define IMU_USE_BUS_SPI
  #define IMU_HAS_MAG 0
  #include "MPUxxxx/MPU_interface.h"
  #include "MPUxxxx/MPUxxxx.h"
  MPU_InterfaceSPI mpu_iface(spi, HW_PIN_IMU_CS);
  MPUXXXX imu_Sensor(MPUXXXX::MPU6500, &mpu_iface);

//=====================================================================================================================
// IMU_USE_SPI_MPU6000
//=====================================================================================================================
#elif IMU_USE == IMU_USE_SPI_MPU6000
  #define IMU_TYPE "IMU_USE_SPI_MPU6000"
  #define IMU_USE_BUS_SPI
  #define IMU_HAS_MAG 0
  #include "MPUxxxx/MPU_interface.h"
  #include "MPUxxxx/MPUxxxx.h"
  MPU_InterfaceSPI mpu_iface(spi, HW_PIN_IMU_CS);
  MPUXXXX imu_Sensor(MPUXXXX::MPU6000, &mpu_iface);

//=====================================================================================================================
// IMU_USE_I2C_MPU9250
//=====================================================================================================================
#elif IMU_USE == IMU_USE_I2C_MPU9250
  #define IMU_TYPE "IMU_USE_I2C_MPU9250"
  #define IMU_USE_BUS_I2C
  #define IMU_HAS_MAG 1
  #include "MPUxxxx/MPU_interface.h"
  #include "MPUxxxx/MPUxxxx.h"
  MPU_InterfaceI2C<HW_WIRETYPE> mpu_iface(i2c, IMU_I2C_ADR);
  MPUXXXX imu_Sensor(MPU9250, &mpu_iface);

//=====================================================================================================================
// IMU_USE_I2C_MPU9150
//=====================================================================================================================
#elif IMU_USE == IMU_USE_I2C_MPU9150
  #define IMU_TYPE "IMU_USE_I2C_MPU9150"
  #define IMU_USE_BUS_I2C
  #define IMU_HAS_MAG 1
  #include "MPUxxxx/MPU_interface.h"
  #include "MPUxxxx/MPUxxxx.h"
  MPU_InterfaceI2C<HW_WIRETYPE> mpu_iface(i2c, IMU_I2C_ADR);
  MPUXXXX imu_Sensor(MPUXXXX::MPU9150, &mpu_iface);

//=====================================================================================================================
// IMU_USE_I2C_MPU6500
//=====================================================================================================================
#elif IMU_USE == IMU_USE_I2C_MPU6500
  #define IMU_TYPE "IMU_USE_I2C_MPU6500"
  #define IMU_USE_BUS_I2C
  #define IMU_HAS_MAG 0
  #include "MPUxxxx/MPU_interface.h"
  #include "MPUxxxx/MPUxxxx.h"
  MPU_InterfaceI2C<HW_WIRETYPE> mpu_iface(i2c, IMU_I2C_ADR);
  MPUXXXX imu_Sensor(MPUXXXX::MPU6500, &mpu_iface);

//=====================================================================================================================
// IMU_USE_I2C_MPU6050
//=====================================================================================================================
#elif IMU_USE == IMU_USE_I2C_MPU6050
  #define IMU_TYPE "IMU_USE_I2C_MPU6050"
  #define IMU_USE_BUS_I2C
  #define IMU_HAS_MAG 0
  #include "MPUxxxx/MPU_interface.h"
  #include "MPUxxxx/MPUxxxx.h"
  MPU_InterfaceI2C<HW_WIRETYPE> mpu_iface(i2c, IMU_I2C_ADR);
  MPUXXXX imu_Sensor(MPUXXXX::MPU6050, &mpu_iface);

//=====================================================================================================================
// IMU_USE_I2C_MPU6000
//=====================================================================================================================
#elif IMU_USE == IMU_USE_I2C_MPU6000
  #define IMU_TYPE "IMU_USE_I2C_MPU6000"
  #define IMU_USE_BUS_I2C
  #define IMU_HAS_MAG 0
  MPU_InterfaceI2C<HW_WIRETYPE> mpu_iface(i2c, IMU_I2C_ADR);
  MPUXXXX imu_Sensor(MPUXXXX::MPU6000, &mpu_iface);
  
//=====================================================================================================================
// None or undefined
//=====================================================================================================================
#elif IMU_USE == IMU_USE_NONE || !defined IMU_USE
  #error "IMU_USE not defined"
  
//=====================================================================================================================
// Invalid value
//=====================================================================================================================
#else
  #error "invalid IMU_USE value"
#endif

class ImuGeneric: public Imu {
  public:
    bool hasMag() { return IMU_HAS_MAG; }
  
    //returns 0 on success, positive on error, negative on warning
    int setup(uint32_t sampleRate) {
      int rv = imu_Sensor.begin(IMU_GYRO_DPS, IMU_ACCEL_G, sampleRate);
      _sampleRate = imu_Sensor.get_rate();
      Serial.printf(IMU_TYPE " sample_rate=%dHz rv=%d\n", (int)_sampleRate, (int)rv);
      return rv;
    }

    void update() {
    #if IMU_HAS_MAG 
      imu_Sensor.getMotion9NED(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    #else
      imu_Sensor.getMotion6NED(&ax, &ay, &az, &gx, &gy, &gz);
    #endif
      IMU_ROTATE();
    }
};

ImuGeneric imu_instance;

Imu &imu = imu_instance;