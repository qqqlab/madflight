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

#pragma once

#include "imu_interface.h" //Imu class declaration

//Available sensors
#define IMU_USE_NONE 0
#define IMU_USE_SPI_BMI270 1
#define IMU_USE_SPI_MPU9250 2
#define IMU_USE_SPI_MPU6500 3
#define IMU_USE_SPI_MPU6000 4
#define IMU_USE_I2C_MPU9250 5
#define IMU_USE_I2C_MPU9150 6
#define IMU_USE_I2C_MPU6500 7
#define IMU_USE_I2C_MPU6050 8
#define IMU_USE_I2C_MPU6000 9

//Available aligns
#define IMU_ALIGN_CW0 1
#define IMU_ALIGN_CW90 2
#define IMU_ALIGN_CW180 3
#define IMU_ALIGN_CW270 4
#define IMU_ALIGN_CW0FLIP 5
#define IMU_ALIGN_CW90FLIP 6
#define IMU_ALIGN_CW180FLIP 7
#define IMU_ALIGN_CW270FLIP 8

//Available excecution methods (not all platforms support all methods)
#define IMU_EXEC_IRQ 1            //execute in IRQ context on first core (works on STM32, RP2350. Does NOT work on ESP32)
#define IMU_EXEC_FREERTOS 2       //execute as IRQ triggered high priority FreeRTOS task on same core as setup() (works on ESP32, RP2040)
#define IMU_EXEC_FREERTOS_OTHERCORE 3 //execute as IRQ triggered high priority FreeRTOS task on second core (works on RP2040)

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

//depreciated
#ifdef HW_USE_FREERTOS
  #error "HW_USE_FREERTOS is depreciated, use IMU_EXEC_XXX"
#endif


#ifndef IMU_USE 
  #define IMU_USE IMU_USE_NONE
#endif

//=====================================================================================================================
// setup the imu_Sensor object
//=====================================================================================================================
#if IMU_USE == IMU_USE_NONE
  //do nothing

#elif IMU_USE == IMU_USE_SPI_BMI270
  #define IMU_TYPE "IMU_USE_SPI_BMI270"
  #define IMU_IS_I2C 0
  #define IMU_HAS_MAG 0
  #include "BMI270/BMI270.h"
  BMI270 imu_Sensor(spi, HW_PIN_IMU_CS);

#elif IMU_USE == IMU_USE_SPI_MPU9250
  #define IMU_TYPE "IMU_USE_SPI_MPU9250"
  #define IMU_IS_I2C 0
  #define IMU_HAS_MAG 1
  #include "MPUxxxx/MPU_interface.h"
  #include "MPUxxxx/MPUxxxx.h"
  MPU_InterfaceSPI mpu_iface(spi, HW_PIN_IMU_CS);
  MPUXXXX imu_Sensor(MPUXXXX::MPU9250, &mpu_iface);

#elif IMU_USE == IMU_USE_SPI_MPU6500
  #define IMU_TYPE "IMU_USE_SPI_MPU6500"
  #define IMU_IS_I2C 0
  #define IMU_HAS_MAG 0
  #include "MPUxxxx/MPU_interface.h"
  #include "MPUxxxx/MPUxxxx.h"
  MPU_InterfaceSPI mpu_iface(spi, HW_PIN_IMU_CS);
  MPUXXXX imu_Sensor(MPUXXXX::MPU6500, &mpu_iface);

#elif IMU_USE == IMU_USE_SPI_MPU6000
  #define IMU_TYPE "IMU_USE_SPI_MPU6000"
  #define IMU_IS_I2C 0
  #define IMU_HAS_MAG 0
  #include "MPUxxxx/MPU_interface.h"
  #include "MPUxxxx/MPUxxxx.h"
  MPU_InterfaceSPI mpu_iface(spi, HW_PIN_IMU_CS);
  MPUXXXX imu_Sensor(MPUXXXX::MPU6000, &mpu_iface);

#elif IMU_USE == IMU_USE_I2C_MPU9250
  #define IMU_TYPE "IMU_USE_I2C_MPU9250"
  #define IMU_IS_I2C 1
  #define IMU_HAS_MAG 1
  #include "MPUxxxx/MPU_interface.h"
  #include "MPUxxxx/MPUxxxx.h"
  MPU_InterfaceI2C<HW_WIRETYPE> mpu_iface(i2c, IMU_I2C_ADR);
  MPUXXXX imu_Sensor(MPUXXXX::MPU9250, &mpu_iface);

#elif IMU_USE == IMU_USE_I2C_MPU9150
  #define IMU_TYPE "IMU_USE_I2C_MPU9150"
  #define IMU_IS_I2C 1
  #define IMU_HAS_MAG 1
  #include "MPUxxxx/MPU_interface.h"
  #include "MPUxxxx/MPUxxxx.h"
  MPU_InterfaceI2C<HW_WIRETYPE> mpu_iface(i2c, IMU_I2C_ADR);
  MPUXXXX imu_Sensor(MPUXXXX::MPU9150, &mpu_iface);

#elif IMU_USE == IMU_USE_I2C_MPU6500
  #define IMU_TYPE "IMU_USE_I2C_MPU6500"
  #define IMU_IS_I2C 1
  #define IMU_HAS_MAG 0
  #include "MPUxxxx/MPU_interface.h"
  #include "MPUxxxx/MPUxxxx.h"
  MPU_InterfaceI2C<HW_WIRETYPE> mpu_iface(i2c, IMU_I2C_ADR);
  MPUXXXX imu_Sensor(MPUXXXX::MPU6500, &mpu_iface);

#elif IMU_USE == IMU_USE_I2C_MPU6050
  #define IMU_TYPE "IMU_USE_I2C_MPU6050"
  #define IMU_IS_I2C 1
  #define IMU_HAS_MAG 0
  #include "MPUxxxx/MPU_interface.h"
  #include "MPUxxxx/MPUxxxx.h"
  MPU_InterfaceI2C<HW_WIRETYPE> mpu_iface(i2c, IMU_I2C_ADR);
  MPUXXXX imu_Sensor(MPUXXXX::MPU6050, &mpu_iface);

#elif IMU_USE == IMU_USE_I2C_MPU6000
  #define IMU_TYPE "IMU_USE_I2C_MPU6000"
  #define IMU_IS_I2C 1
  #define IMU_HAS_MAG 0
  MPU_InterfaceI2C<HW_WIRETYPE> mpu_iface(i2c, IMU_I2C_ADR);
  MPUXXXX imu_Sensor(MPUXXXX::MPU6000, &mpu_iface);

// Invalid value
#else
  #error "invalid IMU_USE value"
#endif



//========================================================================================================================//
// Imu Class Implementation
//========================================================================================================================//



//global Imu class instance
Imu imu;

#if IMU_USE == IMU_USE_NONE
  //dummy class define when no imu is present
  int Imu::setup(uint32_t sampleRate) {return 0;}
  bool Imu::waitNewSample() {return false;}
  bool Imu::hasMag() {return false;}
  bool Imu::usesI2C() {return false;}
  void Imu::statReset() {}
  void Imu::_interrupt_handler() {}
#else //#if IMU_USE == IMU_USE_NONE

void _imu_ll_interrupt_setup(); //prototype
volatile bool _imu_ll_interrupt_enabled = false;
volatile bool _imu_ll_interrupt_busy = false;
volatile uint32_t _imu_ll_interrupt_ts = 0;

bool Imu::usesI2C() { return IMU_IS_I2C; } //returns true if IMU uses I2C bus (not SPI bus)
bool Imu::hasMag() { return IMU_HAS_MAG; }
 
//returns 0 on success, positive on error, negative on warning
int Imu::setup(uint32_t sampleRate) {
  _imu_ll_interrupt_enabled = false;
  int rv = imu_Sensor.begin(IMU_GYRO_DPS, IMU_ACCEL_G, sampleRate);
  _sampleRate = imu_Sensor.get_rate();
  Serial.printf("IMU:  " IMU_TYPE " sample_rate=%dHz rv=%d\n", (int)_sampleRate, (int)rv);
  onUpdate = NULL;
  _imu_ll_interrupt_busy = false;
  _imu_ll_interrupt_ts = 0;
  overrun_cnt = 0;
  dt = 0;
  ts = micros();
  statReset();
  _imu_ll_interrupt_setup();
  _imu_ll_interrupt_enabled = true;
  interrupt_cnt = 0;
  update_cnt = 0;
  return rv;
}

//wait for new sample, returns false on fail
bool Imu::waitNewSample() {
  uint32_t last_cnt = update_cnt;
  uint32_t start = millis();
  while( last_cnt == update_cnt && millis() - start <= (10*1000) / _sampleRate );
  return (last_cnt != update_cnt);
}

void Imu::statReset() {
    stat_cnt = 0; //number of accumulated samples
    stat_latency = 0; //summed interrupt latency from start of interrupt handler to start of interrupt task in us
    stat_io_runtime = 0; //summed runtime of SPI/I2C io transfer in us
    stat_runtime = 0; //summed runtime imu update including io transfer in us
    stat_runtime_max = 0; //max runtime imu update including transfer in us, since last reset to 0
    stat_reset_ts = micros(); //last time statReset() was called
}


//this function executes whenever the imu task is triggered
void Imu::_interrupt_handler() {
  //local copy of timestamp (_imu_ll_interrupt_ts might change during execution of this function)
  uint32_t interrupt_ts = _imu_ll_interrupt_ts;
  
  //start of task timestamp
  uint32_t task_ts = micros();
  
  //latency between start of low level interrupt handler and start of this method
  stat_latency += task_ts - _imu_ll_interrupt_ts;

  //get sensor data and update timestamps, count
  #if IMU_HAS_MAG 
    imu_Sensor.getMotion9NED(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  #else
    imu_Sensor.getMotion6NED(&ax, &ay, &az, &gx, &gy, &gz);
  #endif
  IMU_ROTATE();
  dt = (interrupt_ts - ts) / 1000000.0;
  ts = interrupt_ts;
  stat_io_runtime += micros() - task_ts; //runtime of SPI/I2C transfer
  update_cnt++;

  //call external update event handler
  if(onUpdate) onUpdate();

  uint32_t rt = micros() - task_ts; //runtime of full update
  stat_runtime += rt;
  if(stat_runtime_max < rt) stat_runtime_max = rt; //max runtime
  stat_cnt++;
}

//========================================================================================================================//
// _IMU_LL_ IMU Low Level Interrrupt Handler
//========================================================================================================================//
// This runs the IMU updates triggered from pin HW_PIN_IMU_EXTI interrupt. When using FreeRTOS with IMU_EXEC_FREERTOS the 
// IMU update is not executed directly in the interrupt handler, but a high priority task is used. This prevents FreeRTOS 
// watchdog resets. The delay (latency) from rising edge INT pin to handler is approx. 10 us on ESP32 and 50 us on RP2040.

void _imu_ll_interrupt_handler();

//-------------------------------------------------------------------------------------------------------------------------
#if IMU_EXEC == IMU_EXEC_FREERTOS || IMU_EXEC == IMU_EXEC_FREERTOS_OTHERCORE

  TaskHandle_t _imu_ll_task_handle = NULL;

  void _imu_ll_task(void*) {
    for(;;) {
      ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
      imu._interrupt_handler();
    }
  }

  void _imu_ll_interrupt_setup() {
    if(!_imu_ll_task_handle) {
      //xTaskCreatePinnedToCore(_imu_ll_task, "_imu_ll_task", 4096, NULL, IMU_FREERTOS_TASK_PRIORITY /*priority 0=lowest*/, &_imu_ll_task_handle, othercore); //ESP32 only
      xTaskCreate(_imu_ll_task, "IMU", FREERTOS_DEFAULT_STACK_SIZE, NULL, IMU_FREERTOS_TASK_PRIORITY /*priority 0=lowest*/, &_imu_ll_task_handle);
      #if IMU_EXEC == IMU_EXEC_FREERTOS_OTHERCORE
        int callcore = hw_get_core_num();
        int othercore = (callcore+1)%2;
        vTaskCoreAffinitySet(_imu_ll_task_handle, (1<<othercore)); //Sets the core affinity mask for a task, i.e. the cores on which a task can run.
        Serial.printf("IMU:  IMU_EXEC_FREERTOS_OTHERCORE call_core=%d imu_core=%d\n", callcore, othercore);
      #else
        Serial.println("IMU:  IMU_EXEC_FREERTOS");
      #endif
    }
    attachInterrupt(digitalPinToInterrupt(HW_PIN_IMU_EXTI), _imu_ll_interrupt_handler, RISING); 
  }
  
  inline void _imu_ll_interrupt_handler2() {
    //let RTOS task _imu_ll_task handle the interrupt
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(_imu_ll_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
//-------------------------------------------------------------------------------------------------------------------------
#elif IMU_EXEC == IMU_EXEC_IRQ
  void _imu_ll_interrupt_setup() {
    Serial.println("IMU:  IMU_EXEC_IRQ");
    attachInterrupt(digitalPinToInterrupt(HW_PIN_IMU_EXTI), _imu_ll_interrupt_handler, RISING);
  }

  inline void _imu_ll_interrupt_handler2() {
      //call interrupt handler directly from interrupt context
      imu._interrupt_handler();
  }
//-------------------------------------------------------------------------------------------------------------------------
#else
  #error #define IMU_EXEC is needed, see imu.h for available options
#endif

//main IRQ handler - timestamp + busy wrapper around _imu_ll_interrupt_handler2()
void _imu_ll_interrupt_handler() {
  _imu_ll_interrupt_ts = micros();
  if(_imu_ll_interrupt_enabled) {
    imu.interrupt_cnt = imu.interrupt_cnt + 1; //NOTE: "imu.interrupt_cnt++" gives warning: '++' expression of 'volatile'-qualified type is deprecated
    if (_imu_ll_interrupt_busy) { //note: time difference between check/update of _imu_ll_interrupt_busy can cause a race condition...
      imu.overrun_cnt++;
    } else {
      _imu_ll_interrupt_busy = true;
      _imu_ll_interrupt_handler2();
      _imu_ll_interrupt_busy = false;
    }
  }
}

#endif //#if IMU_USE == IMU_USE_NONE
