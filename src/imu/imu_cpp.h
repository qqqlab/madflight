/*==========================================================================================
MIT License

Copyright (c) 2023-2025 https://madflight.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
===========================================================================================*/

/*========================================================================================================================
This file is kept as header to allow #IMU_xxx defines to be added in user code (Arduino IDE has no easy way to add -D compiler options)

Body frame is NED:
  x-axis North(front) - gyro-x roll right is positive - accelerometer-x vehicle right side down position is +1 G
  y-axis East(right)  - gyro-y pitch up is positive   - accelerometer-y vehicle nose down postion is +1 G
  z-axis Down         - gyro-z yaw right is positive  - accelerometer-z vehicle level position is +1 G

MPU-6XXX and MPU-9XXX sensor family
===================================
These are 6 or 9 axis sensors, with maximum sample rates: gyro 8 kHz, accel 4 kHz, and mag 100 Hz. The driver
configures gyro and accel with 1000 Hz sample rate (with on sensor 200 Hz low pass filter), and mag 100 Hz.

===================================
ICM-4xxxx sensors
===================================
Currently only ICM45686 is supported.
This is a 6 axis sensor, with maximum sample rates of 6.4khz, max gyro range 4000dps, max accelerometer range 32G
Limitations:
- The underlying driver lib supports only one sensor instance
- only via SPI + interupt; I2C can be added using the same driver lib
========================================================================================================================*/

// Make sure this file is included from madflight.h and not from somewhere else
#ifndef MF_ALLOW_INCLUDE_CCP_H
  #error "Only include this file from madflight.h"
#endif
//#pragma once //don't use here, we want to get an error if included twice

#define MF_MOD "IMU"

//Available excecution methods (not all platforms support all methods)
#define IMU_EXEC_IRQ 1            //execute in IRQ context on first core (works on STM32. Does NOT work on ESP32, RP2040)
#define IMU_EXEC_FREERTOS 2       //execute as IRQ triggered high priority FreeRTOS task on same core as setup() (works on ESP32, RP2040)
#define IMU_EXEC_FREERTOS_OTHERCORE 3 //execute as IRQ triggered high priority FreeRTOS task on second core (works on RP2040)

//default settings
#ifndef IMU_GYRO_DPS
  #define IMU_GYRO_DPS 2000 //Full scale gyro range in deg/sec. Most IMUs support 250,500,1000,2000. Can use any value here, driver will pick next greater setting.
#endif
#ifndef IMU_ACCEL_G
  #define IMU_ACCEL_G 16 //Full scale accelerometer range in G's. Most IMUs support 2,4,8,16. Can use any value here, driver will pick next greater setting.
#endif


#include "./imu.h"
#include "../cfg/cfg.h"

//the "gizmos"
#include "./MPUxxxx/MPU_interface.h"
#include "./MPUxxxx/MPUxxxx.h"
#include "./ImuGizmoBMI270.h"
#include "./ImuGizmoICM45686.h"
#include "./ImuGizmoICM426XX.h"
#include "./ImuGizmoICM20948.h"

//global module class instance
Imu imu;

void _imu_ll_interrupt_setup(int interrupt_pin, bool rising_edge); //prototype
volatile bool _imu_ll_interrupt_enabled = false;
volatile bool _imu_ll_interrupt_busy = false;
volatile uint32_t _imu_ll_interrupt_ts = 0;

bool Imu::usesI2C() { return gizmo->uses_i2c; } //returns true if IMU uses I2C bus (not SPI bus)
bool Imu::hasMag() { return gizmo->has_mag; }
bool Imu::hasSensorFusion() { return gizmo->has_sensor_fusion; }

//returns 0 on success, positive on error, negative on warning
int Imu::setup() {
  cfg.printModule(MF_MOD);

  //disable interrupt handler
  _imu_ll_interrupt_enabled = false;

  delete gizmo;

  //exit if gizmo == NONE
  if(config.gizmo == Cfg::imu_gizmo_enum::mf_NONE) return 0;

  //==============================================================
  //create gizmo
  //==============================================================
  if(!config.uses_i2c && config.spi_bus && config.spi_cs >= 0) {
    //-----------
    //SPI Sensors
    //-----------
    switch(config.gizmo) {
      case Cfg::imu_gizmo_enum::mf_NONE : {
        //do nothing
        break;
      }
      case Cfg::imu_gizmo_enum::mf_MPU9250 : {
        auto mpu_iface = new MPU_InterfaceSPI(config.spi_bus, config.spi_cs);
        gizmo = new MPUXXXX(MPUXXXX::MPU9250, mpu_iface);
        gizmo->uses_i2c = false;
        gizmo->has_mag = true;
        break;
      }
      case Cfg::imu_gizmo_enum::mf_MPU6500 : {
        auto mpu_iface = new MPU_InterfaceSPI(config.spi_bus, config.spi_cs);
        gizmo = new MPUXXXX(MPUXXXX::MPU6500, mpu_iface);
        gizmo->uses_i2c = false;
        gizmo->has_mag = false;
        break;
      }
      case Cfg::imu_gizmo_enum::mf_MPU6000 : {
        auto mpu_iface = new MPU_InterfaceSPI(config.spi_bus, config.spi_cs);
        gizmo = new MPUXXXX(MPUXXXX::MPU6000, mpu_iface);
        gizmo->uses_i2c = false;
        gizmo->has_mag = false;
        break;
      }
      case Cfg::imu_gizmo_enum::mf_BMI270 : {
        gizmo = new ImuGizmoBMI270(config.spi_bus, config.spi_cs);
        gizmo->uses_i2c = false;
        gizmo->has_mag = false;
        break;
      }
      case Cfg::imu_gizmo_enum::mf_ICM45686 : {
        auto icm_iface = new Invensensev3_InterfaceSPI(config.spi_bus, config.spi_cs);
        gizmo = new ImuGizmoICM45686( (uint8_t)config.pin_int, icm_iface );
        gizmo->uses_i2c = false;
        gizmo->has_mag = false;
        break;
      }
      case Cfg::imu_gizmo_enum::mf_ICM42688 : {
        gizmo = ImuGizmoICM426XX::create(&config, (ImuState*)this);
        break;
      }
      case Cfg::imu_gizmo_enum::mf_ICM20948 : {
        gizmo = new ImuGizmoICM20948(config.spi_bus, config.spi_cs, config.pin_int, config.use_mag);
        break;
      }
      default: {
        Serial.println("\n" MF_MOD ": ERROR - Sensor does not support imu_bus_type=SPI\n");
      }
    }
  }else if(config.uses_i2c && config.i2c_bus) {
    //-----------
    //I2C Sensors
    //-----------
    switch(config.gizmo) {
      case Cfg::imu_gizmo_enum::mf_NONE : {
        //do nothing
        break;
      }
      case Cfg::imu_gizmo_enum::mf_MPU9250 : {
        auto mpu_iface = new MPU_InterfaceI2C(config.i2c_bus, config.i2c_adr);
        gizmo = new MPUXXXX(MPUXXXX::MPU9250, mpu_iface);
        gizmo->uses_i2c = true;
        gizmo->has_mag = true;
        break;
      }
      case Cfg::imu_gizmo_enum::mf_MPU9150 : {
        auto mpu_iface = new MPU_InterfaceI2C(config.i2c_bus, config.i2c_adr);
        gizmo = new MPUXXXX(MPUXXXX::MPU9150, mpu_iface);
        gizmo->uses_i2c = true;
        gizmo->has_mag = true;
        break;
      }
      case Cfg::imu_gizmo_enum::mf_MPU6500 : {
        auto mpu_iface = new MPU_InterfaceI2C(config.i2c_bus, config.i2c_adr);
        gizmo = new MPUXXXX(MPUXXXX::MPU6500, mpu_iface);
        gizmo->uses_i2c = true;
        gizmo->has_mag = false;
        break;
      }
      case Cfg::imu_gizmo_enum::mf_MPU6050 : {
        auto mpu_iface = new MPU_InterfaceI2C(config.i2c_bus, config.i2c_adr);
        gizmo = new MPUXXXX(MPUXXXX::MPU6050, mpu_iface);
        gizmo->uses_i2c = true;
        gizmo->has_mag = false;
        break;
      }
      case Cfg::imu_gizmo_enum::mf_MPU6000 : {
        auto mpu_iface = new MPU_InterfaceI2C(config.i2c_bus, config.i2c_adr);
        gizmo = new MPUXXXX(MPUXXXX::MPU6000, mpu_iface);
        gizmo->uses_i2c = true;
        gizmo->has_mag = false;
        break;
      }
      case Cfg::imu_gizmo_enum::mf_ICM42688 : {
        gizmo = ImuGizmoICM426XX::create(&config, (ImuState*)this);
        break;
      }
      default: {
        Serial.println("\n" MF_MOD ": ERROR - Sensor does not support imu_bus_type=I2C\n");
      }
    }
  }

  //check gizmo
  if(!gizmo && config.gizmo != Cfg::imu_gizmo_enum::mf_NONE) {
    Serial.println("\n" MF_MOD ": ERROR - Check pin/bus/bus_type config\n");
    return -1001;
  }

  //exit if no gizmo installed
  if(!gizmo) return 0;

  //exit if no interrupt pin specified
  if(config.pin_int < 0) return 0;

  return 0;
}

int Imu::setup_interrupt() {
  //==============================================================
  //setup low-level interrupt stuff
  //==============================================================
  //_imu_ll_interrupt_enabled = false; //already done
  int rv = gizmo->begin(IMU_GYRO_DPS, IMU_ACCEL_G, config.sampleRate);
  _sampleRate = gizmo->get_rate();
  Serial.printf(MF_MOD ": Actual sample rate:%d Hz\n", (int)_sampleRate);
  onUpdate = NULL;
  _imu_ll_interrupt_busy = false;
  _imu_ll_interrupt_ts = 0;
  overrun_cnt = 0;
  dt = 0;
  ts = micros();
  statReset();
  _imu_ll_interrupt_setup(config.pin_int, gizmo->interrupt_has_rising_edge);
  _imu_ll_interrupt_enabled = true;
  interrupt_cnt = 0;
  update_cnt = 0;
  return rv;
}

bool Imu::update() {
  if(!gizmo) return false;

  // In quaternion math, to rotate an orientation q by another rotation qc, you compute:
  //
  //   q_corrected = qc * q
  //
  // applyQuatCorrection rotates q by a fixed angle and is used
  // to correct for IMU board mounting orientation. We use it when the
  // IMU is returning quaternions directly when it has sensor fusion implemented
  // on the chip. Assumes that the get9DOF() or get6DOF() are returning
  // quaternions in the RH NED frame.
  auto applyQuatCorrection = [](float q[4], const float qc[4]) {
    float w1 = qc[0], x1 = qc[1], y1 = qc[2], z1 = qc[3];
    float w2 = q[0],  x2 = q[1],  y2 = q[2],  z2 = q[3];
    q[0] = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    q[1] = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    q[2] = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    q[3] = w1*z2 + x1*y2 - y1*x2 + z1*w2;
  };

  //start of update timestamp
  uint32_t update_ts = micros();

  //get sensor data and update timestamps, count
  if(gizmo->has_mag) {
    if (gizmo->has_sensor_fusion) {
      gizmo->get9DOF(&q[0], &q[1], &q[2], &q[3]);
    }
    gizmo->getMotion9NED(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  }
  else {
    if (gizmo->has_sensor_fusion) {
      gizmo->get6DOF(&q[0], &q[1], &q[2], &q[3]);
    }
    gizmo->getMotion6NED(&ax, &ay, &az, &gx, &gy, &gz);
  }

  //handle rotation for different mounting positions
  switch((Cfg::imu_align_enum)cfg.imu_align) {
    case Cfg::imu_align_enum::mf_CW0 :
      // Input quaternion q[] is assumed to be in RH NED frame already
      // No correction needed.
      break;
    case Cfg::imu_align_enum::mf_CW90 :
      if (gizmo->has_sensor_fusion) {
        // Input quaternion q[] is assumed to be in RH NED frame already
        const float qc[4] = { 0.7071f, 0.0f, 0.0f, -0.7071f }; // -90° about Z (corrects +90° CW mounting)
        applyQuatCorrection(q, qc);
      }
      else {
        float tmp;
        tmp = ax; ax = -ay; ay = tmp;
        tmp = gx; gx = -gy; gy = tmp;
        tmp = mx; mx = -my; my = tmp;
      }
      break;
    case Cfg::imu_align_enum::mf_CW180 :
      if (gizmo->has_sensor_fusion) {
        // Input quaternion q[] is assumed to be in RH NED frame already
        const float qc[4] = { 0.0f, 0.0f, 0.0f, 1.0f }; // 180° about Z
        applyQuatCorrection(q, qc);
      }
      else {
        ax = -ax; ay = -ay;
        gx = -gx; gy = -gy;
        mx = -mx; my = -my;
      }
      break;
    case Cfg::imu_align_enum::mf_CW270 :
      if (gizmo->has_sensor_fusion) {
        // Input quaternion q[] is assumed to be in RH NED frame already
        const float qc[4] = { 0.7071f, 0.0f, 0.0f, 0.7071f }; // +90° about Z (corrects -90° CW mounting)
        applyQuatCorrection(q, qc);
      }
      else {
        float tmp;
        tmp = ax; ax = ay; ay = -tmp;
        tmp = gx; gx = gy; gy = -tmp;
        tmp = mx; mx = my; my = -tmp;
      }
      break;
    case Cfg::imu_align_enum::mf_CW0FLIP :
      if (gizmo->has_sensor_fusion) {
        // Input quaternion q[] is assumed to be in RH NED frame already
        const float qc[4] = { 0.0f, 1.0f, 0.0f, 0.0f }; // flip about X (Z up to Z down)
        applyQuatCorrection(q, qc);
      }
      else {
        az = -az;
        gz = -gz;
        mz = -mz;
      }
      break;
    case Cfg::imu_align_enum::mf_CW90FLIP :
      if (gizmo->has_sensor_fusion) {
        // Input quaternion q[] is assumed to be in RH NED frame already
        const float qc[4] = { 0.0f, 0.7071f, 0.7071f, 0.0f }; // -90° about Z then flip about X (Z up to down)
        applyQuatCorrection(q, qc);
      }
      else {
        float tmp;
        tmp = ax; ax = -ay; ay = tmp; az = -az;
        tmp = gx; gx = -gy; gy = tmp; gz = -gz;
        tmp = mx; mx = -my; my = tmp; mz = -mz;
      }
      break;
    case Cfg::imu_align_enum::mf_CW180FLIP :
      if (gizmo->has_sensor_fusion) {
        // Input quaternion q[] is assumed to be in RH NED frame already
        const float qc[4] = { 0.0f, 0.0f, 1.0f, 0.0f }; // flip about Y
        applyQuatCorrection(q, qc);
      }
      else {
        ax = -ax; ay = -ay; az = -az;
        gx = -gx; gy = -gy; gz = -gz;
        mx = -mx; my = -my; mz = -mz;
      }
      break;
    case Cfg::imu_align_enum::mf_CW270FLIP :
      if (gizmo->has_sensor_fusion) {
        // Input quaternion q[] is assumed to be in RH NED frame already
        const float qc[4] = { 0.5f, 0.5f, 0.5f, -0.5f }; // +90° about Z then flip about X
        applyQuatCorrection(q, qc);
      }
      else {
        float tmp;
        tmp = ax; ax = ay; ay = -tmp; az = -az;
        tmp = gx; gx = gy; gy = -tmp; gz = -gz;
        tmp = mx; mx = my; my = -tmp; mz = -mz;
      }
      break;
  }

  this->dt = (update_ts - this->ts) / 1000000.0;
  this->ts = update_ts;
  this->update_cnt++;

  return true; //FIXME: should only return true if new sample was retrieved
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
  //latency between start of low level interrupt handler and start of this method
  stat_latency += micros() - _imu_ll_interrupt_ts;

  //start of task timestamp
  uint32_t task_ts = micros();

  //update sensor data
  update();

  this->stat_io_runtime += micros() - task_ts; //runtime of SPI/I2C transfer

  //call external update event handler
  if(onUpdate) onUpdate();

  uint32_t rt = micros() - task_ts; //runtime of full update including external event handler
  this->stat_runtime += rt;
  if(this->stat_runtime_max < rt) this->stat_runtime_max = rt; //max runtime
  this->stat_cnt++;
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

  void _imu_ll_interrupt_setup(int interrupt_pin, bool rising_edge) {
    if(!_imu_ll_task_handle) {
      //
      #if IMU_EXEC == IMU_EXEC_FREERTOS_OTHERCORE
        int callcore = hal_get_core_num();
        int othercore = (callcore+1)%2;

        //TODO move this to hal
        #if defined ARDUINO_ARCH_ESP32
          //note: probably don't what to use this because of single FPU context switching issues...
          xTaskCreatePinnedToCore(_imu_ll_task, "IMU", MF_FREERTOS_DEFAULT_STACK_SIZE, NULL, IMU_FREERTOS_TASK_PRIORITY /*priority 0=lowest*/, &_imu_ll_task_handle, othercore); //[ESP32 only]
        #elif defined ARDUINO_ARCH_RP2040
          xTaskCreate(_imu_ll_task, "IMU", MF_FREERTOS_DEFAULT_STACK_SIZE, NULL, IMU_FREERTOS_TASK_PRIORITY /*priority 0=lowest*/, &_imu_ll_task_handle);
          vTaskCoreAffinitySet(_imu_ll_task_handle, (1<<othercore)); //[RP2040 only] Sets the core affinity mask for a task, i.e. the cores on which a task can run.
        #else
          #error "IMU_EXEC == IMU_EXEC_FREERTOS_OTHERCORE not supported on this processor"
        #endif

        Serial.printf(MF_MOD ": IMU_EXEC_FREERTOS_OTHERCORE call_core=%d imu_core=%d\n", callcore, othercore);
      #else
        xTaskCreate(_imu_ll_task, "IMU", MF_FREERTOS_DEFAULT_STACK_SIZE, NULL, IMU_FREERTOS_TASK_PRIORITY /*priority 0=lowest*/, &_imu_ll_task_handle);
        Serial.println(MF_MOD ": IMU_EXEC_FREERTOS");
      #endif
    }
    if (rising_edge) {
      attachInterrupt(digitalPinToInterrupt(interrupt_pin), _imu_ll_interrupt_handler, RISING);
    } else {
      attachInterrupt(digitalPinToInterrupt(interrupt_pin), _imu_ll_interrupt_handler, FALLING);
    }
    Serial.printf("Attached interrupt to pin %d with %s edge\n", interrupt_pin, rising_edge ? "RISING" : "FALLING");
  }

  inline void _imu_ll_interrupt_handler2() {
    //let RTOS task _imu_ll_task handle the interrupt
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(_imu_ll_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
//-------------------------------------------------------------------------------------------------------------------------
#elif IMU_EXEC == IMU_EXEC_IRQ

  void _imu_ll_interrupt_setup(int interrupt_pin, bool rising_edge) {
    Serial.println(MF_MOD ": IMU_EXEC_IRQ");
    if (rising_edge) {
      attachInterrupt(digitalPinToInterrupt(interrupt_pin), _imu_ll_interrupt_handler, RISING);
    } else {
      attachInterrupt(digitalPinToInterrupt(interrupt_pin), _imu_ll_interrupt_handler, FALLING);
    }
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

#undef MF_MOD
