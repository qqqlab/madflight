/*==========================================================================================
MIT License

Copyright (c) 2023-2026 https://madflight.com

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

#define MF_MOD "IMU"

#include "./imu.h"
#include "../cfg/cfg.h"
#include "../hal/hal.h"
#include "../bbx/bbx.h"
#include "../mag/MagGizmoIMU.h"
#include "../cli/stat.h" //for Stat, used by cli_calibrate_accel()

//the "gizmos"
#include "ImuGizmoMPUXXXX.h"
#include "ImuGizmoBMI270.h"
#include "ImuGizmoICM45686.h"
#include "ImuGizmoICM426XX.h"
#include "ImuGizmoAuto.h"

//global module class instance
Imu imu;

void _imu_ll_interrupt_setup(int interrupt_pin); //prototype
volatile bool _imu_ll_interrupt_enabled = false;
volatile bool _imu_ll_interrupt_busy = false;
volatile uint32_t _imu_ll_interrupt_ts = 0;

//returns 0 on success, positive on error, negative on warning
int Imu::setup() {
  cfg.printModule("IMU");
  
  //disable interrupt handler
  _imu_ll_interrupt_enabled = false;

  delete gizmo;

  //exit if gizmo == NONE
  if(config.gizmo == Cfg::imu_gizmo_enum::mf_NONE) return 0;

  //check bus config
  if (!config.uses_i2c && (!config.spi_bus || config.spi_cs < 0)) {
    Serial.println("IMU: ERROR SPI sensor without imu_spi_bus and/or pin_imu_cs");
    cfg.printModule(MF_MOD, CfgClass::printModuleMode::CFG_ERROR);
    return -1;
  }
  if (config.uses_i2c && !config.i2c_bus) {
    Serial.println("IMU: ERROR I2C sensor without imu_i2c_bus");
    cfg.printModule(MF_MOD, CfgClass::printModuleMode::CFG_ERROR);
    return -1;
  }

  //create gizmo
  switch(config.gizmo) {
    case Cfg::imu_gizmo_enum::mf_NONE : {
      //do nothing
      break;
    }    
    case Cfg::imu_gizmo_enum::mf_AUTO : {
      gizmo = ImuGizmoAuto::create(&config, (ImuState*)this);
      break;
    }
    case Cfg::imu_gizmo_enum::mf_MPU9250 :
    case Cfg::imu_gizmo_enum::mf_MPU9150 :
    case Cfg::imu_gizmo_enum::mf_MPU6500 :
    case Cfg::imu_gizmo_enum::mf_MPU6050 :
    case Cfg::imu_gizmo_enum::mf_MPU6000 : {
      gizmo = ImuGizmoMPUXXXX::create(&config, (ImuState*)this);
      break;
    }
    case Cfg::imu_gizmo_enum::mf_BMI270 : {
      gizmo = ImuGizmoBMI270::create(&config, (ImuState*)this);
      break;
    }
    case Cfg::imu_gizmo_enum::mf_ICM42688 :
    case Cfg::imu_gizmo_enum::mf_ICM42688P : {
      gizmo = ImuGizmoICM426XX::create(&config, (ImuState*)this);
      break;
    }
    case Cfg::imu_gizmo_enum::mf_ICM45686 : {
      gizmo = ImuGizmoICM45686::create(&config, (ImuState*)this);
      break;
    }
    case Cfg::imu_gizmo_enum::mf_LSM6DSV : {
      gizmo = ImuGizmoLSM6DSV::create(&config, (ImuState*)this);
      break;
    }
    case Cfg::imu_gizmo_enum::mf_LSM6DSO : {
      gizmo = ImuGizmoLSM6DSO::create(&config, (ImuState*)this);
      break;
    }
    case Cfg::imu_gizmo_enum::mf_LSM6DSV16B : {
      gizmo = ImuGizmoLSM6DSV16B::create(&config, (ImuState*)this);
      break;
    }
  }

  //check gizmo
  if(!gizmo && config.gizmo != Cfg::imu_gizmo_enum::mf_NONE) {
    Serial.println("\nIMU: ERROR sensor not configured, check pin/bus/bus_type config\n");
    return -1001;
  }

  //exit if no gizmo installed
  if(!gizmo) return 0;

  //exit if no interrupt pin specified
  if(config.pin_int < 0) return 0;

  //==============================================================
  //setup low-level interrupt stuff
  //==============================================================
  //_imu_ll_interrupt_enabled = false; //already done
  onUpdate = NULL;
  _imu_ll_interrupt_busy = false;
  _imu_ll_interrupt_ts = 0;
  overrun_cnt = 0;
  dt = 0;
  ts = micros();
  statReset();
  _imu_ll_interrupt_setup(config.pin_int);
  _imu_ll_interrupt_enabled = true;
  interrupt_cnt = 0;
  update_cnt = 0;

  return 0;
}

/*
static void _imu_rotate(Cfg::imu_align_enum align, float *in, float *out) {
    switch(align) {
      case Cfg::imu_align_enum::mf_CW0 :
        out[0] = +in[0];
        out[1] = +in[1];
        out[2] = +in[2];
        break;
      case Cfg::imu_align_enum::mf_CW90 :
        out[0] = -in[1];
        out[1] = +in[0];
        out[2] = +in[2];
        break;
      case Cfg::imu_align_enum::mf_CW180 :
        out[0] = -in[0];
        out[1] = -in[1];
        out[2] = +in[2];
        break;
      case Cfg::imu_align_enum::mf_CW270 :
        out[0] = +in[1];
        out[1] = -in[0];
        out[2] = +in[2];
        break;
      case Cfg::imu_align_enum::mf_CW0FLIP :
        out[0] = +in[0];
        out[1] = -in[1];
        out[2] = -in[2];
        break;
      case Cfg::imu_align_enum::mf_CW90FLIP :
        out[0] = +in[1];
        out[1] = +in[0];
        out[2] = -in[2];
        break;
      case Cfg::imu_align_enum::mf_CW180FLIP :
        out[0] = -in[0];
        out[1] = +in[1];
        out[2] = -in[2];
        break;
      case Cfg::imu_align_enum::mf_CW270FLIP :
        out[0] = -in[1];
        out[1] = -in[0];
        out[2] = -in[2];
        break;
    }
}
*/

bool Imu::update() {
  runtimeTrace.start();
  bool updated = (gizmo != nullptr);
  if(updated) {
    //start of update timestamp
    uint32_t update_ts = micros();

    //get sensor data and update timestamps, count
    if(config.has_mag) {
      float mx, my, mz;
      gizmo->getMotion9NED(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
      if(config.pmag) {
        if(((MagGizmoIMU*)config.pmag->gizmo)->push_nwu_from_imu(mx, my, mz)) {
          config.pmag->update();
        }
      }
    }else{
      gizmo->getMotion6NED(&ax, &ay, &az, &gx, &gy, &gz);
    }

    // apply zero offsets
    ax -= config.imu_cal_ax[0];
    ay -= config.imu_cal_ax[1];
    az -= config.imu_cal_ax[2];
    gx -= config.imu_cal_gx[0];
    gy -= config.imu_cal_gx[1];
    gz -= config.imu_cal_gx[2];

    // apply accel scale factors - corrects per-axis gain error (e.g. a well-calibrated
    // sensor should read |a|=1g in ANY static orientation; a raw scale mismatch between
    // axes shows up as a different apparent |a| depending on which axis carries gravity).
    // Applied here (raw sensor frame, after offset removal, before rotation) to match
    // where offset correction happens, since scale is a property of the physical axis,
    // not the vehicle mounting orientation. See Imu::cli_calibrate_accel().
    ax *= config.imu_cal_ax_scale[0];
    ay *= config.imu_cal_ax_scale[1];
    az *= config.imu_cal_ax_scale[2];

    //handle imu rotation for different mounting positions (inline for speed)
    switch(*config.imu_align) {
      case Cfg::imu_align_enum::mf_CW0 :
        break;
      case Cfg::imu_align_enum::mf_CW90 :
        { float tmp; tmp=ax; ax=-ay; ay=tmp;   tmp=gx; gx=-gy; gy=tmp; }
        break;
      case Cfg::imu_align_enum::mf_CW180 :
        { ax=-ax; ay=-ay;   gx=-gx; gy=-gy; }
        break;
      case Cfg::imu_align_enum::mf_CW270 :
        { float tmp; tmp=ax; ax=ay; ay=-tmp;   tmp=gx; gx=gy; gy=-tmp; }
        break;
      case Cfg::imu_align_enum::mf_CW0FLIP :
        { ay=-ay; az=-az;   gy=-gy; gz=-gz;}
        break;
      case Cfg::imu_align_enum::mf_CW90FLIP :
        { float tmp; tmp=ax; ax=ay; ay=tmp; az=-az;   tmp=gx; gx=gy; gy=tmp; gz=-gz;}
        break;
      case Cfg::imu_align_enum::mf_CW180FLIP :
        { ax=-ax; az=-az;   gx=-gx; gz=-gz; }
        break;
      case Cfg::imu_align_enum::mf_CW270FLIP :
        { float tmp; tmp=ax; ax=-ay; ay=-tmp; az=-az;   tmp=gx; gx=-gy; gy=-tmp; gz=-gz; }
        break;
    }

    this->dt = (update_ts - this->ts) / 1000000.0;
    this->ts = update_ts;
    this->update_cnt++;

    topic.publish(this);
  }
  runtimeTrace.stop(updated);
  return updated;
}

void Imu::convert_to_raw(float *gyr, float *acc) {
    float gx = gyr[0];
    float gy = gyr[1];
    float gz = gyr[2];
    float ax = acc[0];
    float ay = acc[1];
    float az = acc[2];

    //un-rotate
    switch(*config.imu_align) {
      case Cfg::imu_align_enum::mf_CW0 :
        break;
      case Cfg::imu_align_enum::mf_CW90 :
        //{ float tmp; tmp=ax; ax=-ay; ay=tmp;   tmp=gx; gx=-gy; gy=tmp; } //rot
        { float tmp; tmp=ax; ax=ay; ay=-tmp;   tmp=gx; gx=gy; gy=-tmp; } //unrot = mf_CW270
        break;
      case Cfg::imu_align_enum::mf_CW180 :
        //{ ax=-ax; ay=-ay;   gx=-gx; gy=-gy; } //rot
        { ax=-ax; ay=-ay;   gx=-gx; gy=-gy; } //unrot = same
        break;
      case Cfg::imu_align_enum::mf_CW270 :
        //{ float tmp; tmp=ax; ax=ay; ay=-tmp;   tmp=gx; gx=gy; gy=-tmp; } //rot
        { float tmp; tmp=ax; ax=-ay; ay=tmp;   tmp=gx; gx=-gy; gy=tmp; } //unrot = mf_CW270
        break;
      case Cfg::imu_align_enum::mf_CW0FLIP :
        //{ ay=-ay; az=-az;   gy=-gy; gz=-gz;} //rot
        { ay=-ay; az=-az;   gy=-gy; gz=-gz;} //unrot = same
        break;
      case Cfg::imu_align_enum::mf_CW90FLIP :
        //{ float tmp; tmp=ax; ax=ay; ay=tmp; az=-az;   tmp=gx; gx=gy; gy=tmp; gz=-gz;} //rot
        { float tmp; tmp=ax; ax=ay; ay=tmp; az=-az;   tmp=gx; gx=gy; gy=tmp; gz=-gz;} //unrot = same
        break;
      case Cfg::imu_align_enum::mf_CW180FLIP :
        //{ ax=-ax; az=-az;   gx=-gx; gz=-gz; } //rot
        { ax=-ax; az=-az;   gx=-gx; gz=-gz; } //unrot = same
        break;
      case Cfg::imu_align_enum::mf_CW270FLIP :
        //{ float tmp; tmp=ax; ax=-ay; ay=-tmp; az=-az;   tmp=gx; gx=-gy; gy=-tmp; gz=-gz; } //rot
         { float tmp; tmp=ax; ax=-ay; ay=-tmp; az=-az;   tmp=gx; gx=-gy; gy=-tmp; gz=-gz; } //unrot = same
        break;
    }

    // un-apply accel scale factors (inverse of Imu::update()'s multiply - divide first,
    // since offset must be un-applied last, mirroring the forward order in reverse)
    ax /= config.imu_cal_ax_scale[0];
    ay /= config.imu_cal_ax_scale[1];
    az /= config.imu_cal_ax_scale[2];

    // un-apply zero offsets
    ax += config.imu_cal_ax[0];
    ay += config.imu_cal_ax[1];
    az += config.imu_cal_ax[2];
    gx += config.imu_cal_gx[0];
    gy += config.imu_cal_gx[1];
    gz += config.imu_cal_gx[2];

    gyr[0] = gx;
    gyr[1] = gy;
    gyr[2] = gz;
    acc[0] = ax;
    acc[1] = ay;
    acc[2] = az;
}

// 6-point accelerometer scale + offset calibration.
// Unlike the single-position offset-only calibration that runs via the
// CLI 'calimu' command (see Cli::calibrate_IMU2), this also corrects per-axis SCALE errors -
// which is needed because a well-calibrated accelerometer should read |a|=1g in ANY static orientation.
//
// This function detects which axis actually reads near +-1g each time and
// tracks which of the 6 (axis, sign) combinations have been captured, rejecting duplicates
// and unclear/non-stationary readings, until all 6 are found.
void Imu::cli_calibrate_accel() {
  if (!installed()) {
    Serial.println("ERROR: No IMU");
    return;
  }

  Serial.println("\n6-point accelerometer calibration.");
  Serial.println("Rest the sensor stationary on any 6 distinct faces, one at a time, in any order.");
  Serial.println("The axis is detected automatically.\n");

  auto imu_sub = MsgSubscription<ImuState>("calacc", &imu.topic);

  bool have[3][2] = {{false,false},{false,false},{false,false}}; //[axis][sign: 0=+,1=-]
  float value[3][2];
  int collected = 0;

  while (collected < 6) {
    Serial.printf("Need %d more orientation(s). Missing: ", 6 - collected);
    const char* names[3] = {"axis A", "axis B", "axis C"};
    for (int ax = 0; ax < 3; ax++) {
      if (!have[ax][0]) Serial.printf("+%s ", names[ax]);
      if (!have[ax][1]) Serial.printf("-%s ", names[ax]);
    }
    Serial.println("\nRest the sensor on a NEW face, hold still, then press any key");
    while(Serial.available()) Serial.read(); //clear input

    //live in-place preview until Enter is pressed
    ImuState s;
    float live[3] = {0,0,0};
    while (!Serial.available()) {
      if (imu_sub.pull_next(&s)) {
        convert_to_raw(&s.gx, &s.ax);
        live[0] = s.ax; live[1] = s.ay; live[2] = s.az;
      }
      Serial.printf("\r  live: ax=%+.3f ay=%+.3f az=%+.3f          ", live[0], live[1], live[2]);
      delay(100);
    }
    Serial.println();
    while(Serial.available()) Serial.read(); //consume the Enter itself

    Serial.println("Recording...");
    Stat a[3];
    uint32_t ts = millis();
    const int timeout = 2000; //2 seconds
    while (millis() - ts < timeout) {
      if (imu_sub.pull_next(&s)) {
        convert_to_raw(&s.gx, &s.ax);
        a[0].append(s.ax);
        a[1].append(s.ay);
        a[2].append(s.az);
      }
    }
    if (a[0].n == 0) {
      Serial.println("ERROR: no IMU samples received - try again.\n");
      continue;
    }
    float m[3] = {a[0].mean(), a[1].mean(), a[2].mean()};

    //find the dominant axis (largest |value|) - that's whichever one gravity loaded
    int axis = 0;
    if (fabs(m[1]) > fabs(m[axis])) axis = 1;
    if (fabs(m[2]) > fabs(m[axis])) axis = 2;
    float mag = fabs(m[axis]);
    int sign = (m[axis] >= 0) ? 0 : 1;

    if (mag < 0.7f || mag > 1.3f) {
      Serial.printf("ERROR: dominant axis read %.3fg, expected ~1g - sensor wasn't flat/stationary. Try again.\n\n", mag);
      continue;
    }
    if (have[axis][sign]) {
      Serial.println("This orientation was already captured - it's a duplicate, not a new face. Try a different one.\n");
      continue;
    }

    have[axis][sign] = true;
    value[axis][sign] = m[axis];
    collected++;
    Serial.printf("  recorded: axis=%d sign=%s value=%+.4f (n=%d, stdev=%.4f/%.4f/%.4f)\n\n",
      axis, sign == 0 ? "+" : "-", m[axis], a[0].n, a[0].std(), a[1].std(), a[2].std());
  }

  //for each axis, its own +/- pair should ideally be exactly +1.0/-1.0g.
  //Their average gives offset (should be ~0); their difference vs the ideal 2g swing gives scale.
  float offset[3], scale[3];
  for (int axis = 0; axis < 3; axis++) {
    float plus  = value[axis][0];
    float minus = value[axis][1];
    offset[axis] = (plus + minus) / 2.0f;
    float swing = plus - minus; //ideally +2.0
    scale[axis] = (swing != 0) ? (2.0f / swing) : 1.0f;
  }

  const char axisname[3] = {'x', 'y', 'z'};
  Serial.println("Calibration complete.");
  for (int axis = 0; axis < 3; axis++) {
    Serial.printf("set imu_cal_a%c  %+f #config was %+f\n", axisname[axis], offset[axis], config.imu_cal_ax[axis]);
  }
  for (int axis = 0; axis < 3; axis++) {
    Serial.printf("set imu_cal_as%c %+f #config was %+f\n", axisname[axis], scale[axis], config.imu_cal_ax_scale[axis]);
  }
  Serial.println("Type 'save' to save these values to flash");

  //apply immediately, same as Cli::calibrate_IMU2() / Mag::cli_calibrate()
  for (int axis = 0; axis < 3; axis++) {
    config.imu_cal_ax[axis] = offset[axis];
    config.imu_cal_ax_scale[axis] = scale[axis];
  }
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

  void _imu_ll_interrupt_setup(int interrupt_pin) {
    if(!_imu_ll_task_handle) {
      //
      #if IMU_EXEC == IMU_EXEC_FREERTOS_OTHERCORE
        int callcore = hal_get_core_num();
        int othercore = (callcore + 1) % 2;
        
        //MF_TODO move this to hal
        #if defined ARDUINO_ARCH_ESP32
          //note: probably don't what to use this because of single FPU context switching issues...
          xTaskCreatePinnedToCore(_imu_ll_task, "mf_IMU", MF_FREERTOS_DEFAULT_STACK_SIZE, NULL, IMU_FREERTOS_TASK_PRIORITY /*priority 0=lowest*/, &_imu_ll_task_handle, othercore); //[ESP32 only]
        #elif defined ARDUINO_ARCH_RP2040
          xTaskCreate(_imu_ll_task, "mf_IMU", MF_FREERTOS_DEFAULT_STACK_SIZE, NULL, IMU_FREERTOS_TASK_PRIORITY /*priority 0=lowest*/, &_imu_ll_task_handle);
          vTaskCoreAffinitySet(_imu_ll_task_handle, (1<<othercore)); //[RP2040 only] Sets the core affinity mask for a task, i.e. the cores on which a task can run.
        #else
          #error "IMU_EXEC == IMU_EXEC_FREERTOS_OTHERCORE not supported on this processor"
        #endif

        Serial.printf("IMU: IMU_EXEC_FREERTOS_OTHERCORE call_core=%d imu_core=%d\n", callcore, othercore);
      #else
        xTaskCreate(_imu_ll_task, "mf_IMU", MF_FREERTOS_DEFAULT_STACK_SIZE, NULL, IMU_FREERTOS_TASK_PRIORITY /*priority 0=lowest*/, &_imu_ll_task_handle);
        Serial.println("IMU: IMU_EXEC_FREERTOS");
      #endif
    }
    attachInterrupt(digitalPinToInterrupt(interrupt_pin), _imu_ll_interrupt_handler, RISING);
  }
  
  inline void _imu_ll_interrupt_handler2() {
    //let RTOS task _imu_ll_task handle the interrupt
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(_imu_ll_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
//-------------------------------------------------------------------------------------------------------------------------
#elif IMU_EXEC == IMU_EXEC_IRQ

  void _imu_ll_interrupt_setup(int interrupt_pin) {
    Serial.println("IMU: IMU_EXEC_IRQ");
    attachInterrupt(digitalPinToInterrupt(interrupt_pin), _imu_ll_interrupt_handler, RISING);
    #if MF_HACK_STM32_INTERRUPT_PRIORITY
      // (#79) on stm32, set the interrupt to the highest priority, otherwise it does not get called
      HAL_NVIC_SetPriority(hal_get_irqn_from_pin(interrupt_pin), 0, 0);
    #endif
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


