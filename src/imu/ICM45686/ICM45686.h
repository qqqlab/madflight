//modified for madflight - remove interrupt handler, add hires_en

/*
 *
 * Copyright (c) [2020] by InvenSense, Inc.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */
 
#ifndef ICM456xx_H
#define ICM456xx_H

#include "Arduino.h"
#include "SPI.h"
#include "Wire.h"

#define ICM45686

extern "C" {
#include "./imu/inv_imu_driver_advanced.h"
#include "./imu/inv_imu_edmp.h"
#if defined(ICM45686S) || defined(ICM45605S)
#include "./imu/inv_imu_edmp_gaf.h"
#endif
}

enum {
  ICM456XX_APEX_TILT=0,
  ICM456XX_APEX_PEDOMETER,
  ICM456XX_APEX_TAP,
  ICM456XX_APEX_R2W,
  ICM456XX_APEX_NB,
};

// This defines the handler called when retrieving a sample from the FIFO
//typedef void (*ICM456xx_sensor_event_cb)(inv_imu_sensor_data_t *event);
// This defines the handler called when receiving an irq
typedef void (*ICM456xx_irq_handler)(void);

class ICM456xx {
  public:
    ICM456xx(MF_I2C &i2c,bool address_lsb, uint32_t freq);
    ICM456xx(MF_I2C &i2c,bool address_lsb);
    ICM456xx(SPIClass &spi,uint8_t chip_select_id, uint32_t freq);
    ICM456xx(SPIClass &spi,uint8_t chip_select_id);
    int begin();
    int startAccel(uint16_t odr, uint16_t fsr);
    int startGyro(uint16_t odr, uint16_t fsr);
    int getDataFromRegisters(inv_imu_sensor_data_t& data);
    int enableFifoInterrupt(uint8_t fifo_watermark);
    int getDataFromFifo(inv_imu_fifo_data_t& data);
#if defined(ICM45686S) || defined(ICM45605S)
    int startGaf(uint8_t intpin, ICM456xx_irq_handler handler);
    int getGafData(inv_imu_edmp_gaf_outputs_t& gaf_outputs);
    int getGafData(float& quatW,float& quatX,float& quatY,float& quatZ);
#endif
    int stopAccel(void);
    int stopGyro(void);
    int startTiltDetection(uint8_t intpin=2, ICM456xx_irq_handler handler=NULL);
    int startPedometer(uint8_t intpin=2, ICM456xx_irq_handler handler=NULL);
    int getPedometer(uint32_t& step_count, float& step_cadence, char*& activity);
    int startWakeOnMotion(uint8_t intpin, ICM456xx_irq_handler handler);
    int startTap(uint8_t intpin=2, ICM456xx_irq_handler handler=NULL);
    bool getTilt(void);
    int getTap(uint8_t& tap_count, uint8_t& axis, uint8_t& direction);
    int startRaiseToWake(uint8_t intpin=2, ICM456xx_irq_handler handler=NULL);
    int getRaiseToWake(void);
    int updateApex(void);
    int setApexInterrupt(uint8_t intpin, ICM456xx_irq_handler handler);
    inv_imu_edmp_int_state_t apex_status;

  protected:
    inv_imu_device_t icm_driver;
    accel_config0_accel_odr_t accel_freq_to_param(uint16_t accel_freq_hz);
    gyro_config0_gyro_odr_t gyro_freq_to_param(uint16_t gyro_freq_hz);
    accel_config0_accel_ui_fs_sel_t accel_fsr_g_to_param(uint16_t accel_fsr_g);
    gyro_config0_gyro_ui_fs_sel_t gyro_fsr_dps_to_param(uint16_t gyro_fsr_dps);
    int setup_irq();
    int startAPEX(dmp_ext_sen_odr_cfg_apex_odr_t edmp_odr, accel_config0_accel_odr_t accel_odr);
    uint32_t step_cnt_ovflw;
    bool apex_enable[ICM456XX_APEX_NB];
    dmp_ext_sen_odr_cfg_apex_odr_t apex_edmp_odr;
    accel_config0_accel_odr_t apex_accel_odr;
};

#endif // ICM456xx_H
