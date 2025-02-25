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

#ifndef _INV_IMU_REGMAP_LE_H_
#define _INV_IMU_REGMAP_LE_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @file inv_imu_regmap_le.h
 * File exposing the device register map
 */


/* DREG_BANK1 */
#define ACCEL_DATA_X1_UI                                                        0x00
#define ACCEL_DATA_X0_UI                                                        0x01
#define ACCEL_DATA_Y1_UI                                                        0x02
#define ACCEL_DATA_Y0_UI                                                        0x03
#define ACCEL_DATA_Z1_UI                                                        0x04
#define ACCEL_DATA_Z0_UI                                                        0x05
#define GYRO_DATA_X1_UI                                                         0x06
#define GYRO_DATA_X0_UI                                                         0x07
#define GYRO_DATA_Y1_UI                                                         0x08
#define GYRO_DATA_Y0_UI                                                         0x09
#define GYRO_DATA_Z1_UI                                                         0x0a
#define GYRO_DATA_Z0_UI                                                         0x0b
#define TEMP_DATA1_UI                                                           0x0c
#define TEMP_DATA0_UI                                                           0x0d
#define TMST_FSYNCH                                                             0x0e
#define TMST_FSYNCL                                                             0x0f
#define PWR_MGMT0                                                               0x10
typedef struct {
	uint8_t accel_mode                                                             : 2;
	uint8_t gyro_mode                                                              : 2;
	uint8_t resv_1                                                                 : 4;
} pwr_mgmt0_t;

#define FIFO_COUNT_0                                                            0x12
#define FIFO_COUNT_1                                                            0x13
#define FIFO_DATA                                                               0x14
typedef struct {
	uint8_t fifo_data                                                              : 8;
} fifo_data_t;

#define INT1_CONFIG0                                                            0x16
typedef struct {
	uint8_t int1_status_en_fifo_full                                               : 1;
	uint8_t int1_status_en_fifo_ths                                                : 1;
	uint8_t int1_status_en_drdy                                                    : 1;
	uint8_t int1_status_en_aux1_drdy                                               : 1;
	uint8_t int1_status_en_ap_fsync                                                : 1;
	uint8_t int1_status_en_ap_agc_rdy                                              : 1;
	uint8_t int1_status_en_aux1_agc_rdy                                            : 1;
	uint8_t int1_status_en_reset_done                                              : 1;
} int1_config0_t;

#define INT1_CONFIG1                                                            0x17
typedef struct {
	uint8_t int1_status_en_pll_rdy                                                 : 1;
	uint8_t int1_status_en_wom_x                                                   : 1;
	uint8_t int1_status_en_wom_y                                                   : 1;
	uint8_t int1_status_en_wom_z                                                   : 1;
	uint8_t int1_status_en_i3c_protocol_err                                        : 1;
	uint8_t int1_status_en_i2cm_done                                               : 1;
	uint8_t int1_status_en_apex_event                                              : 1;
	uint8_t resv_1                                                                 : 1;
} int1_config1_t;

#define INT1_CONFIG2                                                            0x18
typedef struct {
	uint8_t int1_polarity                                                          : 1;
	uint8_t int1_mode                                                              : 1;
	uint8_t int1_drive                                                             : 1;
	uint8_t resv_1                                                                 : 5;
} int1_config2_t;

#define INT1_STATUS0                                                            0x19
typedef struct {
	uint8_t int1_status_fifo_full                                                  : 1;
	uint8_t int1_status_fifo_ths                                                   : 1;
	uint8_t int1_status_drdy                                                       : 1;
	uint8_t int1_status_aux1_drdy                                                  : 1;
	uint8_t int1_status_ap_fsync                                                   : 1;
	uint8_t int1_status_ap_agc_rdy                                                 : 1;
	uint8_t int1_status_aux1_agc_rdy                                               : 1;
	uint8_t int1_status_reset_done                                                 : 1;
} int1_status0_t;

#define INT1_STATUS1                                                            0x1a
typedef struct {
	uint8_t int1_status_pll_rdy                                                    : 1;
	uint8_t int1_status_wom_x                                                      : 1;
	uint8_t int1_status_wom_y                                                      : 1;
	uint8_t int1_status_wom_z                                                      : 1;
	uint8_t int1_status_i3c_protocol_err                                           : 1;
	uint8_t int1_status_i2cm_done                                                  : 1;
	uint8_t int1_status_apex_event                                                 : 1;
	uint8_t resv_1                                                                 : 1;
} int1_status1_t;

#define ACCEL_CONFIG0                                                           0x1b
typedef struct {
	uint8_t accel_odr                                                              : 4;
	uint8_t accel_ui_fs_sel                                                        : 3;
	uint8_t resv_1                                                                 : 1;
} accel_config0_t;

#define GYRO_CONFIG0                                                            0x1c
typedef struct {
	uint8_t gyro_odr                                                               : 4;
	uint8_t gyro_ui_fs_sel                                                         : 4;
} gyro_config0_t;

#define FIFO_CONFIG0                                                            0x1d
typedef struct {
	uint8_t fifo_depth                                                             : 6;
	uint8_t fifo_mode                                                              : 2;
} fifo_config0_t;

#define FIFO_CONFIG1_0                                                          0x1e
#define FIFO_CONFIG1_1                                                          0x1f
#define FIFO_CONFIG2                                                            0x20
typedef struct {
	uint8_t resv_1                                                                 : 3;
	uint8_t fifo_wr_wm_gt_th                                                       : 1;
	uint8_t resv_2                                                                 : 3;
	uint8_t fifo_flush                                                             : 1;
} fifo_config2_t;

#define FIFO_CONFIG3                                                            0x21
typedef struct {
	uint8_t fifo_if_en                                                             : 1;
	uint8_t fifo_accel_en                                                          : 1;
	uint8_t fifo_gyro_en                                                           : 1;
	uint8_t fifo_hires_en                                                          : 1;
	uint8_t fifo_es0_en                                                            : 1;
	uint8_t fifo_es1_en                                                            : 1;
	uint8_t resv_1                                                                 : 2;
} fifo_config3_t;

#define FIFO_CONFIG4                                                            0x22
typedef struct {
	uint8_t fifo_es0_6b_9b                                                         : 1;
	uint8_t fifo_tmst_fsync_en                                                     : 1;
	uint8_t fifo_comp_en                                                           : 1;
	uint8_t fifo_comp_nc_flow_cfg                                                  : 3;
	uint8_t resv_1                                                                 : 2;
} fifo_config4_t;

#define TMST_WOM_CONFIG                                                         0x23
typedef struct {
	uint8_t wom_int_dur                                                            : 2;
	uint8_t wom_int_mode                                                           : 1;
	uint8_t wom_mode                                                               : 1;
	uint8_t wom_en                                                                 : 1;
	uint8_t tmst_resol                                                             : 1;
	uint8_t tmst_delta_en                                                          : 1;
	uint8_t resv_1                                                                 : 1;
} tmst_wom_config_t;

#define FSYNC_CONFIG0                                                           0x24
typedef struct {
	uint8_t ap_fsync_sel                                                           : 3;
	uint8_t ap_fsync_flag_clear_sel                                                : 1;
	uint8_t resv_1                                                                 : 4;
} fsync_config0_t;

#define FSYNC_CONFIG1                                                           0x25
typedef struct {
	uint8_t aux1_fsync_sel                                                         : 3;
	uint8_t aux1_fsync_flag_clear_sel                                              : 1;
	uint8_t resv_1                                                                 : 4;
} fsync_config1_t;

#define RTC_CONFIG                                                              0x26
typedef struct {
	uint8_t resv_1                                                                 : 5;
	uint8_t rtc_mode                                                               : 1;
	uint8_t rtc_align                                                              : 1;
	uint8_t resv_2                                                                 : 1;
} rtc_config_t;

#define DMP_EXT_SEN_ODR_CFG                                                     0x27
typedef struct {
	uint8_t apex_odr                                                               : 3;
	uint8_t ext_odr                                                                : 3;
	uint8_t ext_sensor_en                                                          : 1;
	uint8_t resv_1                                                                 : 1;
} dmp_ext_sen_odr_cfg_t;

#define ODR_DECIMATE_CONFIG                                                     0x28
typedef struct {
	uint8_t accel_fifo_odr_dec                                                     : 4;
	uint8_t gyro_fifo_odr_dec                                                      : 4;
} odr_decimate_config_t;

#define EDMP_APEX_EN0                                                           0x29
typedef struct {
	uint8_t tap_en                                                                 : 1;
	uint8_t reserved0                                                              : 1;
	uint8_t reserved1                                                              : 1;
	uint8_t tilt_en                                                                : 1;
	uint8_t pedo_en                                                                : 1;
	uint8_t ff_en                                                                  : 1;
	uint8_t r2w_en                                                                 : 1;
	uint8_t smd_en                                                                 : 1;
} edmp_apex_en0_t;

#define EDMP_APEX_EN1                                                           0x2a
typedef struct {
	uint8_t soft_hard_iron_corr_en                                                 : 1;
	uint8_t init_en                                                                : 1;
	uint8_t power_save_en                                                          : 1;
	uint8_t basic_smd_en                                                           : 1;
	uint8_t resv_1                                                                 : 1;
	uint8_t feature3_en                                                            : 1;
	uint8_t edmp_enable                                                            : 1;
	uint8_t resv_2                                                                 : 1;
} edmp_apex_en1_t;

#define APEX_BUFFER_MGMT                                                        0x2b
typedef struct {
	uint8_t step_count_edmp_wptr                                                   : 2;
	uint8_t step_count_host_rptr                                                   : 2;
	uint8_t ff_duration_edmp_wptr                                                  : 2;
	uint8_t ff_duration_host_rptr                                                  : 2;
} apex_buffer_mgmt_t;

#define INTF_CONFIG0                                                            0x2c
typedef struct {
	uint8_t ap_spi_mode                                                            : 1;
	uint8_t ap_spi_34_mode                                                         : 1;
	uint8_t resv_1                                                                 : 3;
	uint8_t virtual_access_aux1_en                                                 : 1;
	uint8_t resv_2                                                                 : 2;
} intf_config0_t;

#define INTF_CONFIG1_OVRD                                                       0x2d
typedef struct {
	uint8_t ap_spi_mode_ovrd_val                                                   : 1;
	uint8_t ap_spi_mode_ovrd                                                       : 1;
	uint8_t ap_spi_34_mode_ovrd_val                                                : 1;
	uint8_t ap_spi_34_mode_ovrd                                                    : 1;
	uint8_t resv_1                                                                 : 4;
} intf_config1_ovrd_t;

#define INTF_AUX_CONFIG                                                         0x2e
typedef struct {
	uint8_t aux1_spi_mode                                                          : 1;
	uint8_t aux1_spi_34_mode                                                       : 1;
	uint8_t aux2_spi_mode                                                          : 1;
	uint8_t resv_1                                                                 : 5;
} intf_aux_config_t;

#define IOC_PAD_SCENARIO                                                        0x2f
typedef struct {
	uint8_t aux1_enable                                                            : 1;
	uint8_t aux1_mode                                                              : 2;
	uint8_t aux2_enable                                                            : 1;
	uint8_t pads_int2_cfg                                                          : 2;
	uint8_t resv_1                                                                 : 2;
} ioc_pad_scenario_t;

#define IOC_PAD_SCENARIO_AUX_OVRD                                               0x30
typedef struct {
	uint8_t aux1_enable_ovrd_val                                                   : 1;
	uint8_t aux1_enable_ovrd                                                       : 1;
	uint8_t aux1_mode_ovrd_val                                                     : 2;
	uint8_t aux1_mode_ovrd                                                         : 1;
	uint8_t aux2_enable_ovrd_val                                                   : 1;
	uint8_t aux2_enable_ovrd                                                       : 1;
	uint8_t resv_1                                                                 : 1;
} ioc_pad_scenario_aux_ovrd_t;

#define IOC_PAD_SCENARIO_OVRD                                                   0x31
typedef struct {
	uint8_t pads_int2_cfg_ovrd_val                                                 : 2;
	uint8_t pads_int2_cfg_ovrd                                                     : 1;
	uint8_t resv_1                                                                 : 5;
} ioc_pad_scenario_ovrd_t;

#define DRIVE_CONFIG0                                                           0x32
typedef struct {
	uint8_t virtual_access_aux2_en                                                 : 1;
	uint8_t pads_spi_slew                                                          : 3;
	uint8_t pads_i2c_slew                                                          : 3;
	uint8_t resv_1                                                                 : 1;
} drive_config0_t;

#define DRIVE_CONFIG1                                                           0x33
typedef struct {
	uint8_t pads_i3c_sdr_slew                                                      : 3;
	uint8_t pads_i3c_ddr_slew                                                      : 3;
	uint8_t resv_1                                                                 : 2;
} drive_config1_t;

#define DRIVE_CONFIG2                                                           0x34
typedef struct {
	uint8_t pads_slew                                                              : 3;
	uint8_t resv_1                                                                 : 5;
} drive_config2_t;

#define REG_MISC1                                                               0x35
typedef struct {
	uint8_t osc_id_ovrd                                                            : 4;
	uint8_t resv_1                                                                 : 4;
} reg_misc1_t;

#define INT_APEX_CONFIG0                                                        0x39
typedef struct {
	uint8_t int_status_mask_pin_tap_detect                                         : 1;
	uint8_t int_status_mask_pin_high_g_det                                         : 1;
	uint8_t int_status_mask_pin_low_g_det                                          : 1;
	uint8_t int_status_mask_pin_tilt_det                                           : 1;
	uint8_t int_status_mask_pin_step_cnt_ovfl                                      : 1;
	uint8_t int_status_mask_pin_step_det                                           : 1;
	uint8_t int_status_mask_pin_ff_det                                             : 1;
	uint8_t int_status_mask_pin_r2w_wake_det                                       : 1;
} int_apex_config0_t;

#define INT_APEX_CONFIG1                                                        0x3a
typedef struct {
	uint8_t int_status_mask_pin_r2w_sleep_det                                      : 1;
	uint8_t int_status_mask_pin_smd_det                                            : 1;
	uint8_t int_status_mask_pin_selftest_done                                      : 1;
	uint8_t resv_1                                                                 : 1;
	uint8_t int_status_mask_pin_sa_done                                            : 1;
	uint8_t int_status_mask_pin_basic_smd                                          : 1;
	uint8_t resv_2                                                                 : 2;
} int_apex_config1_t;

#define INT_APEX_STATUS0                                                        0x3b
typedef struct {
	uint8_t int_status_tap_det                                                     : 1;
	uint8_t int_status_high_g_det                                                  : 1;
	uint8_t int_status_low_g_det                                                   : 1;
	uint8_t int_status_tilt_det                                                    : 1;
	uint8_t int_status_step_cnt_ovfl                                               : 1;
	uint8_t int_status_step_det                                                    : 1;
	uint8_t int_status_ff_det                                                      : 1;
	uint8_t int_status_r2w_wake_det                                                : 1;
} int_apex_status0_t;

#define INT_APEX_STATUS1                                                        0x3c
typedef struct {
	uint8_t int_status_r2w_sleep_det                                               : 1;
	uint8_t int_status_smd_det                                                     : 1;
	uint8_t int_status_selftest_done                                               : 1;
	uint8_t resv_1                                                                 : 1;
	uint8_t int_status_sa_done                                                     : 1;
	uint8_t int_status_basic_smd                                                   : 1;
	uint8_t resv_2                                                                 : 2;
} int_apex_status1_t;

#define INTF_CONFIG_OVRD_AUX1                                                   0x42
typedef struct {
	uint8_t aux1_spi_mode_ovrd_val                                                 : 1;
	uint8_t aux1_spi_mode_ovrd                                                     : 1;
	uint8_t aux1_spi_34_mode_ovrd_val                                              : 1;
	uint8_t aux1_spi_34_mode_ovrd                                                  : 1;
	uint8_t aux1_ireg_auto_addr_inc_dis                                            : 1;
	uint8_t resv_1                                                                 : 3;
} intf_config_ovrd_aux1_t;

#define ACCEL_DATA_X1_AUX1                                                      0x44
#define ACCEL_DATA_X0_AUX1                                                      0x45
#define ACCEL_DATA_Y1_AUX1                                                      0x46
#define ACCEL_DATA_Y0_AUX1                                                      0x47
#define ACCEL_DATA_Z1_AUX1                                                      0x48
#define ACCEL_DATA_Z0_AUX1                                                      0x49
#define GYRO_DATA_X1_AUX1                                                       0x4a
#define GYRO_DATA_X0_AUX1                                                       0x4b
#define GYRO_DATA_Y1_AUX1                                                       0x4c
#define GYRO_DATA_Y0_AUX1                                                       0x4d
#define GYRO_DATA_Z1_AUX1                                                       0x4e
#define GYRO_DATA_Z0_AUX1                                                       0x4f
#define TEMP_DATA1_AUX1                                                         0x50
#define TEMP_DATA0_AUX1                                                         0x51
#define TMST_FSYNCH_AUX1                                                        0x52
#define TMST_FSYNCL_AUX1                                                        0x53
#define PWR_MGMT_AUX1                                                           0x54
typedef struct {
	uint8_t accel_aux1_en                                                          : 1;
	uint8_t gyro_aux1_en                                                           : 1;
	uint8_t resv_1                                                                 : 6;
} pwr_mgmt_aux1_t;

#define FS_SEL_AUX1                                                             0x55
typedef struct {
	uint8_t accel_aux1_fs_sel                                                      : 3;
	uint8_t gyro_aux1_fs_sel                                                       : 4;
	uint8_t resv_1                                                                 : 1;
} fs_sel_aux1_t;

#define INT2_CONFIG0                                                            0x56
typedef struct {
	uint8_t int2_status_en_fifo_full                                               : 1;
	uint8_t int2_status_en_fifo_ths                                                : 1;
	uint8_t int2_status_en_drdy                                                    : 1;
	uint8_t int2_status_en_aux1_drdy                                               : 1;
	uint8_t int2_status_en_ap_fsync                                                : 1;
	uint8_t int2_status_en_ap_agc_rdy                                              : 1;
	uint8_t int2_status_en_aux1_agc_rdy                                            : 1;
	uint8_t int2_status_en_reset_done                                              : 1;
} int2_config0_t;

#define INT2_CONFIG1                                                            0x57
typedef struct {
	uint8_t int2_status_en_pll_rdy                                                 : 1;
	uint8_t int2_status_en_wom_x                                                   : 1;
	uint8_t int2_status_en_wom_y                                                   : 1;
	uint8_t int2_status_en_wom_z                                                   : 1;
	uint8_t int2_status_en_i3c_protocol_err                                        : 1;
	uint8_t int2_status_en_i2cm_done                                               : 1;
	uint8_t int2_status_en_apex_event                                              : 1;
	uint8_t resv_1                                                                 : 1;
} int2_config1_t;

#define INT2_CONFIG2                                                            0x58
typedef struct {
	uint8_t int2_polarity                                                          : 1;
	uint8_t int2_mode                                                              : 1;
	uint8_t int2_drive                                                             : 1;
	uint8_t resv_1                                                                 : 5;
} int2_config2_t;

#define INT2_STATUS0                                                            0x59
typedef struct {
	uint8_t int2_status_fifo_full                                                  : 1;
	uint8_t int2_status_fifo_ths                                                   : 1;
	uint8_t int2_status_drdy                                                       : 1;
	uint8_t int2_status_aux1_drdy                                                  : 1;
	uint8_t int2_status_ap_fsync                                                   : 1;
	uint8_t int2_status_ap_agc_rdy                                                 : 1;
	uint8_t int2_status_aux1_agc_rdy                                               : 1;
	uint8_t int2_status_reset_done                                                 : 1;
} int2_status0_t;

#define INT2_STATUS1                                                            0x5a
typedef struct {
	uint8_t int2_status_pll_rdy                                                    : 1;
	uint8_t int2_status_wom_x                                                      : 1;
	uint8_t int2_status_wom_y                                                      : 1;
	uint8_t int2_status_wom_z                                                      : 1;
	uint8_t int2_status_i3c_protocol_err                                           : 1;
	uint8_t int2_status_i2cm_done                                                  : 1;
	uint8_t int2_status_apex_event                                                 : 1;
	uint8_t resv_1                                                                 : 1;
} int2_status1_t;

#define INTF_CONFIG_OVRD_AUX2                                                   0x5c
typedef struct {
	uint8_t aux2_spi_mode_ovrd_val                                                 : 1;
	uint8_t aux2_spi_mode_ovrd                                                     : 1;
	uint8_t aux2_ireg_auto_addr_inc_dis                                            : 1;
	uint8_t resv_1                                                                 : 5;
} intf_config_ovrd_aux2_t;

#define ACCEL_DATA_X1_AUX2                                                      0x5e
#define ACCEL_DATA_X0_AUX2                                                      0x5f
#define ACCEL_DATA_Y1_AUX2                                                      0x60
#define ACCEL_DATA_Y0_AUX2                                                      0x61
#define ACCEL_DATA_Z1_AUX2                                                      0x62
#define ACCEL_DATA_Z0_AUX2                                                      0x63
#define GYRO_DATA_X1_AUX2                                                       0x64
#define GYRO_DATA_X0_AUX2                                                       0x65
#define GYRO_DATA_Y1_AUX2                                                       0x66
#define GYRO_DATA_Y0_AUX2                                                       0x67
#define GYRO_DATA_Z1_AUX2                                                       0x68
#define GYRO_DATA_Z0_AUX2                                                       0x69
#define TEMP_DATA1_AUX2                                                         0x6a
#define TEMP_DATA0_AUX2                                                         0x6b
#define TMST_FSYNCH_AUX2                                                        0x6c
#define TMST_FSYNCL_AUX2                                                        0x6d
#define PWR_MGMT_AUX2                                                           0x6e
typedef struct {
	uint8_t accel_aux2_en                                                          : 1;
	uint8_t gyro_aux2_en                                                           : 1;
	uint8_t resv_1                                                                 : 6;
} pwr_mgmt_aux2_t;

#define FS_SEL_AUX2                                                             0x6f
typedef struct {
	uint8_t accel_aux2_fs_sel                                                      : 3;
	uint8_t gyro_aux2_fs_sel                                                       : 4;
	uint8_t resv_1                                                                 : 1;
} fs_sel_aux2_t;

#define INT_AUX2_CONFIG                                                         0x70
typedef struct {
	uint8_t int_en_aux2_agc_rdy                                                    : 1;
	uint8_t int_en_aux2_reset_done                                                 : 1;
	uint8_t int_en_aux2_pll_rdy                                                    : 1;
	uint8_t int_en_aux2_drdy                                                       : 1;
	uint8_t resv_1                                                                 : 4;
} int_aux2_config_t;

#define INT_AUX2_STATUS                                                         0x71
typedef struct {
	uint8_t int_status_aux2_agc_rdy                                                : 1;
	uint8_t int_status_aux2_reset_done                                             : 1;
	uint8_t int_status_aux2_pll_rdy                                                : 1;
	uint8_t int_status_aux2_drdy                                                   : 1;
	uint8_t resv_1                                                                 : 4;
} int_aux2_status_t;

#define WHO_AM_I                                                                0x72
typedef struct {
	uint8_t who_am_i                                                               : 8;
} who_am_i_t;

#define REG_HOST_MSG                                                            0x73
typedef struct {
	uint8_t testopenable                                                           : 1;
	uint8_t resv_1                                                                 : 4;
	uint8_t edmp_on_demand_en                                                      : 1;
	uint8_t resv_2                                                                 : 2;
} reg_host_msg_t;

#define IREG_ADDR_15_8                                                          0x7c
typedef struct {
	uint8_t ireg_addr_15_8                                                         : 8;
} ireg_addr_15_8_t;

#define IREG_ADDR_7_0                                                           0x7d
typedef struct {
	uint8_t ireg_addr_7_0                                                          : 8;
} ireg_addr_7_0_t;

#define IREG_DATA                                                               0x7e
typedef struct {
	uint8_t ireg_data                                                              : 8;
} ireg_data_t;

#define REG_MISC2                                                               0x7f
typedef struct {
	uint8_t ireg_done                                                              : 1;
	uint8_t soft_rst                                                               : 1;
	uint8_t resv_1                                                                 : 6;
} reg_misc2_t;


/* DREG_BANK2 */

/* IPREG_TOP1 */
#define I2CM_COMMAND_0                                                          0xa206
typedef struct {
	uint8_t burstlen_0                                                             : 4;
	uint8_t r_w_0                                                                  : 2;
	uint8_t ch_sel_0                                                               : 1;
	uint8_t endflag_0                                                              : 1;
} i2cm_command_0_t;

#define I2CM_COMMAND_1                                                          0xa207
typedef struct {
	uint8_t burstlen_1                                                             : 4;
	uint8_t r_w_1                                                                  : 2;
	uint8_t ch_sel_1                                                               : 1;
	uint8_t endflag_1                                                              : 1;
} i2cm_command_1_t;

#define I2CM_COMMAND_2                                                          0xa208
typedef struct {
	uint8_t burstlen_2                                                             : 4;
	uint8_t r_w_2                                                                  : 2;
	uint8_t ch_sel_2                                                               : 1;
	uint8_t endflag_2                                                              : 1;
} i2cm_command_2_t;

#define I2CM_COMMAND_3                                                          0xa209
typedef struct {
	uint8_t burstlen_3                                                             : 4;
	uint8_t r_w_3                                                                  : 2;
	uint8_t ch_sel_3                                                               : 1;
	uint8_t endflag_3                                                              : 1;
} i2cm_command_3_t;

#define I2CM_DEV_PROFILE0                                                       0xa20e
typedef struct {
	uint8_t rd_address_0                                                           : 8;
} i2cm_dev_profile0_t;

#define I2CM_DEV_PROFILE1                                                       0xa20f
typedef struct {
	uint8_t dev_id_0                                                               : 7;
	uint8_t resv_1                                                                 : 1;
} i2cm_dev_profile1_t;

#define I2CM_DEV_PROFILE2                                                       0xa210
typedef struct {
	uint8_t rd_address_1                                                           : 8;
} i2cm_dev_profile2_t;

#define I2CM_DEV_PROFILE3                                                       0xa211
typedef struct {
	uint8_t dev_id_1                                                               : 7;
	uint8_t resv_1                                                                 : 1;
} i2cm_dev_profile3_t;

#define I2CM_CONTROL                                                            0xa216
typedef struct {
	uint8_t i2cm_go                                                                : 1;
	uint8_t resv_1                                                                 : 2;
	uint8_t i2cm_speed                                                             : 1;
	uint8_t resv_2                                                                 : 2;
	uint8_t i2cm_restart_en                                                        : 1;
	uint8_t resv_3                                                                 : 1;
} i2cm_control_t;

#define I2CM_STATUS                                                             0xa218
typedef struct {
	uint8_t i2cm_busy                                                              : 1;
	uint8_t i2cm_done                                                              : 1;
	uint8_t i2cm_timeout_err                                                       : 1;
	uint8_t i2cm_srst_err                                                          : 1;
	uint8_t i2cm_scl_err                                                           : 1;
	uint8_t i2cm_sda_err                                                           : 1;
	uint8_t resv_1                                                                 : 2;
} i2cm_status_t;

#define I2CM_EXT_DEV_STATUS                                                     0xa21a
typedef struct {
	uint8_t i2cm_ext_dev_status                                                    : 4;
	uint8_t resv_1                                                                 : 4;
} i2cm_ext_dev_status_t;

#define I2CM_RD_DATA0                                                           0xa21b
typedef struct {
	uint8_t i2cm_rd_data0                                                          : 8;
} i2cm_rd_data0_t;

#define I2CM_RD_DATA1                                                           0xa21c
typedef struct {
	uint8_t i2cm_rd_data1                                                          : 8;
} i2cm_rd_data1_t;

#define I2CM_RD_DATA2                                                           0xa21d
typedef struct {
	uint8_t i2cm_rd_data2                                                          : 8;
} i2cm_rd_data2_t;

#define I2CM_RD_DATA3                                                           0xa21e
typedef struct {
	uint8_t i2cm_rd_data3                                                          : 8;
} i2cm_rd_data3_t;

#define I2CM_RD_DATA4                                                           0xa21f
typedef struct {
	uint8_t i2cm_rd_data4                                                          : 8;
} i2cm_rd_data4_t;

#define I2CM_RD_DATA5                                                           0xa220
typedef struct {
	uint8_t i2cm_rd_data5                                                          : 8;
} i2cm_rd_data5_t;

#define I2CM_RD_DATA6                                                           0xa221
typedef struct {
	uint8_t i2cm_rd_data6                                                          : 8;
} i2cm_rd_data6_t;

#define I2CM_RD_DATA7                                                           0xa222
typedef struct {
	uint8_t i2cm_rd_data7                                                          : 8;
} i2cm_rd_data7_t;

#define I2CM_RD_DATA8                                                           0xa223
typedef struct {
	uint8_t i2cm_rd_data8                                                          : 8;
} i2cm_rd_data8_t;

#define I2CM_RD_DATA9                                                           0xa224
typedef struct {
	uint8_t i2cm_rd_data9                                                          : 8;
} i2cm_rd_data9_t;

#define I2CM_RD_DATA10                                                          0xa225
typedef struct {
	uint8_t i2cm_rd_data10                                                         : 8;
} i2cm_rd_data10_t;

#define I2CM_RD_DATA11                                                          0xa226
typedef struct {
	uint8_t i2cm_rd_data11                                                         : 8;
} i2cm_rd_data11_t;

#define I2CM_RD_DATA12                                                          0xa227
typedef struct {
	uint8_t i2cm_rd_data12                                                         : 8;
} i2cm_rd_data12_t;

#define I2CM_RD_DATA13                                                          0xa228
typedef struct {
	uint8_t i2cm_rd_data13                                                         : 8;
} i2cm_rd_data13_t;

#define I2CM_RD_DATA14                                                          0xa229
typedef struct {
	uint8_t i2cm_rd_data14                                                         : 8;
} i2cm_rd_data14_t;

#define I2CM_RD_DATA15                                                          0xa22a
typedef struct {
	uint8_t i2cm_rd_data15                                                         : 8;
} i2cm_rd_data15_t;

#define I2CM_RD_DATA16                                                          0xa22b
typedef struct {
	uint8_t i2cm_rd_data16                                                         : 8;
} i2cm_rd_data16_t;

#define I2CM_RD_DATA17                                                          0xa22c
typedef struct {
	uint8_t i2cm_rd_data17                                                         : 8;
} i2cm_rd_data17_t;

#define I2CM_RD_DATA18                                                          0xa22d
typedef struct {
	uint8_t i2cm_rd_data18                                                         : 8;
} i2cm_rd_data18_t;

#define I2CM_RD_DATA19                                                          0xa22e
typedef struct {
	uint8_t i2cm_rd_data19                                                         : 8;
} i2cm_rd_data19_t;

#define I2CM_RD_DATA20                                                          0xa22f
typedef struct {
	uint8_t i2cm_rd_data20                                                         : 8;
} i2cm_rd_data20_t;

#define I2CM_WR_DATA0                                                           0xa233
typedef struct {
	uint8_t i2cm_wr_data0                                                          : 8;
} i2cm_wr_data0_t;

#define I2CM_WR_DATA1                                                           0xa234
typedef struct {
	uint8_t i2cm_wr_data1                                                          : 8;
} i2cm_wr_data1_t;

#define I2CM_WR_DATA2                                                           0xa235
typedef struct {
	uint8_t i2cm_wr_data2                                                          : 8;
} i2cm_wr_data2_t;

#define I2CM_WR_DATA3                                                           0xa236
typedef struct {
	uint8_t i2cm_wr_data3                                                          : 8;
} i2cm_wr_data3_t;

#define I2CM_WR_DATA4                                                           0xa237
typedef struct {
	uint8_t i2cm_wr_data4                                                          : 8;
} i2cm_wr_data4_t;

#define I2CM_WR_DATA5                                                           0xa238
typedef struct {
	uint8_t i2cm_wr_data5                                                          : 8;
} i2cm_wr_data5_t;

#define SIFS_IXC_ERROR_STATUS                                                   0xa24b
typedef struct {
	uint8_t sifs_ixc_timeout_err                                                   : 1;
	uint8_t aux1_sifs_ixc_timeout_err                                              : 1;
	uint8_t resv_1                                                                 : 6;
} sifs_ixc_error_status_t;

#define EDMP_PRGRM_IRQ0_0                                                       0xa24f
#define EDMP_PRGRM_IRQ0_1                                                       0xa250
#define EDMP_PRGRM_IRQ1_0                                                       0xa251
#define EDMP_PRGRM_IRQ1_1                                                       0xa252
#define EDMP_PRGRM_IRQ2_0                                                       0xa253
#define EDMP_PRGRM_IRQ2_1                                                       0xa254
#define EDMP_SP_START_ADDR                                                      0xa255
typedef struct {
	uint8_t edmp_sp_start_addr                                                     : 8;
} edmp_sp_start_addr_t;

#define SMC_CONTROL_0                                                           0xa258
typedef struct {
	uint8_t tmst_en                                                                : 1;
	uint8_t tmst_fsync_en                                                          : 1;
	uint8_t tmst_force_aux_fine_en                                                 : 1;
	uint8_t temp_dis                                                               : 1;
	uint8_t accel_lp_clk_sel                                                       : 1;
	uint8_t resv_1                                                                 : 3;
} smc_control_0_t;

#define SMC_CONTROL_1                                                           0xa259
typedef struct {
	uint8_t resv_1                                                                 : 3;
	uint8_t sreg_aux_accel_only_en                                                 : 1;
	uint8_t resv_2                                                                 : 4;
} smc_control_1_t;

#define STC_CONFIG                                                              0xa263
typedef struct {
	uint8_t resv_1                                                                 : 2;
	uint8_t stc_sensor_sel                                                         : 2;
	uint8_t resv_2                                                                 : 4;
} stc_config_t;

#define SREG_CTRL                                                               0xa267
typedef struct {
	uint8_t resv_1                                                                 : 1;
	uint8_t sreg_data_endian_sel                                                   : 1;
	uint8_t resv_2                                                                 : 6;
} sreg_ctrl_t;

#define SIFS_I3C_STC_CFG                                                        0xa268
typedef struct {
	uint8_t resv_1                                                                 : 2;
	uint8_t i3c_stc_mode                                                           : 1;
	uint8_t resv_2                                                                 : 5;
} sifs_i3c_stc_cfg_t;

#define INT_PULSE_MIN_ON_INTF0                                                  0xa269
typedef struct {
	uint8_t int0_tpulse_duration                                                   : 3;
	uint8_t resv_1                                                                 : 5;
} int_pulse_min_on_intf0_t;

#define INT_PULSE_MIN_ON_INTF1                                                  0xa26a
typedef struct {
	uint8_t int1_tpulse_duration                                                   : 3;
	uint8_t resv_1                                                                 : 5;
} int_pulse_min_on_intf1_t;

#define INT_PULSE_MIN_OFF_INTF0                                                 0xa26b
typedef struct {
	uint8_t int0_tdeassert_disable                                                 : 3;
	uint8_t resv_1                                                                 : 5;
} int_pulse_min_off_intf0_t;

#define INT_PULSE_MIN_OFF_INTF1                                                 0xa26c
typedef struct {
	uint8_t int1_tdeassert_disable                                                 : 3;
	uint8_t resv_1                                                                 : 5;
} int_pulse_min_off_intf1_t;

#define ISR_0_7                                                                 0xa26e
typedef struct {
	uint8_t int_status_accel_drdy_pin_0                                            : 1;
	uint8_t resv_1                                                                 : 2;
	uint8_t int_status_ext_odr_drdy_pin_0                                          : 1;
	uint8_t resv_2                                                                 : 1;
	uint8_t int_status_on_demand_pin_0                                             : 1;
	uint8_t resv_3                                                                 : 2;
} isr_0_7_t;

#define ISR_8_15                                                                0xa26f
typedef struct {
	uint8_t int_status_accel_drdy_pin_1                                            : 1;
	uint8_t resv_1                                                                 : 2;
	uint8_t int_status_ext_odr_drdy_pin_1                                          : 1;
	uint8_t resv_2                                                                 : 1;
	uint8_t int_status_on_demand_pin_1                                             : 1;
	uint8_t resv_3                                                                 : 2;
} isr_8_15_t;

#define ISR_16_23                                                               0xa270
typedef struct {
	uint8_t int_status_accel_drdy_pin_2                                            : 1;
	uint8_t resv_1                                                                 : 2;
	uint8_t int_status_ext_odr_drdy_pin_2                                          : 1;
	uint8_t resv_2                                                                 : 1;
	uint8_t int_status_on_demand_pin_2                                             : 1;
	uint8_t resv_3                                                                 : 2;
} isr_16_23_t;

#define STATUS_MASK_PIN_0_7                                                     0xa271
typedef struct {
	uint8_t int_accel_drdy_pin_0_dis                                               : 1;
	uint8_t resv_1                                                                 : 2;
	uint8_t int_ext_odr_drdy_pin_0_dis                                             : 1;
	uint8_t resv_2                                                                 : 1;
	uint8_t int_on_demand_pin_0_dis                                                : 1;
	uint8_t resv_3                                                                 : 2;
} status_mask_pin_0_7_t;

#define STATUS_MASK_PIN_8_15                                                    0xa272
typedef struct {
	uint8_t int_accel_drdy_pin_1_dis                                               : 1;
	uint8_t resv_1                                                                 : 2;
	uint8_t int_ext_odr_drdy_pin_1_dis                                             : 1;
	uint8_t resv_2                                                                 : 1;
	uint8_t int_on_demand_pin_1_dis                                                : 1;
	uint8_t resv_3                                                                 : 2;
} status_mask_pin_8_15_t;

#define STATUS_MASK_PIN_16_23                                                   0xa273
typedef struct {
	uint8_t int_accel_drdy_pin_2_dis                                               : 1;
	uint8_t resv_1                                                                 : 2;
	uint8_t int_ext_odr_drdy_pin_2_dis                                             : 1;
	uint8_t resv_2                                                                 : 1;
	uint8_t int_on_demand_pin_2_dis                                                : 1;
	uint8_t resv_3                                                                 : 2;
} status_mask_pin_16_23_t;

#define INT_I2CM_SOURCE                                                         0xa274
typedef struct {
	uint8_t int_status_i2cm_ioc_ext_trig_en                                        : 1;
	uint8_t int_status_i2cm_smc_ext_odr_en                                         : 1;
	uint8_t resv_1                                                                 : 6;
} int_i2cm_source_t;

#define ACCEL_WOM_X_THR                                                         0xa27e
typedef struct {
	uint8_t wom_x_th                                                               : 8;
} accel_wom_x_thr_t;

#define ACCEL_WOM_Y_THR                                                         0xa27f
typedef struct {
	uint8_t wom_y_th                                                               : 8;
} accel_wom_y_thr_t;

#define ACCEL_WOM_Z_THR                                                         0xa280
typedef struct {
	uint8_t wom_z_th                                                               : 8;
} accel_wom_z_thr_t;

#define SELFTEST                                                                0xa290
typedef struct {
	uint8_t en_ax_st                                                               : 1;
	uint8_t en_ay_st                                                               : 1;
	uint8_t en_az_st                                                               : 1;
	uint8_t en_gx_st                                                               : 1;
	uint8_t en_gy_st                                                               : 1;
	uint8_t en_gz_st                                                               : 1;
	uint8_t resv_1                                                                 : 2;
} selftest_t;

#define IPREG_MISC                                                              0xa297
typedef struct {
	uint8_t resv_1                                                                 : 1;
	uint8_t edmp_idle                                                              : 1;
	uint8_t resv_2                                                                 : 6;
} ipreg_misc_t;

#define SW_PLL1_TRIM                                                            0xa2a2
typedef struct {
	uint8_t sw_pll1_trim                                                           : 8;
} sw_pll1_trim_t;

#define FIFO_SRAM_SLEEP                                                         0xa2a7
typedef struct {
	uint8_t fifo_gsleep_shared_sram                                                : 2;
	uint8_t resv_1                                                                 : 6;
} fifo_sram_sleep_t;


/* IPREG_SYS1 */
#define IPREG_SYS1_REG_42                                                       0xa42a
#define IPREG_SYS1_REG_43                                                       0xa42b
#define IPREG_SYS1_REG_56                                                       0xa438
#define IPREG_SYS1_REG_57                                                       0xa439
#define IPREG_SYS1_REG_70                                                       0xa446
#define IPREG_SYS1_REG_71                                                       0xa447
#define IPREG_SYS1_REG_166                                                      0xa4a6
typedef struct {
	uint8_t resv_1                                                                 : 3;
	uint8_t gyro_afsr_mode                                                         : 2;
	uint8_t gyro_src_ctrl                                                          : 2;
	uint8_t resv_2                                                                 : 1;
} ipreg_sys1_reg_166_t;

#define IPREG_SYS1_REG_168                                                      0xa4a8
typedef struct {
	uint8_t gyro_afsr_shared                                                       : 1;
	uint8_t gyro_ois_m6_byp                                                        : 1;
	uint8_t resv_1                                                                 : 6;
} ipreg_sys1_reg_168_t;

#define IPREG_SYS1_REG_170                                                      0xa4aa
typedef struct {
	uint8_t resv_1                                                                 : 1;
	uint8_t gyro_lp_avg_sel                                                        : 4;
	uint8_t gyro_ois_hpfbw_sel                                                     : 3;
} ipreg_sys1_reg_170_t;

#define IPREG_SYS1_REG_171                                                      0xa4ab
typedef struct {
	uint8_t gyro_ois_lpf1bw_sel                                                    : 3;
	uint8_t gyro_ois_lpf2bw_sel                                                    : 3;
	uint8_t resv_1                                                                 : 2;
} ipreg_sys1_reg_171_t;

#define IPREG_SYS1_REG_172                                                      0xa4ac
typedef struct {
	uint8_t gyro_ui_lpfbw_sel                                                      : 3;
	uint8_t resv_1                                                                 : 4;
	uint8_t gyro_ois_hpf1_byp                                                      : 1;
} ipreg_sys1_reg_172_t;

#define IPREG_SYS1_REG_173                                                      0xa4ad
typedef struct {
	uint8_t gyro_ois_hpf2_byp                                                      : 1;
	uint8_t resv_1                                                                 : 7;
} ipreg_sys1_reg_173_t;


/* IPREG_SYS2 */
#define IPREG_SYS2_REG_24                                                       0xa518
#define IPREG_SYS2_REG_25                                                       0xa519
#define IPREG_SYS2_REG_32                                                       0xa520
#define IPREG_SYS2_REG_33                                                       0xa521
#define IPREG_SYS2_REG_40                                                       0xa528
#define IPREG_SYS2_REG_41                                                       0xa529
#define IPREG_SYS2_REG_123                                                      0xa57b
typedef struct {
	uint8_t accel_src_ctrl                                                         : 2;
	uint8_t resv_1                                                                 : 6;
} ipreg_sys2_reg_123_t;

#define IPREG_SYS2_REG_129                                                      0xa581
typedef struct {
	uint8_t accel_lp_avg_sel                                                       : 4;
	uint8_t accel_ois_hpfbw_sel                                                    : 3;
	uint8_t resv_1                                                                 : 1;
} ipreg_sys2_reg_129_t;

#define IPREG_SYS2_REG_130                                                      0xa582
typedef struct {
	uint8_t accel_ois_lpf1bw_sel                                                   : 3;
	uint8_t accel_ois_lpf2bw_sel                                                   : 3;
	uint8_t resv_1                                                                 : 2;
} ipreg_sys2_reg_130_t;

#define IPREG_SYS2_REG_131                                                      0xa583
typedef struct {
	uint8_t accel_ui_lpfbw_sel                                                     : 3;
	uint8_t resv_1                                                                 : 5;
} ipreg_sys2_reg_131_t;

#define IPREG_SYS2_REG_132                                                      0xa584
typedef struct {
	uint8_t accel_ois_hpf1_byp                                                     : 1;
	uint8_t accel_ois_hpf2_byp                                                     : 1;
	uint8_t accel_ois_m6_byp                                                       : 1;
	uint8_t resv_1                                                                 : 5;
} ipreg_sys2_reg_132_t;


/* IPREG_OTP */

/* IPREG_BAR */
#define IPREG_BAR_REG_57                                                        0xa039
typedef struct {
	uint8_t resv_1                                                                 : 5;
	uint8_t io_opt1                                                                : 1;
	uint8_t io_opt0                                                                : 1;
	uint8_t resv_2                                                                 : 1;
} ipreg_bar_reg_57_t;

#define IPREG_BAR_REG_58                                                        0xa03a
typedef struct {
	uint8_t io_opt2                                                                : 1;
	uint8_t resv_1                                                                 : 2;
	uint8_t pads_ap_cs_pe_trim_d2a                                                 : 1;
	uint8_t pads_ap_cs_pud_trim_d2a                                                : 1;
	uint8_t resv_2                                                                 : 1;
	uint8_t pads_ap_sclk_pe_trim_d2a                                               : 1;
	uint8_t pads_ap_sclk_pud_trim_d2a                                              : 1;
} ipreg_bar_reg_58_t;

#define IPREG_BAR_REG_59                                                        0xa03b
typedef struct {
	uint8_t resv_1                                                                 : 1;
	uint8_t pads_ap_sdi_pe_trim_d2a                                                : 1;
	uint8_t pads_ap_sdi_pud_trim_d2a                                               : 1;
	uint8_t resv_2                                                                 : 1;
	uint8_t pads_ap_sdo_pe_trim_d2a                                                : 1;
	uint8_t pads_ap_sdo_pud_trim_d2a                                               : 1;
	uint8_t resv_3                                                                 : 1;
	uint8_t pads_pin7_pe_trim_d2a                                                  : 1;
} ipreg_bar_reg_59_t;

#define IPREG_BAR_REG_60                                                        0xa03c
typedef struct {
	uint8_t pads_pin7_cs_pud_trim_d2a                                              : 1;
	uint8_t resv_1                                                                 : 1;
	uint8_t pads_aux1_cs_pe_trim_d2a                                               : 1;
	uint8_t pads_aux1_cs_pud_trim_d2a                                              : 1;
	uint8_t pads_aux_sclk_tp2_from_pad_disable_trim_d2a                            : 1;
	uint8_t pads_aux1_sclk_pe_trim_d2a                                             : 1;
	uint8_t pads_aux1_sclk_pud_trim_d2a                                            : 1;
	uint8_t resv_2                                                                 : 1;
} ipreg_bar_reg_60_t;

#define IPREG_BAR_REG_61                                                        0xa03d
typedef struct {
	uint8_t pads_aux1_sdi_pe_trim_d2a                                              : 1;
	uint8_t pads_aux1_sdi_pud_trim_d2a                                             : 1;
	uint8_t resv_1                                                                 : 1;
	uint8_t pads_aux1_sdo_pe_trim_d2a                                              : 1;
	uint8_t pads_aux1_sdo_pud_trim_d2a                                             : 1;
	uint8_t resv_2                                                                 : 1;
	uint8_t pads_int1_pe_trim_d2a                                                  : 1;
	uint8_t pads_int1_pud_trim_d2a                                                 : 1;
} ipreg_bar_reg_61_t;

#define IPREG_BAR_REG_62                                                        0xa03e
typedef struct {
	uint8_t resv_1                                                                 : 1;
	uint8_t pads_int2_pe_trim_d2a                                                  : 1;
	uint8_t pads_int2_pud_trim_d2a                                                 : 1;
	uint8_t resv_2                                                                 : 5;
} ipreg_bar_reg_62_t;



#ifdef __cplusplus
}
#endif

#endif  /*#ifndef _INV_IMU_REGMAP_LE_H_*/
