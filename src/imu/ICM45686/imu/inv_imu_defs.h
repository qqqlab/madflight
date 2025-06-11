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

#ifndef _INV_IMU_DEFS_H_
#define _INV_IMU_DEFS_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup Defs Defs
 *  @brief Registers and driver-related definitions and descriptions
 *  @{
 */

/** @file inv_imu_defs.h */

#include <stdint.h>

/* Include device definition */
#include "./inv_imu.h"

/* Include regmap (le = little endian, be = big endian) */
#include "./inv_imu_regmap_le.h"
/* #include "imu/inv_imu_regmap_be.h" */

/* Error/Success codes */
#define INV_IMU_OK                   0 /**< Success */
#define INV_IMU_ERROR                -1 /**< Unspecified error */
#define INV_IMU_ERROR_TRANSPORT      -3 /**< Error occurred at transport level */
#define INV_IMU_ERROR_TIMEOUT        -4 /**< Action did not complete in the expected time window */
#define INV_IMU_ERROR_BAD_ARG        -11 /**< Invalid argument provided */
#define INV_IMU_ERROR_EDMP_ODR       -126 /**< EDMP ODR decimator reconfiguration is needed */
#define INV_IMU_ERROR_EDMP_BUF_EMPTY -127 /**< EDMP buffer is empty */

/* Enable and Disable state */
#define INV_IMU_DISABLE (0U)
#define INV_IMU_ENABLE  (1U)

/* Startup times */
#define ACC_STARTUP_TIME_US 10000
#define GYR_STARTUP_TIME_US 70000

/* FIFO: Data size */
#define ACCEL_DATA_SIZE               6
#define GYRO_DATA_SIZE                6
#define TEMP_DATA_SIZE                2
#define FIFO_HEADER_SIZE              1
#define FIFO_TEMP_DATA_SIZE           1
#define FIFO_TS_FSYNC_SIZE            2
#define FIFO_TEMP_HIGH_RES_SIZE       1
#define FIFO_ACCEL_GYRO_HIGH_RES_SIZE 3
#define FIFO_ES0_6B_DATA_SIZE         6
#define FIFO_ES0_9B_DATA_SIZE         9
#define FIFO_ES1_DATA_SIZE            6

/* FIFO: Special values */
#define INVALID_VALUE_FIFO            ((int16_t)0x8000)
#define INVALID_VALUE_FIFO_1B         ((int8_t)0x80)
#define OUT_OF_BOUND_TEMP_NEG_FIFO_1B ((int8_t)0x81)
#define OUT_OF_BOUND_TEMP_POS_FIFO_1B ((int8_t)0x7F)

/* FIFO: Compression ratio */
#define FIFO_COMP_X2_COMPRESSION 0
#define FIFO_COMP_X3_COMPRESSION 1
#define FIFO_COMP_X4_COMPRESSION 2

/* FIFO: Total number of sample in the frame */
#define FIFO_COMP_1_SAMPLE_IN_FRAME  0
#define FIFO_COMP_2_SAMPLES_IN_FRAME 1
#define FIFO_COMP_3_SAMPLES_IN_FRAME 2
#define FIFO_COMP_4_SAMPLES_IN_FRAME 3

/** Converts an integer from a 5-bits signed to a 8-bits signed */
#define INT5_TO_INT8(in) (((in) < 16) ? ((int8_t)(in)) : ((int8_t)(in)-32))

/** Converts an integer from a 4-bits signed to a 8-bits signed */
#define INT4_TO_INT8(in) (((in) < 8) ? ((int8_t)(in)) : ((int8_t)(in)-16))

/** Sensor data from registers */
typedef struct {
	int16_t accel_data[3];
	int16_t gyro_data[3];
	int16_t temp_data;
} inv_imu_sensor_data_t;

/** Interrupt number */
typedef enum {
	INV_IMU_INT1,
	INV_IMU_INT2,
} inv_imu_int_num_t;

/** Describe the content of the FIFO header */
typedef union {
	uint8_t Byte;
	struct {
		uint8_t gyro_odr_different : 1;
		uint8_t accel_odr_different : 1;
		uint8_t fsync_bit : 1;
		uint8_t timestamp_bit : 1;
		uint8_t twentybits_bit : 1;
		uint8_t gyro_bit : 1;
		uint8_t accel_bit : 1;
		uint8_t ext_header : 1;
	} bits;
} fifo_header_t;

/** Describe the content of the FIFO header for compressed packets */
typedef union {
	uint8_t Byte;
	struct {
		uint8_t tot_sample : 2;
		uint8_t comp_ratio : 2;
		uint8_t comp_frame : 1;
		uint8_t gyro_bit : 1;
		uint8_t accel_bit : 1;
		uint8_t ext_header : 1;
	} bits;
} fifo_comp_header_t;

/** Describe the content of the second FIFO header */
typedef union {
	uint8_t Byte;
	struct {
		uint8_t es0_en : 1;
		uint8_t es1_en : 1;
		uint8_t es0_vld : 1;
		uint8_t es1_vld : 1;
		uint8_t es0_6b_9b : 1;
		uint8_t unused1 : 1;
		uint8_t unused2 : 1;
		uint8_t unused3 : 1;
	} bits;
} fifo_header2_t;

/** Describe the content of the FIFO Compression Decoding Tag */
typedef union {
	uint8_t Byte;
	struct {
		uint8_t valid_samples_a : 4;
		uint8_t valid_samples_g : 4;
	} bits;
} fifo_comp_decode_t;

/** Required registers to configure FIFO */
typedef struct {
	fifo_config0_t fifo_config0;
	uint8_t        fifo_config1_0;
	uint8_t        fifo_config1_1;
	fifo_config2_t fifo_config2;
	fifo_config3_t fifo_config3;
	fifo_config4_t fifo_config4;
} fifo_configx_t;

/** Required registers to configure interrupts.
 *  This structure applies to INT1 and INT2 as bit location are the same.
 */
typedef struct {
	int1_config0_t int1_config0;
	int1_config1_t int1_config1;
} intx_configx_t;

/** Registers to retrieve interrupts status.
 *  This structure applies to INT1 and INT2 as bit location are the same.
 */
typedef struct {
	int1_status0_t int1_status0;
	int1_status1_t int1_status1;
} intx_statusx_t;

/*
 * Registers description 
 */

/* ---------------------------------------------------------------------------
 * Bank DREG_BANK1
 * ---------------------------------------------------------------------------*/

/*
 * PWR_MGMT0
 */

/* gyro_mode */
typedef enum {
	PWR_MGMT0_GYRO_MODE_LN      = 0x03,
	PWR_MGMT0_GYRO_MODE_LP      = 0x02,
	PWR_MGMT0_GYRO_MODE_STANDBY = 0x01,
	PWR_MGMT0_GYRO_MODE_OFF     = 0x00,
} pwr_mgmt0_gyro_mode_t;

/* accel_mode */
typedef enum {
	PWR_MGMT0_ACCEL_MODE_LN  = 0x03,
	PWR_MGMT0_ACCEL_MODE_LP  = 0x02,
	PWR_MGMT0_ACCEL_MODE_OFF = 0x00,
} pwr_mgmt0_accel_mode_t;

/*
 * INTX_CONFIG2
 * Applies to both INT1_CONFIG2 and INT2_CONFIG2 (bits are located at the 
 * same position on two different registers)
 */

/* intX_drive */
typedef enum {
	INTX_CONFIG2_INTX_DRIVE_PP = 0x00,
	INTX_CONFIG2_INTX_DRIVE_OD = 0x01,
} intx_config2_intx_drive_t;

/* intX_mode */
typedef enum {
	INTX_CONFIG2_INTX_MODE_PULSE = 0x00,
	INTX_CONFIG2_INTX_MODE_LATCH = 0x01,
} intx_config2_intx_mode_t;

/* intX_polarity */
typedef enum {
	INTX_CONFIG2_INTX_POLARITY_LOW  = 0x00,
	INTX_CONFIG2_INTX_POLARITY_HIGH = 0x01,
} intx_config2_intx_polarity_t;

/** @brief Interrupts pin configuration */
typedef struct {
	intx_config2_intx_polarity_t int_polarity;
	intx_config2_intx_mode_t     int_mode;
	intx_config2_intx_drive_t    int_drive;
} inv_imu_int_pin_config_t;

/*
 * ACCEL_CONFIG0
 */

/* accel_ui_fs_sel */
typedef enum {
	ACCEL_CONFIG0_ACCEL_UI_FS_SEL_2_G  = 0x4,
	ACCEL_CONFIG0_ACCEL_UI_FS_SEL_4_G  = 0x3,
	ACCEL_CONFIG0_ACCEL_UI_FS_SEL_8_G  = 0x2,
	ACCEL_CONFIG0_ACCEL_UI_FS_SEL_16_G = 0x1,
#if INV_IMU_HIGH_FSR_SUPPORTED
	ACCEL_CONFIG0_ACCEL_UI_FS_SEL_32_G = 0x0,
#endif
} accel_config0_accel_ui_fs_sel_t;

/* accel_odr */
typedef enum {
	ACCEL_CONFIG0_ACCEL_ODR_1_5625_HZ = 0xF,
	ACCEL_CONFIG0_ACCEL_ODR_3_125_HZ  = 0xE,
	ACCEL_CONFIG0_ACCEL_ODR_6_25_HZ   = 0xD,
	ACCEL_CONFIG0_ACCEL_ODR_12_5_HZ   = 0xC,
	ACCEL_CONFIG0_ACCEL_ODR_25_HZ     = 0xB,
	ACCEL_CONFIG0_ACCEL_ODR_50_HZ     = 0xA,
	ACCEL_CONFIG0_ACCEL_ODR_100_HZ    = 0x9,
	ACCEL_CONFIG0_ACCEL_ODR_200_HZ    = 0x8,
	ACCEL_CONFIG0_ACCEL_ODR_400_HZ    = 0x7,
	ACCEL_CONFIG0_ACCEL_ODR_800_HZ    = 0x6,
	ACCEL_CONFIG0_ACCEL_ODR_1600_HZ   = 0x5,
	ACCEL_CONFIG0_ACCEL_ODR_3200_HZ   = 0x4,
	ACCEL_CONFIG0_ACCEL_ODR_6400_HZ   = 0x3,
} accel_config0_accel_odr_t;

/*
 * GYRO_CONFIG0
 */

/* gyro_ui_fs_sel */
typedef enum {
	GYRO_CONFIG0_GYRO_UI_FS_SEL_15_625_DPS = 8,
	GYRO_CONFIG0_GYRO_UI_FS_SEL_31_25_DPS  = 7,
	GYRO_CONFIG0_GYRO_UI_FS_SEL_62_5_DPS   = 6,
	GYRO_CONFIG0_GYRO_UI_FS_SEL_125_DPS    = 5,
	GYRO_CONFIG0_GYRO_UI_FS_SEL_250_DPS    = 4,
	GYRO_CONFIG0_GYRO_UI_FS_SEL_500_DPS    = 3,
	GYRO_CONFIG0_GYRO_UI_FS_SEL_1000_DPS   = 2,
	GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS   = 1,
#if INV_IMU_HIGH_FSR_SUPPORTED
	GYRO_CONFIG0_GYRO_UI_FS_SEL_4000_DPS = 0,
#endif
} gyro_config0_gyro_ui_fs_sel_t;

/* gyro_odr */
typedef enum {
	GYRO_CONFIG0_GYRO_ODR_1_5625_HZ = 0xF,
	GYRO_CONFIG0_GYRO_ODR_3_125_HZ  = 0xE,
	GYRO_CONFIG0_GYRO_ODR_6_25_HZ   = 0xD,
	GYRO_CONFIG0_GYRO_ODR_12_5_HZ   = 0xC,
	GYRO_CONFIG0_GYRO_ODR_25_HZ     = 0xB,
	GYRO_CONFIG0_GYRO_ODR_50_HZ     = 0xA,
	GYRO_CONFIG0_GYRO_ODR_100_HZ    = 0x9,
	GYRO_CONFIG0_GYRO_ODR_200_HZ    = 0x8,
	GYRO_CONFIG0_GYRO_ODR_400_HZ    = 0x7,
	GYRO_CONFIG0_GYRO_ODR_800_HZ    = 0x6,
	GYRO_CONFIG0_GYRO_ODR_1600_HZ   = 0x5,
	GYRO_CONFIG0_GYRO_ODR_3200_HZ   = 0x4,
	GYRO_CONFIG0_GYRO_ODR_6400_HZ   = 0x3,
} gyro_config0_gyro_odr_t;

/*
 * FIFO_CONFIG0
 */

/* fifo_mode */
typedef enum {
	FIFO_CONFIG0_FIFO_MODE_SNAPSHOT = 0x02,
	FIFO_CONFIG0_FIFO_MODE_STREAM   = 0x01,
	FIFO_CONFIG0_FIFO_MODE_BYPASS   = 0x00,
} fifo_config0_fifo_mode_t;

/* fifo_depth */
typedef enum {
	FIFO_CONFIG0_FIFO_DEPTH_MAX = 0x1E, /* Errata AN-000364 */
#if defined(ICM45632M)
	FIFO_CONFIG0_FIFO_DEPTH_APEX = 0x0B,
#else
	FIFO_CONFIG0_FIFO_DEPTH_APEX = 0x07,
#endif
	FIFO_CONFIG0_FIFO_DEPTH_GAF = 0x04,
} fifo_config0_fifo_depth_t;

/*
 * FIFO_CONFIG2
 */

/* fifo_wr_wm_gt_th */
typedef enum {
	FIFO_CONFIG2_FIFO_WR_WM_EQ_OR_GT_TH = 0x1,
	FIFO_CONFIG2_FIFO_WR_WM_EQ_TH       = 0x0,
} fifo_config2_fifo_wr_wm_gt_th_t;

/*
 * FIFO_CONFIG4
 */

/* fifo_comp_nc_flow_cfg */
typedef enum {
	FIFO_CONFIG4_FIFO_COMP_NC_FLOW_CFG_EVERY_128_FR = 0x5,
	FIFO_CONFIG4_FIFO_COMP_NC_FLOW_CFG_EVERY_64_FR  = 0x4,
	FIFO_CONFIG4_FIFO_COMP_NC_FLOW_CFG_EVERY_32_FR  = 0x3,
	FIFO_CONFIG4_FIFO_COMP_NC_FLOW_CFG_EVERY_16_FR  = 0x2,
	FIFO_CONFIG4_FIFO_COMP_NC_FLOW_CFG_EVERY_8_FR   = 0x1,
	FIFO_CONFIG4_FIFO_COMP_NC_FLOW_CFG_DIS          = 0x0,
} fifo_config4_fifo_comp_nc_flow_cfg_t;

/* fifo_es0_6b_9b */
typedef enum {
	FIFO_CONFIG4_FIFO_ES0_9B = 0x1,
	FIFO_CONFIG4_FIFO_ES0_6B = 0x0,
} fifo_config4_fifo_es0_6b_9b_t;

/*
 * TMST_WOM_CONFIG
 */

/* tmst_resol */
typedef enum {
	TMST_WOM_CONFIG_TMST_RESOL_16_US = 0x01,
	TMST_WOM_CONFIG_TMST_RESOL_1_US  = 0x00,
} tmst_wom_config_tmst_resol_t;

/* wom_mode */
typedef enum {
	TMST_WOM_CONFIG_WOM_MODE_CMP_PREV = 0x01,
	TMST_WOM_CONFIG_WOM_MODE_CMP_INIT = 0x00,
} tmst_wom_config_wom_mode_t;

/* wom_int_mode*/
typedef enum {
	TMST_WOM_CONFIG_WOM_INT_MODE_ANDED = 0x01,
	TMST_WOM_CONFIG_WOM_INT_MODE_ORED  = 0x00,
} tmst_wom_config_wom_int_mode_t;

/* wom_int_dur */
typedef enum {
	TMST_WOM_CONFIG_WOM_INT_DUR_1_SMPL = 0x00,
	TMST_WOM_CONFIG_WOM_INT_DUR_2_SMPL = 0x01,
	TMST_WOM_CONFIG_WOM_INT_DUR_3_SMPL = 0x02,
	TMST_WOM_CONFIG_WOM_INT_DUR_4_SMPL = 0x03,
} tmst_wom_config_wom_int_dur_t;

/*
 * FSYNC_CONFIG0
 */

/* ap_fsync_sel */
typedef enum {
	FSYNC_CONFIG0_AP_FSYNC_NO      = 0x0,
	FSYNC_CONFIG0_AP_FSYNC_TEMP    = 0x1,
	FSYNC_CONFIG0_AP_FSYNC_GYRO_X  = 0x2,
	FSYNC_CONFIG0_AP_FSYNC_GYRO_Y  = 0x3,
	FSYNC_CONFIG0_AP_FSYNC_GYRO_Z  = 0x4,
	FSYNC_CONFIG0_AP_FSYNC_ACCEL_X = 0x5,
	FSYNC_CONFIG0_AP_FSYNC_ACCEL_Y = 0x6,
	FSYNC_CONFIG0_AP_FSYNC_ACCEL_Z = 0x7,
} fsync_config0_ap_fsync_sel_t;

/*
 * DMP_EXT_SEN_ODR_CFG
 */

/* ext_odr */
typedef enum {
	DMP_EXT_SEN_ODR_CFG_EXT_ODR_3_25_HZ = 0x00,
	DMP_EXT_SEN_ODR_CFG_EXT_ODR_6_25_HZ = 0x01,
	DMP_EXT_SEN_ODR_CFG_EXT_ODR_12_5_HZ = 0x02,
	DMP_EXT_SEN_ODR_CFG_EXT_ODR_25_HZ   = 0x03,
	DMP_EXT_SEN_ODR_CFG_EXT_ODR_50_HZ   = 0x04,
	DMP_EXT_SEN_ODR_CFG_EXT_ODR_100_HZ  = 0x05,
	DMP_EXT_SEN_ODR_CFG_EXT_ODR_200_HZ  = 0x06,
	DMP_EXT_SEN_ODR_CFG_EXT_ODR_400_HZ  = 0x07,
} dmp_ext_sen_odr_cfg_ext_odr_t;

/* apex_odr */
typedef enum {
	DMP_EXT_SEN_ODR_CFG_APEX_ODR_25_HZ  = 0x00,
	DMP_EXT_SEN_ODR_CFG_APEX_ODR_50_HZ  = 0x01,
	DMP_EXT_SEN_ODR_CFG_APEX_ODR_100_HZ = 0x02,
	DMP_EXT_SEN_ODR_CFG_APEX_ODR_200_HZ = 0x03,
	DMP_EXT_SEN_ODR_CFG_APEX_ODR_400_HZ = 0x04,
	DMP_EXT_SEN_ODR_CFG_APEX_ODR_800_HZ = 0x05,
} dmp_ext_sen_odr_cfg_apex_odr_t;

/*
 * ODR_DECIMATE_CONFIG
 */

/* gyro_fifo_odr_dec */
typedef enum {
	ODR_DECIMATE_CONFIG_GYRO_FIFO_ODR_DEC_1    = 0x0,
	ODR_DECIMATE_CONFIG_GYRO_FIFO_ODR_DEC_2    = 0x1,
	ODR_DECIMATE_CONFIG_GYRO_FIFO_ODR_DEC_4    = 0x2,
	ODR_DECIMATE_CONFIG_GYRO_FIFO_ODR_DEC_8    = 0x3,
	ODR_DECIMATE_CONFIG_GYRO_FIFO_ODR_DEC_16   = 0x4,
	ODR_DECIMATE_CONFIG_GYRO_FIFO_ODR_DEC_32   = 0x5,
	ODR_DECIMATE_CONFIG_GYRO_FIFO_ODR_DEC_64   = 0x6,
	ODR_DECIMATE_CONFIG_GYRO_FIFO_ODR_DEC_128  = 0x7,
	ODR_DECIMATE_CONFIG_GYRO_FIFO_ODR_DEC_256  = 0x8,
	ODR_DECIMATE_CONFIG_GYRO_FIFO_ODR_DEC_512  = 0x9,
	ODR_DECIMATE_CONFIG_GYRO_FIFO_ODR_DEC_1024 = 0xA,
	ODR_DECIMATE_CONFIG_GYRO_FIFO_ODR_DEC_2048 = 0xB,
	ODR_DECIMATE_CONFIG_GYRO_FIFO_ODR_DEC_4096 = 0xC,
} odr_decimate_config_gyro_fifo_odr_dec_t;

/* accel_fifo_odr_dec */
typedef enum {
	ODR_DECIMATE_CONFIG_ACCEL_FIFO_ODR_DEC_1    = 0x0,
	ODR_DECIMATE_CONFIG_ACCEL_FIFO_ODR_DEC_2    = 0x1,
	ODR_DECIMATE_CONFIG_ACCEL_FIFO_ODR_DEC_4    = 0x2,
	ODR_DECIMATE_CONFIG_ACCEL_FIFO_ODR_DEC_8    = 0x3,
	ODR_DECIMATE_CONFIG_ACCEL_FIFO_ODR_DEC_16   = 0x4,
	ODR_DECIMATE_CONFIG_ACCEL_FIFO_ODR_DEC_32   = 0x5,
	ODR_DECIMATE_CONFIG_ACCEL_FIFO_ODR_DEC_64   = 0x6,
	ODR_DECIMATE_CONFIG_ACCEL_FIFO_ODR_DEC_128  = 0x7,
	ODR_DECIMATE_CONFIG_ACCEL_FIFO_ODR_DEC_256  = 0x8,
	ODR_DECIMATE_CONFIG_ACCEL_FIFO_ODR_DEC_512  = 0x9,
	ODR_DECIMATE_CONFIG_ACCEL_FIFO_ODR_DEC_1024 = 0xA,
	ODR_DECIMATE_CONFIG_ACCEL_FIFO_ODR_DEC_2048 = 0xB,
	ODR_DECIMATE_CONFIG_ACCEL_FIFO_ODR_DEC_4096 = 0xC,
} odr_decimate_config_accel_fifo_odr_dec_t;

/*
 * INTF_CONFIG1_OVRD
 */

/* ap_spi_34_mode_ovrd_val */
typedef enum {
	INTF_CONFIG1_OVRD_AP_SPI_34_MODE_OVRD_VAL_3_WIRE = 0x0,
	INTF_CONFIG1_OVRD_AP_SPI_34_MODE_OVRD_VAL_4_WIRE = 0x1,
} intf_config1_ovrd_ap_spi_34_mode_ovrd_val_t;

/* ap_spi_mode_ovrd_val */
typedef enum {
	INTF_CONFIG1_OVRD_AP_SPI_MODE_OVRD_VAL_0_OR_3 = 0x0,
	INTF_CONFIG1_OVRD_AP_SPI_MODE_OVRD_VAL_1_OR_2 = 0x1,
} intf_config1_ovrd_ap_spi_mode_ovrd_val_t;

/*
 * DRIVE_CONFIG0
 */

/* pads_i2c_slew */
typedef enum {
	DRIVE_CONFIG0_PADS_I2C_SLEW_TYP_20NS = 0x0,
	DRIVE_CONFIG0_PADS_I2C_SLEW_TYP_7NS  = 0x2,
} drive_config0_pads_i2c_slew_t;

/* pads_spi_slew */
typedef enum {
	DRIVE_CONFIG0_PADS_SPI_SLEW_TYP_38NS  = 0x0,
	DRIVE_CONFIG0_PADS_SPI_SLEW_TYP_14NS  = 0x1,
	DRIVE_CONFIG0_PADS_SPI_SLEW_TYP_10NS  = 0x2,
	DRIVE_CONFIG0_PADS_SPI_SLEW_TYP_7NS   = 0x3,
	DRIVE_CONFIG0_PADS_SPI_SLEW_TYP_5NS   = 0x4,
	DRIVE_CONFIG0_PADS_SPI_SLEW_TYP_4NS   = 0x5,
	DRIVE_CONFIG0_PADS_SPI_SLEW_TYP_0_5NS = 0x6,
} drive_config0_pads_spi_slew_t;

/*
 * IOC_PAD_SCENARIO_OVRD
 */

/* pads_int2_cfg_ovrd_val */
typedef enum {
	IOC_PAD_SCENARIO_OVRD_INT2_CFG_OVRD_VAL_INT2 = 0,
#if INV_IMU_FSYNC_SUPPORTED
	IOC_PAD_SCENARIO_OVRD_INT2_CFG_OVRD_VAL_FSYNC = 1,
#endif
#if INV_IMU_CLKIN_SUPPORTED
	IOC_PAD_SCENARIO_OVRD_INT2_CFG_OVRD_VAL_CLKIN = 2,
#endif
	IOC_PAD_SCENARIO_OVRD_INT2_CFG_OVRD_VAL_DRDY_INTR = 3,
} ioc_pad_scenario_ovrd_pads_int2_cfg_ovrd_val_t;

/*
 * REG_MISC1
 */

/* osc_id_ovrd */
typedef enum {
	REG_MISC1_OSC_ID_OVRD_OFF     = 0x0,
	REG_MISC1_OSC_ID_OVRD_EDOSC   = 0x1,
	REG_MISC1_OSC_ID_OVRD_RCOSC   = 0x2,
	REG_MISC1_OSC_ID_OVRD_PLL     = 0x4,
	REG_MISC1_OSC_ID_OVRD_EXT_CLK = 0x8,
} reg_misc1_osc_id_ovrd_t;

/*
 * FS_SEL_AUX{1|2}
 */

/* aux_gyro_fs_sel */
typedef enum {
	FS_SEL_AUX_GYRO_FS_SEL_15_625_DPS = 8,
	FS_SEL_AUX_GYRO_FS_SEL_31_25_DPS  = 7,
	FS_SEL_AUX_GYRO_FS_SEL_62_5_DPS   = 6,
	FS_SEL_AUX_GYRO_FS_SEL_125_DPS    = 5,
	FS_SEL_AUX_GYRO_FS_SEL_250_DPS    = 4,
	FS_SEL_AUX_GYRO_FS_SEL_500_DPS    = 3,
	FS_SEL_AUX_GYRO_FS_SEL_1000_DPS   = 2,
	FS_SEL_AUX_GYRO_FS_SEL_2000_DPS   = 1,
#if INV_IMU_HIGH_FSR_SUPPORTED
	FS_SEL_AUX_GYRO_FS_SEL_4000_DPS = 0,
#endif
} fs_sel_aux_gyro_fs_sel_t;

/* aux_accel_fs_sel */
typedef enum {
	FS_SEL_AUX_ACCEL_FS_SEL_2_G  = 0x4,
	FS_SEL_AUX_ACCEL_FS_SEL_4_G  = 0x3,
	FS_SEL_AUX_ACCEL_FS_SEL_8_G  = 0x2,
	FS_SEL_AUX_ACCEL_FS_SEL_16_G = 0x1,
#if INV_IMU_HIGH_FSR_SUPPORTED
	FS_SEL_AUX_ACCEL_FS_SEL_32_G = 0x0,
#endif
} fs_sel_aux_accel_fs_sel_t;

/* ---------------------------------------------------------------------------
 * Bank IPREG_TOP1
 * ---------------------------------------------------------------------------*/

/*
 * SMC_CONTROL_0
 */

/* accel_lp_clk_sel */
typedef enum {
	SMC_CONTROL_0_ACCEL_LP_CLK_RCOSC = 0x01,
	SMC_CONTROL_0_ACCEL_LP_CLK_WUOSC = 0x00,
} smc_control_0_accel_lp_clk_sel_t;

/*
 * SREG_CTRL
 */

/* sreg_data_endian_sel */
typedef enum {
	SREG_CTRL_SREG_DATA_BIG_ENDIAN    = 0x01,
	SREG_CTRL_SREG_DATA_LITTLE_ENDIAN = 0x00,
} sreg_ctrl_sreg_data_endian_sel_t;

/*
 * STATUS_MASK_PIN_X_Y
 * Applies to STATUS_MASK_PIN_0_7_IPREG_TOP1, STATUS_MASK_PIN_8_15_IPREG_TOP1 and 
 * STATUS_MASK_PIN_16_23_IPREG_TOP1.
 */
#define EDMP_INT_SRC_ACCEL_DRDY_MASK   0x01
#define EDMP_INT_SRC_GYRO_DRDY_MASK    0x02
#define EDMP_INT_SRC_EXT_INT_DRDY_MASK 0x04
#define EDMP_INT_SRC_EXT_ODR_DRDY_MASK 0x08
#define EDMP_INT_SRC_WOM_DRDY_MASK     0x10
#define EDMP_INT_SRC_ON_DEMAND_MASK    0x20

/* ---------------------------------------------------------------------------
 * Bank IPREG_SYS1
 * ---------------------------------------------------------------------------*/

/*
 * IPREG_SYS1_REG_166
 */

/* gyro_src_ctrl */
typedef enum {
	IPREG_SYS1_REG_166_GYRO_SRC_CTRL_INTERPOLATOR_ON_FIR_ON   = 0x2,
	IPREG_SYS1_REG_166_GYRO_SRC_CTRL_INTERPOLATOR_OFF_FIR_ON  = 0x1,
	IPREG_SYS1_REG_166_GYRO_SRC_CTRL_INTERPOLATOR_OFF_FIR_OFF = 0x0,
} ipreg_sys1_reg_166_gyro_src_ctrl_sel_t;

/*
 * IPREG_SYS1_REG_170
 */

/* gyro_lp_avg_sel */
typedef enum {
	IPREG_SYS1_REG_170_GYRO_LP_AVG_64 = 0xC,
	IPREG_SYS1_REG_170_GYRO_LP_AVG_32 = 0xB,
	IPREG_SYS1_REG_170_GYRO_LP_AVG_20 = 0xA,
	IPREG_SYS1_REG_170_GYRO_LP_AVG_18 = 0x9,
	IPREG_SYS1_REG_170_GYRO_LP_AVG_16 = 0x8,
	IPREG_SYS1_REG_170_GYRO_LP_AVG_11 = 0x7,
	IPREG_SYS1_REG_170_GYRO_LP_AVG_10 = 0x6,
	IPREG_SYS1_REG_170_GYRO_LP_AVG_8  = 0x5,
	IPREG_SYS1_REG_170_GYRO_LP_AVG_7  = 0x4,
	IPREG_SYS1_REG_170_GYRO_LP_AVG_5  = 0x3,
	IPREG_SYS1_REG_170_GYRO_LP_AVG_4  = 0x2,
	IPREG_SYS1_REG_170_GYRO_LP_AVG_2  = 0x1,
	IPREG_SYS1_REG_170_GYRO_LP_AVG_1  = 0x0,
} ipreg_sys1_reg_170_gyro_lp_avg_sel_t;

/*
 * IPREG_SYS1_REG_172
 */

/* gyro_ui_lpfbw_sel */
typedef enum {
	IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_128   = 0x06,
	IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_64    = 0x05,
	IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_32    = 0x04,
	IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_16    = 0x03,
	IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_8     = 0x02,
	IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_4     = 0x01,
	IPREG_SYS1_REG_172_GYRO_UI_LPFBW_NO_FILTER = 0x00,
} ipreg_sys1_reg_172_gyro_ui_lpfbw_sel_t;

/* ---------------------------------------------------------------------------
 * Bank IPREG_SYS2
 * ---------------------------------------------------------------------------*/

/*
 * IPREG_SYS2_REG_123
 */

/* accel_src_ctrl */
typedef enum {
	IPREG_SYS2_REG_123_ACCEL_SRC_CTRL_INTERPOLATOR_ON_FIR_ON   = 0x2,
	IPREG_SYS2_REG_123_ACCEL_SRC_CTRL_INTERPOLATOR_OFF_FIR_ON  = 0x1,
	IPREG_SYS2_REG_123_ACCEL_SRC_CTRL_INTERPOLATOR_OFF_FIR_OFF = 0x0,
} ipreg_sys2_reg_123_accel_src_ctrl_sel_t;

/*
 * IPREG_SYS2_REG_129
 */

/* accel_lp_avg_sel */
typedef enum {
	IPREG_SYS2_REG_129_ACCEL_LP_AVG_64 = 0xC,
	IPREG_SYS2_REG_129_ACCEL_LP_AVG_32 = 0xB,
	IPREG_SYS2_REG_129_ACCEL_LP_AVG_20 = 0xA,
	IPREG_SYS2_REG_129_ACCEL_LP_AVG_18 = 0x9,
	IPREG_SYS2_REG_129_ACCEL_LP_AVG_16 = 0x8,
	IPREG_SYS2_REG_129_ACCEL_LP_AVG_11 = 0x7,
	IPREG_SYS2_REG_129_ACCEL_LP_AVG_10 = 0x6,
	IPREG_SYS2_REG_129_ACCEL_LP_AVG_8  = 0x5,
	IPREG_SYS2_REG_129_ACCEL_LP_AVG_7  = 0x4,
	IPREG_SYS2_REG_129_ACCEL_LP_AVG_5  = 0x3,
	IPREG_SYS2_REG_129_ACCEL_LP_AVG_4  = 0x2,
	IPREG_SYS2_REG_129_ACCEL_LP_AVG_2  = 0x1,
	IPREG_SYS2_REG_129_ACCEL_LP_AVG_1  = 0x0,
} ipreg_sys2_reg_129_accel_lp_avg_sel_t;

/*
 * IPREG_SYS2_REG_131
 */

/* accel_ui_lpfbw_sel */
typedef enum {
	IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_128   = 0x06,
	IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_64    = 0x05,
	IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_32    = 0x04,
	IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_16    = 0x03,
	IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_8     = 0x02,
	IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_4     = 0x01,
	IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_NO_FILTER = 0x00,
} ipreg_sys2_reg_131_accel_ui_lpfbw_t;

/* ---------------------------------------------------------------------------
 * Bank IMEM_SRAM
 * ---------------------------------------------------------------------------*/

/*
 * EDMP_TAP_TMAX
 */

/* equivalent to 0.5 seconds in sample count at 400Hz */
#define TAP_TMAX_400HZ 198
/* equivalent to 0.5 seconds in sample count at 800Hz */
#define TAP_TMAX_800HZ 396

/*
 * EDMP_TAP_TMIN
 */

/* equivalent to 0.165 seconds in sample count at 400Hz */
#define TAP_TMIN_400HZ 66
/* equivalent to 0.165 seconds in sample count at 800Hz */
#define TAP_TMIN_800HZ 132

/*
 * EDMP_TAP_SMUDGE_REJECT_THR
 */

/* equivalent to 0.085 seconds in sample count at 400Hz */
#define TAP_SMUDGE_REJECT_THR_400HZ 34
/* equivalent to 0.085 seconds in sample count at 800Hz */
#define TAP_SMUDGE_REJECT_THR_800HZ 68

/*
 * EDMP_STC_RESULTS
 */

#define STC_RESULTS_ACCEL_X_MASK   0x0001
#define STC_RESULTS_ACCEL_Y_MASK   0x0002
#define STC_RESULTS_ACCEL_Z_MASK   0x0004
#define STC_RESULTS_GYRO_X_MASK    0x0008
#define STC_RESULTS_GYRO_Y_MASK    0x0010
#define STC_RESULTS_GYRO_Z_MASK    0x0020
#define STC_RESULTS_ST_STATUS_MASK 0x00C0
#define STC_RESULTS_ACCEL_SC_MASK  0x0300
#define STC_RESULTS_GYRO_SC_MASK   0x0C00

/*
 * EDMP_STC_CONFIGPARAMS
 */

#define SELFTESTCAL_INIT_EN_MASK   0x0001
#define SELFTESTCAL_INIT_EN        0x0001
#define SELFTESTCAL_INIT_DIS       0x0000
#define SELFTEST_ACCEL_EN_MASK     0x0002
#define SELFTEST_ACCEL_EN          0x0002
#define SELFTEST_ACCEL_DIS         0x0000
#define SELFTEST_GYRO_EN_MASK      0x0004
#define SELFTEST_GYRO_EN           0x0004
#define SELFTEST_GYRO_DIS          0x0000
#define SELFTEST_AVERAGE_TIME_MASK 0x0380
#define SELFTEST_ACCEL_THRESH_MASK 0x1C00
#define SELFTEST_GYRO_THRESH_MASK  0xE000

typedef enum {
	SELFTEST_AVG_TIME_10_MS  = 0x0000,
	SELFTEST_AVG_TIME_20_MS  = 0x0080,
	SELFTEST_AVG_TIME_40_MS  = 0x0100,
	SELFTEST_AVG_TIME_80_MS  = 0x0180,
	SELFTEST_AVG_TIME_160_MS = 0x0200,
	SELFTEST_AVG_TIME_320_MS = 0x0280
} selftest_average_time_t;

typedef enum {
	SELFTEST_ACCEL_THRESHOLD_5_PERCENT  = 0x0000,
	SELFTEST_ACCEL_THRESHOLD_10_PERCENT = 0x0400,
	SELFTEST_ACCEL_THRESHOLD_15_PERCENT = 0x0800,
	SELFTEST_ACCEL_THRESHOLD_20_PERCENT = 0x0c00,
	SELFTEST_ACCEL_THRESHOLD_25_PERCENT = 0x1000,
	SELFTEST_ACCEL_THRESHOLD_30_PERCENT = 0x1400,
	SELFTEST_ACCEL_THRESHOLD_40_PERCENT = 0x1800,
	SELFTEST_ACCEL_THRESHOLD_50_PERCENT = 0x1c00
} selftest_accel_threshold_t;

typedef enum {
	SELFTEST_GYRO_THRESHOLD_5_PERCENT  = 0x0000,
	SELFTEST_GYRO_THRESHOLD_10_PERCENT = 0x2000,
	SELFTEST_GYRO_THRESHOLD_15_PERCENT = 0x4000,
	SELFTEST_GYRO_THRESHOLD_20_PERCENT = 0x6000,
	SELFTEST_GYRO_THRESHOLD_25_PERCENT = 0x8000,
	SELFTEST_GYRO_THRESHOLD_30_PERCENT = 0xa000,
	SELFTEST_GYRO_THRESHOLD_40_PERCENT = 0xc000,
	SELFTEST_GYRO_THRESHOLD_50_PERCENT = 0xe000
} selftest_gyro_threshold_t;

/*
 * EDMP_STC_PATCH_EN
 */

typedef enum {
	SELFTEST_PATCH_EN_ACCEL_PHASE1 = 0x0001,
	SELFTEST_PATCH_EN_ACCEL_PHASE2 = 0x0002,
	SELFTEST_PATCH_EN_GYRO1_PHASE1 = 0x0004,
	SELFTEST_PATCH_EN_GYRO1_PHASE2 = 0x0008
} stc_patch_params_t;

#ifdef __cplusplus
}
#endif

#endif /* #ifndef _INV_IMU_DEFS_H_ */

/** @} */
