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

/** @defgroup DriverAdv Driver Advanced
 *  @brief    High-level API for advanced functionalities.
 *  @{
 */

/** @file inv_imu_driver_advanced.h */

#ifndef _INV_IMU_DRIVER_ADVANCED_H_
#define _INV_IMU_DRIVER_ADVANCED_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "./inv_imu_driver.h"

#include <stdint.h>
#include <string.h>

/** @brief Maximum buffer size mirrored from FIFO */
#define FIFO_MIRRORING_SIZE 16 * 258 /* packet size * max_count = 4kB */

/*
 * Driver's structures definitions 
 */

/** @brief Sensor event structure definition */
typedef struct {
	/** Specifies which sensors are available in the event 
	 * (defined by inv_imu_sensor_id_t as a mask)
	 */
	int sensor_mask;

	/** Value of the FIFO timestamp (if FIFO is used) */
	uint16_t timestamp_fsync;

	/** Accel raw data */
	int16_t accel[3];

	/** Gyro raw data */
	int16_t gyro[3];

	/** Temperature raw data */
	int16_t temperature;

	/** High-res portion of the accel raw data (if using high-res mode) */
	int8_t accel_high_res[3];

	/** High-res portion of the accel raw data (if using high-res mode) */
	int8_t gyro_high_res[3];

	/** Buffer for external sensor 0 connected to EDMP */
	uint8_t es0[9];

	/** Buffer for external sensor 1 connected to EDMP */
	uint8_t es1[6];
} inv_imu_sensor_event_t;

/** @brief Definition of extended variables */
typedef struct {
	/** @brief Callback executed when a new sensor event is available.
	 *  @param[in] event  Pointer to the sensor event.
	 */
	void (*sensor_event_cb)(inv_imu_sensor_event_t *event);

	/* The following fields will be initialized by inv_imu_adv_init() */
	uint8_t                  fifo_is_used; /**< Keeps track of FIFO usage */
	uint8_t                  fifo_comp_en; /**< Indicates if FIFO compression is enabled */
	fifo_config0_fifo_mode_t fifo_mode; /**< Current fifo mode. Required by AN-000364 */

	/* Variables related to FIFO compression */
	int16_t accel_baseline[3]; /**< Baseline for the accel */
	int16_t gyro_baseline[3]; /**< Baseline for the gyro */
	int16_t temp_baseline; /**< Baseline for the temperature sensor */
	uint8_t accel_baseline_found; /**< Flag indicating accel baseline has been found */
	uint8_t gyro_baseline_found; /**< Flag indicating gyro baseline has been found */
	uint8_t temp_baseline_found; /**< Flag indicating temperature baseline has been found */

#if INV_IMU_FSYNC_SUPPORTED
	/* Variables related to FSYNC tag */
	fsync_config0_ap_fsync_sel_t fsync_tag; /**< Keeps track of FSYNC tag in sensor data regs */
#endif
} inv_imu_adv_var_t;

/** @brief Sensor identifier enumeration */
typedef enum {
	INV_SENSOR_ACCEL,
	INV_SENSOR_GYRO,
	INV_SENSOR_FSYNC_EVENT,
	INV_SENSOR_TEMPERATURE,
	INV_SENSOR_EDMP_PEDOMETER_EVENT,
	INV_SENSOR_EDMP_PEDOMETER_COUNT,
	INV_SENSOR_EDMP_TILT,
	INV_SENSOR_EDMP_FF,
	INV_SENSOR_EDMP_LOWG,
	INV_SENSOR_EDMP_HIGHG,
	INV_SENSOR_EDMP_SMD,
	INV_SENSOR_EDMP_TAP,
	INV_SENSOR_EDMP_R2W_WAKE,
	INV_SENSOR_EDMP_R2W_SLEEP,
	INV_SENSOR_ES0,
	INV_SENSOR_ES1,
	INV_SENSOR_MAX
} inv_imu_sensor_id_t;

/** @brief FIFO configuration structure */
typedef struct {
	/** @brief Basic FIFO configuration */
	inv_imu_fifo_config_t base_conf;

	/** @brief Condition to trig watermark interrupt */
	fifo_config2_fifo_wr_wm_gt_th_t fifo_wr_wm_gt_th;

	/* FIFO content */
	uint8_t tmst_fsync_en; /**< Enable Timestamp or FSYNC delay in FIFO */

	/* External Sensor(s) */
	uint8_t es1_en; /**< Enable External Sensor 1 to be pushed in FIFO */
	uint8_t es0_en; /**< Enable External Sensor 0 to be pushed in FIFO */

	/** @brief Size of the External Sensor 0 data (6 bytes or 9 bytes) */
	fifo_config4_fifo_es0_6b_9b_t es0_6b_9b;

	/* FIFO Compression */

	/** @brief Enable FIFO compression */
	uint8_t comp_en;

	/** @brief Rate at which an uncompressed frame is generated */
	fifo_config4_fifo_comp_nc_flow_cfg_t comp_nc_flow_cfg;

	/* FIFO Decimation */

	/** @brief Decimation rate for gyro */
	odr_decimate_config_gyro_fifo_odr_dec_t gyro_dec;

	/** @brief Decimation rate for accel */
	odr_decimate_config_accel_fifo_odr_dec_t accel_dec;
} inv_imu_adv_fifo_config_t;

/*
 * API definitions 
 */

/** @brief Initializes device.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_init(inv_imu_device_t *s);

/** @brief Performs a soft reset of the device.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_device_reset(inv_imu_device_t *s);

/** @brief Enable accel in low power mode
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_enable_accel_lp(inv_imu_device_t *s);

/** @brief Enable accel in low noise mode
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_enable_accel_ln(inv_imu_device_t *s);

/** @brief Disable accel.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_disable_accel(inv_imu_device_t *s);

/** @brief Enable gyro in low noise mode.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_enable_gyro_ln(inv_imu_device_t *s);

/** @brief Enable gyro in low power mode.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_enable_gyro_lp(inv_imu_device_t *s);

/** @brief Disable gyro
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_disable_gyro(inv_imu_device_t *s);

#if INV_IMU_INT2_PIN_SUPPORTED
/** @brief Configures INT2 pin for the requested usage (INT2, FSYNC, CLKIN or DRDY_INTR).
 *  It also disables AUX2 in pad scenario if AUX2 is supported for current variant.
 *  @param[in] s      Pointer to device.
 *  @param[in] usage  Requested usage for INT2 pin.
 *  @return           0 on success, negative value on error.
 */
int inv_imu_adv_set_int2_pin_usage(inv_imu_device_t *                             s,
                                   ioc_pad_scenario_ovrd_pads_int2_cfg_ovrd_val_t usage);
#endif /* INV_IMU_INT2_PIN_SUPPORTED */

#if INV_IMU_FSYNC_SUPPORTED
/** @brief Configures FSYNC tag in sensor registers
 *  @param[in] s           Pointer to device.
 *  @param[in] sensor_tag  Indicates which sensor data register LSB should be set when a FSYNC event is detected.
 *  @return                0 on success, negative value on error.
 */
int inv_imu_adv_configure_fsync_ap_tag(inv_imu_device_t *           s,
                                       fsync_config0_ap_fsync_sel_t sensor_tag);

/** @brief Enable fsync tagging functionnality.
 *  In details it:
 *     - enables fsync
 *     - enables timestamp to registers. Once fysnc is enabled fsync counter is pushed to 
 *       fifo instead of timestamp. So timestamp is made available in registers. Note that 
 *       this increase power consumption.
 *     - enables fsync related interrupt
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_enable_fsync(inv_imu_device_t *s);

/** @brief Disable fsync tagging functionnality.
 *  In details it:
 *     - disables fsync
 *     - disables timestamp to registers. Once fysnc is disabled  timestamp is pushed to fifo 
 *        instead of fsync counter. So in order to decrease power consumption, timestamp is no 
 *        more available in registers.
 *     - disables fsync related interrupt
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_disable_fsync(inv_imu_device_t *s);
#endif

/** @brief Read all registers containing data (temperature, accelerometer and gyroscope). 
 *         It will then call `sensor_event_cb` function provided 
 *         in the `inv_imu_device_t` for each packet.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_get_data_from_registers(inv_imu_device_t *s);

/** @brief reset IMU fifo
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_reset_fifo(inv_imu_device_t *s);

/** @brief Retrieve FIFO configuration.
 *  @param[in] s     Pointer to device.
 *  @param[in] conf  Structure that will be filled with current configuration.
 *  @return          0 on success, negative value on error.
 */
int inv_imu_adv_get_fifo_config(inv_imu_device_t *s, inv_imu_adv_fifo_config_t *conf);

/** @brief Set FIFO configuration.
 *  @param[in] s     Pointer to device.
 *  @param[in] conf  Structure containing the requested configuration.
 *  @return          0 on success, negative value on error.
 */
int inv_imu_adv_set_fifo_config(inv_imu_device_t *s, const inv_imu_adv_fifo_config_t *conf);

/** @brief Read all available packets from the FIFO. 
 *  @param[in] s            Pointer to device.
 *  @param[out] fifo_data   Pointer to FIFO data buffer.
 *  @param[out] fifo_count  Number of packet read in FIFO.
 *  @return                 0 on success, negative value on error.
 */
int inv_imu_adv_get_data_from_fifo(inv_imu_device_t *s, uint8_t fifo_data[FIFO_MIRRORING_SIZE],
                                   uint16_t *fifo_count);

/** @brief Parse packets from FIFO buffer. For each packet function builds a
 *         sensor event containing packet data and validity information. Then it calls 
 *         sensor_event_cb funtion passed in parameter of inv_imu_init function for each 
 *         packet.
 *  @param[in] s           Pointer to device.
 *  @param[in] fifo_data   Pointer to FIFO data buffer.
 *  @param[in] fifo_count  Number of packet read in FIFO.
 *  @return                0 on success, negative value on error.
 */
int inv_imu_adv_parse_fifo_data(inv_imu_device_t *s, const uint8_t fifo_data[FIFO_MIRRORING_SIZE],
                                const uint16_t fifo_count);

/** @brief Converts accel_config0_accel_odr_t or gyro_config0_gyro_odr_t enums to period expressed in us
 *  @param[in] odr_bitfield An accel_config0_accel_odr_t or gyro_config0_gyro_odr_t enum
 *  @return The corresponding period expressed in us
 */
uint32_t inv_imu_adv_convert_odr_bitfield_to_us(uint32_t odr_bitfield);

/** @brief Access accel full scale range
 *  @param[in] s           Pointer to device.
 *  @param[out] accel_fsr  Current full scale range.
 *  @return                0 on success, negative value on error.
 */
int inv_imu_adv_get_accel_fsr(inv_imu_device_t *s, accel_config0_accel_ui_fs_sel_t *accel_fsr);

/** @brief Access gyro full scale range
 *  @param[in] s          Pointer to device.
 *  @param[out] gyro_fsr  Current full scale range.
 *  @return               0 on success, negative value on error.
 */
int inv_imu_adv_get_gyro_fsr(inv_imu_device_t *s, gyro_config0_gyro_ui_fs_sel_t *gyro_fsr);

/** @brief Set timestamp resolution
 *  @param[in] s                Pointer to device.
 *  @param[in] timestamp_resol  Requested timestamp resolution
 *  @return                     0 on success, negative value on error.
 */
int inv_imu_adv_set_timestamp_resolution(inv_imu_device_t *                 s,
                                         const tmst_wom_config_tmst_resol_t timestamp_resol);

/** @brief Get timestamp resolution.
 *  @param[in] s  Pointer to device.
 *  @return       Timestamp resolution in us, negative value on error
 */
uint32_t inv_imu_adv_get_timestamp_resolution_us(inv_imu_device_t *s);

#if INV_IMU_CLKIN_SUPPORTED
/** @brief Enable CLKIN RTC functionality.
 *  In details it:
 *     - disables the I3CSM STC.
 *     - enables FIR filter and interpolator for accel and gyro.
 *     - enables CLKIN RTC mode.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_enable_clkin_rtc(inv_imu_device_t *s);

/** @brief Disable CLKIN RTC functionality.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_disable_clkin_rtc(inv_imu_device_t *s);
#endif /* INV_IMU_CLKIN_SUPPORTED */

/** @brief  Enable Wake On Motion.
 *  @param[in] s         Pointer to device. 
 *  @param[in] wom_x_th  Threshold for X axis with 1g/256 resolution (wom_x_th = mg * 256 / 1000).
 *  @param[in] wom_y_th  Threshold for Y axis with 1g/256 resolution (wom_y_th = mg * 256 / 1000).
 *  @param[in] wom_z_th  Threshold for Z axis with 1g/256 resolution (wom_z_th = mg * 256 / 1000).
 *  @param[in] wom_int   Mode used to generate interrupt (AND/OR).
 *  @param[in] wom_dur   Number of overthreshold events to wait before generating interrupt.
 *  @return              0 on success, negative value on error.
 */
int inv_imu_adv_configure_wom(inv_imu_device_t *s, const uint8_t wom_x_th, const uint8_t wom_y_th,
                              const uint8_t wom_z_th, tmst_wom_config_wom_int_mode_t wom_int,
                              tmst_wom_config_wom_int_dur_t wom_dur);

/** @brief  Enable Wake On Motion.
 *  note : WoM requests to have the accelerometer enabled to work. 
 *  As a consequence Fifo water-mark interrupt is disabled to only trigger WoM interrupts.
 *  To have good performance, it's recommended to set accelerometer ODR (Output Data Rate) to 20ms
 *  and the accelerometer in Low Power Mode.
 *  @param[in] s  Pointer to device. 
   @return        0 on success, negative value on error.
 */
int inv_imu_adv_enable_wom(inv_imu_device_t *s);

/** @brief  Disable Wake On Motion.
 *  note : Fifo water-mark interrupt is re-enabled when WoM is disabled.
 *  @param[in] s  Pointer to device. 
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_disable_wom(inv_imu_device_t *s);

/** @brief Set the UI endianness and set the inv_device endianness field
 *  @param[in] s           Pointer to device. 
 *  @param[in] endianness  Requested endianness value. 
 *  @return                0 on success, negative value on error.
 */
int inv_imu_adv_set_endianness(inv_imu_device_t *s, sreg_ctrl_sreg_data_endian_sel_t endianness);

/** @brief Power-up the SRAM.
 *  @param[in] s  Pointer to device. 
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_power_up_sram(inv_imu_device_t *s);

/** @brief Power-down the SRAM.
 *  @param[in] s  Pointer to device. 
 *  @return       0 on success, negative value on error.
 */
int inv_imu_adv_power_down_sram(inv_imu_device_t *s);

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_DRIVER_ADVANCED_H_ */

/** @} */
