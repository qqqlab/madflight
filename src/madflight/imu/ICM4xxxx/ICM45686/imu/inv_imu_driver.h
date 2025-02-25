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

/** @defgroup Driver Basic Driver
 *  @brief Basic API to drive the device.
 *  @{
 */

/** @file inv_imu_driver.h */

#ifndef _INV_IMU_DRIVER_H_
#define _INV_IMU_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "./inv_imu_defs.h"
#include "./inv_imu_transport.h"

#include <stdint.h>
#include <string.h>

/*
 * Driver's structures definitions 
 */

/** @brief Basic driver configuration structure */
typedef struct {
	/** @brief Transport structure */
	inv_imu_transport_t transport;

	/** @brief The calculated FIFO frame size in Bytes. */
	uint8_t fifo_frame_size;

	/** @brief Keeps track of data endianness mode 
	 * 0 : data in Sensor Registers and FIFO are in Little Endian format
	 * 1 : data in Sensor Registers and FIFO are in Big Endian format
	 */
	uint8_t endianness_data;

	/** @brief Memory area reserved for advanced module.
	 *         (only required when using advanced feature set).
	 *  @note  In case advanced module is not used, this field can be removed.
	 */
	uint64_t adv_var[6];

} inv_imu_device_t;

/** @brief One frame of FIFO header+data */
typedef union {
	fifo_header_t header;
	struct {
		fifo_header_t header;
		int16_t       sensor_data[3];
		int8_t        temp_data;
	} byte_8;
	struct {
		fifo_header_t header;
		int16_t       accel_data[3];
		int16_t       gyro_data[3];
		int8_t        temp_data;
		uint16_t      timestamp;
	} byte_16;
	struct {
		fifo_header_t header;
		int32_t       accel_data[3];
		int32_t       gyro_data[3];
		int16_t       temp_data;
		uint16_t      timestamp;
	} byte_20;
} inv_imu_fifo_data_t;

/** @brief Interrupts definition */
typedef struct {
	/* INTx_CONFIG0 */
	uint8_t INV_FIFO_FULL;
	uint8_t INV_FIFO_THS;
	uint8_t INV_UI_DRDY;
	uint8_t INV_OIS1;
	uint8_t INV_UI_FSYNC;
	uint8_t INV_AGC_RDY;
	uint8_t INV_OIS1_AGC_RDY;
	uint8_t INV_RESET_DONE;

	/* INTx_CONFIG1 */
	uint8_t INV_PLL_RDY;
	uint8_t INV_WOM_X;
	uint8_t INV_WOM_Y;
	uint8_t INV_WOM_Z;
	uint8_t INV_I3C_PROT_ERR;
	uint8_t INV_I2CM_DONE;
	uint8_t INV_EDMP_EVENT;
} inv_imu_int_state_t;

/** @brief Basic FIFO configuration */
typedef struct {
	uint8_t                   gyro_en; /**< Enable Gyro in FIFO */
	uint8_t                   accel_en; /**< Enable Accel in FIFO */
	uint8_t                   hires_en; /**< Enable High Resolution mode (20-bits long data) */
	uint16_t                  fifo_wm_th; /**< Watermark threshold value */
	fifo_config0_fifo_mode_t  fifo_mode; /**< Operating mode of the FIFO */
	fifo_config0_fifo_depth_t fifo_depth; /**< FIFO size */
} inv_imu_fifo_config_t;

/** @brief Macro to convert 2 bytes in 1 half-word depending on IMU endianness */
#define FORMAT_16_BITS_DATA(is_big_endian, pIn8, pOut16)                                           \
	*(pOut16) = ((is_big_endian) == 1) ? ((pIn8)[0] << 8) | (pIn8)[1] : ((pIn8)[1] << 8) | (pIn8)[0]

/*
 * API definitions 
 */

/** @brief Sleep function.
 *  @param[in] s   Pointer to device.
 *  @param[in] us  Time to sleep in microseconds.
 */
void inv_imu_sleep_us(inv_imu_device_t *s, uint32_t us);

/** @brief Performs a soft reset of the device.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_soft_reset(inv_imu_device_t *s);

/** @brief return WHOAMI value
 *  @param[in] s         Pointer to device.
 *  @param[out] who_am_i WHOAMI for device
 *  @return 0 on success, negative value on error
 */
int inv_imu_get_who_am_i(inv_imu_device_t *s, uint8_t *who_am_i);

/** @brief Configure accel mode.
 *  @param[in] s           Pointer to transport structure.
 *  @param[in] accel_mode  The requested mode.
 *  @return                0 on success, negative value on error.
 */
int inv_imu_set_accel_mode(inv_imu_device_t *s, pwr_mgmt0_accel_mode_t accel_mode);

/** @brief Configure gyro mode.
 *  @param[in] s          Pointer to transport structure.
 *  @param[in] gyro_mode  The requested mode.
 *  @return               0 on success, negative value on error.
 */
int inv_imu_set_gyro_mode(inv_imu_device_t *s, pwr_mgmt0_gyro_mode_t gyro_mode);

/** @brief Configure accel Output Data Rate.
 *  @param[in] s          Pointer to device.
 *  @param[in] frequency  The requested frequency.
 *  @return               0 on success, negative value on error.
 */
int inv_imu_set_accel_frequency(inv_imu_device_t *s, const accel_config0_accel_odr_t frequency);

/** @brief Configure gyro Output Data Rate.
 *  @param[in] s          Pointer to device.
 *  @param[in] frequency  The requested frequency.
 *  @return               0 on success, negative value on error.
 */
int inv_imu_set_gyro_frequency(inv_imu_device_t *s, const gyro_config0_gyro_odr_t frequency);

/** @brief Set accel full scale range.
 *  @param[in] s          Pointer to device.
 *  @param[in] accel_fsr  Requested full scale range.
 *  @return               0 on success, negative value on error.
 */
int inv_imu_set_accel_fsr(inv_imu_device_t *s, accel_config0_accel_ui_fs_sel_t accel_fsr);

/** @brief Set gyro full scale range.
 *  @param[in] s         Pointer to device.
 *  @param[in] gyro_fsr  Requested full scale range.
 *  @return              0 on success, negative value on error.
  */
int inv_imu_set_gyro_fsr(inv_imu_device_t *s, gyro_config0_gyro_ui_fs_sel_t gyro_fsr);

/** @brief Set accel Low-Power averaging value
 *  @param[in] s        Pointer to device.
 *  @param[in] acc_avg  Requested averaging value
 *  @return             0 on success, negative value on error.
 */
int inv_imu_set_accel_lp_avg(inv_imu_device_t *s, ipreg_sys2_reg_129_accel_lp_avg_sel_t acc_avg);

/** @brief Set gyro Low-Power averaging value
 *  @param[in] s        Pointer to device.
 *  @param[in] gyr_avg  Requested averaging value
 *  @return             0 on success, negative value on error.
 */
int inv_imu_set_gyro_lp_avg(inv_imu_device_t *s, ipreg_sys1_reg_170_gyro_lp_avg_sel_t gyr_avg);

/** @brief Set accel Low-Noise bandwidth value
 *  @param[in] s       Pointer to device.
 *  @param[in] acc_bw  Requested bandwidth value
 *  @return            0 on success, negative value on error.
 */
int inv_imu_set_accel_ln_bw(inv_imu_device_t *s, ipreg_sys2_reg_131_accel_ui_lpfbw_t acc_bw);

/** @brief Set gyro Low-Noise bandwidth value
 *  @param[in] s       Pointer to device.
 *  @param[in] gyr_bw  Requested bandwidth value
 *  @return            0 on success, negative value on error.
 */
int inv_imu_set_gyro_ln_bw(inv_imu_device_t *s, ipreg_sys1_reg_172_gyro_ui_lpfbw_sel_t gyr_bw);

/** @brief Get current sensor data from the registers.
 *  @param[in] s      Pointer to device.
 *  @param[out] data  Current accel, gyro and temperature data from the registers.
 *  @return           0 on success, negative value on error.
 */
int inv_imu_get_register_data(inv_imu_device_t *s, inv_imu_sensor_data_t *data);

/** @brief Configures the FIFO to the specified state. 
 *  @param[in] s            Pointer to device.
 *  @param[in] fifo_config  Structure containing the FIFO configuration.
 *  @return                 0 on success, negative value on error.
 */
int inv_imu_set_fifo_config(inv_imu_device_t *s, const inv_imu_fifo_config_t *fifo_config);

/** @brief Gets the current FIFO configuration.
 *  @param[in] s            Pointer to device.
 *  @param[in] fifo_config  Structure containing the FIFO configuration.
 *  @return                 0 on success, negative value on error.
 */
int inv_imu_get_fifo_config(inv_imu_device_t *s, inv_imu_fifo_config_t *fifo_config);

/** @brief Flush FIFO content.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_flush_fifo(inv_imu_device_t *s);

/** @brief Get FIFO frame count.
 *  @param[in] s             Pointer to device.
 *  @param[out] frame_count  The number of frames in the FIFO.
 *  @return                  0 on success, negative value on error.
 */
int inv_imu_get_frame_count(inv_imu_device_t *s, uint16_t *frame_count);

/** @brief Get one frame of FIFO data.
 *  @param[in] s      Pointer to device.
 *  @param[out] data  Accel, gyro and temperature data from the top frame on the FIFO.
 *  @return           0 on success, negative value on error.
 */
int inv_imu_get_fifo_frame(inv_imu_device_t *s, inv_imu_fifo_data_t *data);

/** @brief Configure interrupts source.
 *  @param[in] s    Pointer to device.
 *  @param[in] num  Interrupt number
 *  @param[in] it   State of each interrupt
 *  @return         0 on success, negative value on error.
 */
int inv_imu_set_config_int(inv_imu_device_t *s, const inv_imu_int_num_t num,
                           const inv_imu_int_state_t *it);

/** @brief Retrieve interrupts configuration.
 *  @param[in] s    Pointer to device.
 *  @param[in] num  Interrupt number
 *  @param[out] it  State of each interrupt
 *  @return         0 on success, negative value on error.
 */
int inv_imu_get_config_int(inv_imu_device_t *s, const inv_imu_int_num_t num,
                           inv_imu_int_state_t *it);

/** @brief Configure pin behavior.
 *  @param[in] s     Pointer to device.
 *  @param[in] num   Interrupt number
 *  @param[in] conf  Structure with the requested configuration.
 *  @return          0 on success, negative value on error.
 */
int inv_imu_set_pin_config_int(inv_imu_device_t *s, const inv_imu_int_num_t num,
                               const inv_imu_int_pin_config_t *conf);

/** @brief Read interrupt 1 status.
 *  @param[in] s    Pointer to device.
 *  @param[in] num  Interrupt number
 *  @param[out] it  Status of each interrupt.
 *  @return         0 on success, negative value on error.
 */
int inv_imu_get_int_status(inv_imu_device_t *s, const inv_imu_int_num_t num,
                           inv_imu_int_state_t *it);

/** @brief Read the UI endianness and set the inv_device endianness field
 *  @param[in] s  Pointer to device. 
 *  @return       0 on success, negative value on error.
 */
int inv_imu_get_endianness(inv_imu_device_t *s);

/** @brief Select which clock to use when in Low Power mode.
 *         Use `SMC_CONTROL_0_ACCEL_LP_CLK_RCOSC` for Low Power (LP) mode.
 *         Use `SMC_CONTROL_0_ACCEL_LP_CLK_WUOSC` for Ultra Low Power (ULP) mode.
 *  @note In ULP mode, sensor registers are not available and the host must retrieve 
 *        data from the FIFO.
 *  @param[in] s        Pointer to device.
 *  @param[in] clk_sel  Selected clock.
 *  @return             0 on success, negative value on error.
 */
int inv_imu_select_accel_lp_clk(inv_imu_device_t *s, smc_control_0_accel_lp_clk_sel_t clk_sel);

/** @brief Return driver version x.y.z-suffix as a char array
 *  @return  Driver version as char array "x.y.z-suffix"
 */
const char *inv_imu_get_version(void);

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_DRIVER_H_ */

/** @} */
