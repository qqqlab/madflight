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

/** @defgroup EDMP EDMP
 *  @brief API to drive eDMP features.
 *  @{
 */

/** @file inv_imu_edmp.h */

#ifndef _INV_IMU_EDMP_H_
#define _INV_IMU_EDMP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "./inv_imu_driver.h"
#include "./inv_imu_edmp_memmap.h"

#include <stdint.h>
#include <string.h>

/** @brief Writes in EDMP SRAM
 *  @param[in] s     Pointer to device.
 *  @param[in] name  Name of the parameter.
 *  @param[in] val   Value to be written.
 *  @return          0 on success, negative value on error.
 */
#define INV_IMU_WRITE_EDMP_SRAM(s, name, val)                                                      \
	inv_imu_write_sram(s, (uint32_t)name, name##_SIZE, val)

/** @brief Reads in EDMP SRAM
 *  @param[in] s     Pointer to device.
 *  @param[in] name  Name of the parameter.
 *  @param[in] val   Value to be read.
 *  @return          0 on success, negative value on error.
 */
#define INV_IMU_READ_EDMP_SRAM(s, name, val) inv_imu_read_sram(s, (uint32_t)name, name##_SIZE, val)

/** @brief EDMP input interrupt lines definition */
typedef enum { INV_IMU_EDMP_INT0 = 0, INV_IMU_EDMP_INT1, INV_IMU_EDMP_INT2 } inv_imu_edmp_int_t;

/** @brief APEX interrupts definition */
typedef struct {
	uint8_t INV_TAP;
	uint8_t INV_HIGHG;
	uint8_t INV_LOWG;
	uint8_t INV_TILT_DET;
	uint8_t INV_STEP_CNT_OVFL;
	uint8_t INV_STEP_DET;
	uint8_t INV_FF;
	uint8_t INV_R2W;
	uint8_t INV_B2S;

	uint8_t INV_R2W_SLEEP;
	uint8_t INV_B2S_REV;
	uint8_t INV_SMD;
	uint8_t INV_SELF_TEST;
	uint8_t INV_SEC_AUTH;
} inv_imu_edmp_int_state_t;

/** Registers to retrieve interrupts status for APEX. */
typedef struct {
	int_apex_status0_t int_apex_status0;
	int_apex_status1_t int_apex_status1;
} int_apex_statusx_t;

/** Registers to configure interrupts for APEX. */
typedef struct {
	int_apex_config0_t int_apex_config0;
	int_apex_config1_t int_apex_config1;
} int_apex_configx_t;

/** Registers to enable APEX features. */
typedef struct {
	edmp_apex_en0_t edmp_apex_en0;
	edmp_apex_en1_t edmp_apex_en1;
} edmp_apex_enx_t;

/** @brief IMU APEX inputs parameters definition
 *  @note Refer to the datasheet for details on how to configure these parameters.
 */
typedef struct {
	/* Pedometer */
	uint32_t ped_amp_th;
	uint16_t ped_step_cnt_th;
	uint16_t ped_step_det_th;
	uint16_t ped_sb_timer_th;
	uint32_t ped_hi_en_th;
	uint8_t  ped_sensitivity_mode;
	uint32_t ped_low_en_amp_th;

	/* Tilt */
	uint16_t tilt_wait_time;

#if INV_IMU_USE_BASIC_SMD
	/* Basic SMD */
	uint32_t basicsmd_win;
	uint32_t basicsmd_win_wait;
#else
	/* SMD */
	uint8_t smd_sensitivity;
#endif
	/* R2W */
	uint32_t r2w_sleep_time_out;
	uint32_t r2w_sleep_gesture_delay;
	uint32_t r2w_mounting_matrix;
	uint32_t r2w_gravity_filter_gain;
	uint32_t r2w_motion_th_angle_cosine;
	uint32_t r2w_motion_th_timer_fast;
	uint32_t r2w_motion_th_timer_slow;
	uint32_t r2w_motion_prev_gravity_timeout;
	uint32_t r2w_last_gravity_motion_timer;
	uint32_t r2w_last_gravity_timeout;
	uint32_t r2w_gesture_validity_timeout;

	/* Freefall */
	uint16_t lowg_peak_th;
	uint16_t lowg_peak_th_hyst;
	uint16_t lowg_time_th;
	uint16_t highg_peak_th;
	uint16_t highg_peak_th_hyst;
	uint16_t highg_time_th;
	uint32_t ff_min_duration;
	uint32_t ff_max_duration;
	uint32_t ff_debounce_duration;

	/* Tap */
	uint8_t  tap_min_jerk;
	uint16_t tap_tmax;
	uint8_t  tap_tmin;
	uint8_t  tap_max_peak_tol;
	uint8_t  tap_smudge_reject_th;
	uint8_t  tap_tavg;

	/* CalMag */
	int32_t soft_iron_sensitivity_matrix[9];
	int32_t hard_iron_offset[3];

	/* Power save */
	uint32_t power_save_time;
	uint8_t  power_save_en;

} inv_imu_edmp_apex_parameters_t;

/** @brief Pedometer activity class */
typedef enum {
	INV_IMU_EDMP_UNKNOWN = 0,
	INV_IMU_EDMP_WALK    = 1,
	INV_IMU_EDMP_RUN     = 2,
} inv_imu_edmp_activity_class_t;

/** @brief Pedometer outputs */
typedef struct {
	/** @brief Number of steps. */
	uint16_t step_cnt;

	/** @brief Walk/Run cadency in number of samples.
	 *         Number of samples between two steps with u6.2 format (8-bits unsigned in Q2).
	 *         cadency (steps/s) = EDMP_ODR_HZ / (step_cadence * 0.25)
	 */
	uint8_t step_cadence;

	/** @brief Detected activity. */
	inv_imu_edmp_activity_class_t activity_class;
} inv_imu_edmp_pedometer_data_t;

/** @brief Tap number definition */
typedef enum {
	INV_IMU_EDMP_TAP_DOUBLE = 0x02,
	INV_IMU_EDMP_TAP_SINGLE = 0x01,
} inv_imu_edmp_tap_num_t;

/** @brief Tap axis definition */
typedef enum {
	INV_IMU_EDMP_TAP_AXIS_Z = 0x02,
	INV_IMU_EDMP_TAP_AXIS_Y = 0x01,
	INV_IMU_EDMP_TAP_AXIS_X = 0x00,
} inv_imu_edmp_tap_axis_t;

/** @brief Tap direction definition */
typedef enum {
	INV_IMU_EDMP_TAP_DIR_POSITIVE = 0x01,
	INV_IMU_EDMP_TAP_DIR_NEGATIVE = 0x00,
} inv_imu_edmp_tap_dir_t;

/** @brief Tap outputs */
typedef struct {
	inv_imu_edmp_tap_num_t  num;
	inv_imu_edmp_tap_axis_t axis;
	inv_imu_edmp_tap_dir_t  direction;
	uint8_t                 double_tap_timing;
} inv_imu_edmp_tap_data_t;

/** @brief Configure EDMP Output Data Rate.
 *  @warning Accel frequency must be higher or equal to EDMP frequency.
 *  @warning If inv_imu_edmp_init_apex() was already called, application should call
 *          `inv_imu_edmp_recompute_apex_decimation()` afterwards if APEX algorithms are to be run.
 *  @param[in] s         Pointer to device.
 *  @param[in] frequency The requested frequency.
 *  @return              0 on success, negative value on error.
 */
int inv_imu_edmp_set_frequency(inv_imu_device_t *s, const dmp_ext_sen_odr_cfg_apex_odr_t frequency);

/** @brief Initialize EDMP APEX algorithms. This function should be called before
 *         calling any other function (except for `inv_imu_edmp_set_frequency`).
 *  @warning This function will power-up the SRAM. For power consumption consideration,
 *           you can manually call `inv_imu_adv_power_down_sram` if you don't need to 
 *           preserve SRAM content.
 *  @warning This function will reset all interrupt masks previously set with
 *           `inv_imu_edmp_unmask_int_src` and exit with `EDMP_INT_SRC_ACCEL_DRDY_MASK` unmasked on
 *           `INV_IMU_EDMP_INT0`.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error or if EDMP is enabled.
 */
int inv_imu_edmp_init_apex(inv_imu_device_t *s);

/** @brief Recompute EDMP APEX algorithms internal decimator based on new EDMP output Data Rate
           configured with `inv_imu_edmp_set_frequency`.
 *  @warning It is up to application level to save/restore previously configured APEX parameters,
 *           if any, with `inv_imu_edmp_set_apex_parameters`.
 *  @warning EDMP must be disabled before calling this function.
 *  @warning This function will reset all interrupt masks previously set with
 *           `inv_imu_edmp_unmask_int_src` and exit with `EDMP_INT_SRC_ACCEL_DRDY_MASK` unmasked on
 *           `INV_IMU_EDMP_INT0`.
 *  @param[in] s         Pointer to device.
 *  @return              0 on success, negative value on error or if EDMP is enabled.
 */
int inv_imu_edmp_recompute_apex_decimation(inv_imu_device_t *s);

/** @brief Returns current EDMP parameters for APEX algorithms.
 *  @param[in] s   Pointer to device.
 *  @param[out] p  The current parameters read from registers.
 *  @return        0 on success, negative value on error.
 */
int inv_imu_edmp_get_apex_parameters(inv_imu_device_t *s, inv_imu_edmp_apex_parameters_t *p);

/** @brief Configures EDMP parameters for APEX algorithms.
 *  @warning This function should be called only when all EDMP algorithms are disabled.
 *  @param[in] s  Pointer to device.
 *  @param[in] p  The requested input parameters.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_set_apex_parameters(inv_imu_device_t *s, const inv_imu_edmp_apex_parameters_t *p);

/** @brief Retrieve interrupts configuration.
 *  @param[in] s    Pointer to device.
 *  @param[out] it  Configuration of each APEX interrupt.
 *  @return         0 on success, negative value on error.
 */
int inv_imu_edmp_get_config_int_apex(inv_imu_device_t *s, inv_imu_edmp_int_state_t *it);

/** @brief Configure APEX interrupt.
 *  @param[in] s   Pointer to device.
 *  @param[in] it  State of each APEX interrupt to configure.
 *  @return        0 on success, negative value on error.
 */
int inv_imu_edmp_set_config_int_apex(inv_imu_device_t *s, const inv_imu_edmp_int_state_t *it);

/** @brief  Enable EDMP.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_enable(inv_imu_device_t *s);

/** @brief  Disable EDMP.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_disable(inv_imu_device_t *s);

/** @brief  Enable APEX algorithm Pedometer.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success
 *                INV_IMU_ERROR_EDMP_ODR if user should have called
 *                                       `inv_imu_edmp_recompute_apex_decimation()`
 *                other negative value on error.
 */
int inv_imu_edmp_enable_pedometer(inv_imu_device_t *s);

/** @brief  Disable APEX algorithm Pedometer.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_disable_pedometer(inv_imu_device_t *s);

/** @brief  Enable APEX algorithm Significant Motion Detection.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success
 *                INV_IMU_ERROR_EDMP_ODR if user should have called
 *                                       `inv_imu_edmp_recompute_apex_decimation()`
 *                other negative value on error.
 */
int inv_imu_edmp_enable_smd(inv_imu_device_t *s);

/** @brief  Disable APEX algorithm Significant Motion Detection.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_disable_smd(inv_imu_device_t *s);

/** @brief  Enable APEX algorithm Tilt.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success
 *                INV_IMU_ERROR_EDMP_ODR if user should have called
 *                                       `inv_imu_edmp_recompute_apex_decimation()`
 *                other negative value on error.
 */
int inv_imu_edmp_enable_tilt(inv_imu_device_t *s);

/** @brief  Disable APEX algorithm Tilt.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_disable_tilt(inv_imu_device_t *s);

/** @brief  Enable APEX algorithm R2W.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success
 *                INV_IMU_ERROR_EDMP_ODR if user should have called
 *                                       `inv_imu_edmp_recompute_apex_decimation()`
 *                other negative value on error.
 */
int inv_imu_edmp_enable_r2w(inv_imu_device_t *s);

/** @brief  Disable APEX algorithm R2W.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_disable_r2w(inv_imu_device_t *s);

/** @brief  Enable APEX algorithm Tap.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success
 *                INV_IMU_ERROR_EDMP_ODR if user should have called
 *                                       `inv_imu_edmp_recompute_apex_decimation()`
 *                other negative value on error.
 */
int inv_imu_edmp_enable_tap(inv_imu_device_t *s);

/** @brief  Disable APEX algorithm Tap.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_disable_tap(inv_imu_device_t *s);

/** @brief  Enable APEX algorithm Free Fall.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success
 *                INV_IMU_ERROR_EDMP_ODR if user should have called
 *                                       `inv_imu_edmp_recompute_apex_decimation()`
 *                other negative value on error.
 */
int inv_imu_edmp_enable_ff(inv_imu_device_t *s);

/** @brief  Disable APEX algorithm Free Fall.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_disable_ff(inv_imu_device_t *s);

/** @brief Read APEX interrupt status.
 *  @param[in] s    Pointer to device.
 *  @param[out] it  Status of each APEX interrupt.
 *  @return         0 on success, negative value on error.
 */
int inv_imu_edmp_get_int_apex_status(inv_imu_device_t *s, inv_imu_edmp_int_state_t *it);

/** @brief Retrieve pedometer outputs.
 *  @param[in] s      Pointer to device.
 *  @param[out] data  Pedometer step count and activity data value.
 *  @return           0 on success, negative value on error.
 *  @retval           INV_IMU_ERROR_EDMP_BUF_EMPTY if step count buffer is empty.
 */
int inv_imu_edmp_get_pedometer_data(inv_imu_device_t *s, inv_imu_edmp_pedometer_data_t *data);

/** @brief Retrieve APEX free fall outputs and format them.
 *  @param[in] s                   Pointer to device.
 *  @param[out] freefall_duration  Duration in number of sample.
 *  @return                        0 on success, negative value on error.
 */
int inv_imu_edmp_get_ff_data(inv_imu_device_t *s, uint16_t *freefall_duration);

/** @brief Retrieve tap outputs.
 *  @param[in] s      Pointer to device.
 *  @param[out] data  Tap number and direction.
 *  @return           0 on success, negative value on error.
 */
int inv_imu_edmp_get_tap_data(inv_imu_device_t *s, inv_imu_edmp_tap_data_t *data);

/** @brief  Mask requested interrupt sources for edmp interrupt line passed in parameter.
 *  @param[in] s            Pointer to device.
 *  @param[in] edmp_int_nb  EDMP input interrupt line number that should be configured.
 *  @param[in] int_mask     Interrupt sources to mask.
 *  @return                 0 on success, negative value on error.
 */
int inv_imu_edmp_mask_int_src(inv_imu_device_t *s, inv_imu_edmp_int_t edmp_int_nb,
                              uint8_t int_mask);

/** @brief  Unmask requested interrupt sources for edmp interrupt line passed in parameter.
 *  @param[in] s            Pointer to device.
 *  @param[in] edmp_int_nb  EDMP input interrupt line number that should be configured.
 *  @param[in] int_mask     Interrupt sources to unmask. 
 *  @return                 0 on success, negative value on error.
 */
int inv_imu_edmp_unmask_int_src(inv_imu_device_t *s, inv_imu_edmp_int_t edmp_int_nb,
                                uint8_t int_mask);

/** @brief  Setup EDMP to execute code in ROM.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_configure(inv_imu_device_t *s);

/** @brief Run EDMP using the on-demand mechanism.
 *  @param[in] s            Pointer to device.
 *  @param[in] edmp_int_nb  EDMP input interrupt line.
 *  @return                 0 on success, negative value on error.
 */
int inv_imu_edmp_run_ondemand(inv_imu_device_t *s, inv_imu_edmp_int_t edmp_int_nb);

/** @brief Wait until EDMP idle bit is set (means EDMP execution is completed).
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_wait_for_idle(inv_imu_device_t *s);

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_EDMP_H_ */

/** @} */
