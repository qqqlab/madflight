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

/** @defgroup selftest Self-test
 *  @brief    API to execute self-test procedure.
 *  @{
 */

/** @file inv_imu_selftest.h */

#ifndef _INV_IMU_SELFTEST_H_
#define _INV_IMU_SELFTEST_H_

#include <stdint.h>

#include "./inv_imu_driver_advanced.h"

#ifdef __cplusplus
extern "C" {
#endif

/* STC status codes */
#define INV_IMU_ST_STATUS_SUCCESS 1 /**< Indicates test is successful */
#define INV_IMU_ST_STATUS_FAIL    -1 /**< Indicates test is failing */
#define INV_IMU_ST_STATUS_NOT_RUN 0 /**< Indicates test has not run */

/** Self-Test parameters */
typedef struct {
	/** If set, enable accel self-test */
	uint8_t accel_en;

	/** If set, enable gyro self-test */
	uint8_t gyro_en;

	/** Averaging time used to perform self-test */
	selftest_average_time_t avg_time;

	/** Tolerance between factory trim and accel self-test response */
	selftest_accel_threshold_t accel_limit;

	/** Tolerance between factory trim and gyro self-test response */
	selftest_gyro_threshold_t gyro_limit;

	/** Mechanism for adding patches to self-test operations */
	uint32_t patch_settings;
} inv_imu_selftest_parameters_t;

/** Self-test outputs */
typedef struct {
	/** Global accel self-test status. 1 for success, 0 otherwise. */
	int8_t accel_status;

	/** Global gyro self-test status. 1 for success, 0 otherwise. */
	int8_t gyro_status;

	/** AX self-test status. 1 for success, 0 otherwise. */
	int8_t ax_status;

	/** AY self-test status. 1 for success, 0 otherwise. */
	int8_t ay_status;

	/** AZ self-test status. 1 for success, 0 otherwise. */
	int8_t az_status;

	/** GX self-test status. 1 for success, 0 otherwise. */
	int8_t gx_status;

	/** GY self-test status. 1 for success, 0 otherwise. */
	int8_t gy_status;

	/** GZ self-test status. 1 for success, 0 otherwise. */
	int8_t gz_status;
} inv_imu_selftest_output_t;

/** @brief Provide recommended parameters to execute self-test.
 *  @param[in] s          Pointer to device.
 *  @param[in] st_params  Structure filled with recommended params.
 *  @return               0 on success, negative value on error.
 */
int inv_imu_selftest_init_params(inv_imu_device_t *s, inv_imu_selftest_parameters_t *st_params);

/** @brief Perform hardware self-test for Accel and/or Gyro.
 *  @param[in] s            Pointer to device.
 *  @param[in] st_params    Self-test parameters to be used.
 *  @param[out] st_output   Output from Self-test operation.
 *  @return                 0 on success, negative value on error.
 */
int inv_imu_selftest(inv_imu_device_t *s, const inv_imu_selftest_parameters_t *st_params,
                     inv_imu_selftest_output_t *st_output);

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_SELFTEST_H_ */

/** @} */
