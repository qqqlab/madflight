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

#include "./inv_imu_selftest.h"
#include "./inv_imu_edmp.h"

/* Static functions definition */
static int set_selftest_parameters(inv_imu_device_t *                   s,
                                   const inv_imu_selftest_parameters_t *st_params);
static int run_internal_selftest(inv_imu_device_t *s);
static int get_selftest_output(inv_imu_device_t *s, const inv_imu_selftest_parameters_t *st_params,
                               inv_imu_selftest_output_t *st_output);

/* API implementation */

int inv_imu_selftest_init_params(inv_imu_device_t *s, inv_imu_selftest_parameters_t *st_params)
{
	int rc = INV_IMU_OK;

	st_params->accel_en       = INV_IMU_ENABLE;
	st_params->gyro_en        = INV_IMU_ENABLE;
	st_params->avg_time       = SELFTEST_AVG_TIME_320_MS;
	st_params->accel_limit    = SELFTEST_ACCEL_THRESHOLD_50_PERCENT;
	st_params->gyro_limit     = SELFTEST_GYRO_THRESHOLD_50_PERCENT;
	st_params->patch_settings = 0;

	return rc;
}

int inv_imu_selftest(inv_imu_device_t *s, const inv_imu_selftest_parameters_t *st_params,
                     inv_imu_selftest_output_t *st_output)
{
	int rc = INV_IMU_OK;

	if (!(st_params->accel_en || st_params->gyro_en))
		return INV_IMU_ERROR;

	rc |= inv_imu_adv_device_reset(s);

	inv_imu_sleep_us(s, 10000);

	/* Configure start addresses as we reset the device */
	rc |= inv_imu_edmp_configure(s);

	rc |= set_selftest_parameters(s, st_params);

	rc |= run_internal_selftest(s);

	rc |= get_selftest_output(s, st_params, st_output);

	rc |= inv_imu_adv_device_reset(s);

	return rc;
}

/* Static functions implementation */

static int set_selftest_parameters(inv_imu_device_t *                   s,
                                   const inv_imu_selftest_parameters_t *st_params)
{
	int      rc             = INV_IMU_OK;
	uint32_t tmp_stc_params = 0;
	int      init_en;

	rc |= inv_imu_adv_power_up_sram(s);

	init_en = (st_params->accel_en || st_params->gyro_en);
	tmp_stc_params |= (init_en ? SELFTESTCAL_INIT_EN : SELFTESTCAL_INIT_DIS);
	tmp_stc_params |= (st_params->accel_en ? SELFTEST_ACCEL_EN : SELFTEST_ACCEL_DIS);
	tmp_stc_params |= (st_params->gyro_en ? SELFTEST_GYRO_EN : SELFTEST_GYRO_DIS);
	tmp_stc_params |= (uint32_t)(st_params->accel_limit & SELFTEST_ACCEL_THRESH_MASK);
	tmp_stc_params |= (uint32_t)(st_params->gyro_limit & SELFTEST_GYRO_THRESH_MASK);
	tmp_stc_params |= (uint32_t)(st_params->avg_time & SELFTEST_AVERAGE_TIME_MASK);
	rc |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_STC_CONFIGPARAMS, (uint8_t *)&tmp_stc_params);

	tmp_stc_params = 0; /* disable any selftest stop point */
	rc |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_STC_DEBUG_EN, (uint8_t *)&tmp_stc_params);

	rc |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_STC_PATCH_EN, (uint8_t *)&(st_params->patch_settings));

	return rc;
}

static int run_internal_selftest(inv_imu_device_t *s)
{
	int                rc = INV_IMU_OK;
	reg_host_msg_t     reg_host_msg;
	int_apex_config1_t int_apex_config1;
	int                timeout_us = 3000000; /* 3 seconds */

	rc |= inv_imu_read_reg(s, REG_HOST_MSG, 1, (uint8_t *)&reg_host_msg);
	reg_host_msg.testopenable = INV_IMU_ENABLE;
	rc |= inv_imu_write_reg(s, REG_HOST_MSG, 1, (uint8_t *)&reg_host_msg);

	/* Enable desired interrupt */
	rc |= inv_imu_read_reg(s, INT_APEX_CONFIG1, 1, (uint8_t *)&int_apex_config1);
	int_apex_config1.int_status_mask_pin_selftest_done = 0;
	rc |= inv_imu_write_reg(s, INT_APEX_CONFIG1, 1, (uint8_t *)&int_apex_config1);

	/* Run EDMP */
	rc |= inv_imu_edmp_run_ondemand(s, INV_IMU_EDMP_INT2);

	/* Wait for the desired interrupt */
	while (1) {
		int_apex_status1_t int_apex_status1;
		rc |= inv_imu_read_reg(s, INT_APEX_STATUS1, 1, (uint8_t *)&int_apex_status1);
		if (int_apex_status1.int_status_selftest_done)
			break;

		inv_imu_sleep_us(s, 100);
		timeout_us -= 100;

		if (timeout_us <= 0)
			return INV_IMU_ERROR_TIMEOUT;
	}

	/* Disable interrupt */
	rc |= inv_imu_read_reg(s, INT_APEX_CONFIG1, 1, (uint8_t *)&int_apex_config1);
	int_apex_config1.int_status_mask_pin_selftest_done = 1;
	rc |= inv_imu_write_reg(s, INT_APEX_CONFIG1, 1, (uint8_t *)&int_apex_config1);

	return rc;
}

static int get_selftest_output(inv_imu_device_t *s, const inv_imu_selftest_parameters_t *st_params,
                               inv_imu_selftest_output_t *st_output)
{
	int      rc = INV_IMU_OK;
	uint32_t stc_results;

	/* Read STC results */
	rc |= INV_IMU_READ_EDMP_SRAM(s, EDMP_STC_RESULTS, (uint8_t *)&stc_results);

	if (!st_params->accel_en) {
		st_output->accel_status = INV_IMU_ST_STATUS_NOT_RUN;
		st_output->ax_status    = INV_IMU_ST_STATUS_NOT_RUN;
		st_output->ay_status    = INV_IMU_ST_STATUS_NOT_RUN;
		st_output->az_status    = INV_IMU_ST_STATUS_NOT_RUN;
	} else {
		st_output->accel_status =
		    ((stc_results & (STC_RESULTS_ACCEL_X_MASK | STC_RESULTS_ACCEL_Y_MASK |
		                     STC_RESULTS_ACCEL_Z_MASK | STC_RESULTS_ST_STATUS_MASK)) == 0) ?
                INV_IMU_ST_STATUS_SUCCESS :
                INV_IMU_ST_STATUS_FAIL;
		st_output->ax_status = ((stc_results & STC_RESULTS_ACCEL_X_MASK) == 0) ?
                                   INV_IMU_ST_STATUS_SUCCESS :
                                   INV_IMU_ST_STATUS_FAIL;
		st_output->ay_status = ((stc_results & STC_RESULTS_ACCEL_Y_MASK) == 0) ?
                                   INV_IMU_ST_STATUS_SUCCESS :
                                   INV_IMU_ST_STATUS_FAIL;
		st_output->az_status = ((stc_results & STC_RESULTS_ACCEL_Z_MASK) == 0) ?
                                   INV_IMU_ST_STATUS_SUCCESS :
                                   INV_IMU_ST_STATUS_FAIL;
	}

	if (!st_params->gyro_en) {
		st_output->gyro_status = INV_IMU_ST_STATUS_NOT_RUN;
		st_output->gx_status   = INV_IMU_ST_STATUS_NOT_RUN;
		st_output->gy_status   = INV_IMU_ST_STATUS_NOT_RUN;
		st_output->gz_status   = INV_IMU_ST_STATUS_NOT_RUN;
	} else {
		st_output->gyro_status =
		    ((stc_results & (STC_RESULTS_GYRO_X_MASK | STC_RESULTS_GYRO_Y_MASK |
		                     STC_RESULTS_GYRO_Z_MASK | STC_RESULTS_ST_STATUS_MASK)) == 0) ?
                INV_IMU_ST_STATUS_SUCCESS :
                INV_IMU_ST_STATUS_FAIL;
		st_output->gx_status = ((stc_results & STC_RESULTS_GYRO_X_MASK) == 0) ?
                                   INV_IMU_ST_STATUS_SUCCESS :
                                   INV_IMU_ST_STATUS_FAIL;
		st_output->gy_status = ((stc_results & STC_RESULTS_GYRO_Y_MASK) == 0) ?
                                   INV_IMU_ST_STATUS_SUCCESS :
                                   INV_IMU_ST_STATUS_FAIL;
		st_output->gz_status = ((stc_results & STC_RESULTS_GYRO_Z_MASK) == 0) ?
                                   INV_IMU_ST_STATUS_SUCCESS :
                                   INV_IMU_ST_STATUS_FAIL;
	}

	return rc;
}
