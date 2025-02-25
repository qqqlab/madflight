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

#include "./inv_imu_driver_aux1.h"

#if INV_IMU_AUX1_SUPPORTED

int inv_imu_set_aux1_accel_mode(inv_imu_transport_t *t, uint8_t en)
{
	int             status = INV_IMU_OK;
	pwr_mgmt_aux1_t pwr_mgmt_aux1;

	status |= inv_imu_read_reg(t, PWR_MGMT_AUX1, 1, (uint8_t *)&pwr_mgmt_aux1);
	pwr_mgmt_aux1.accel_aux1_en = en;
	status |= inv_imu_write_reg(t, PWR_MGMT_AUX1, 1, (uint8_t *)&pwr_mgmt_aux1);

	return status;
}

int inv_imu_set_aux1_drdy(inv_imu_transport_t *t, uint8_t en, inv_imu_int_num_t it)
{
	int            status = INV_IMU_OK;
	int1_config0_t int1_config0;
	uint32_t       reg = (it == INV_IMU_INT1) ? INT1_CONFIG0 : INT2_CONFIG0;

	/* Use `int1_config0_t` for both INT1 and INT2 as bit location are the same */
	status |= inv_imu_read_reg(t, reg, 1, (uint8_t *)&int1_config0);
	int1_config0.int1_status_en_aux1_drdy = en;
	status |= inv_imu_write_reg(t, reg, 1, (uint8_t *)&int1_config0);

	return status;
}

int inv_imu_get_aux1_int_status(inv_imu_transport_t *t, inv_imu_aux1_int_state_t *it)
{
	int            status = INV_IMU_OK;
	intx_statusx_t int_status;

	/* Read AUX1 interrupt status */
	status |= inv_imu_read_reg(t, INT2_STATUS0, 2, (uint8_t *)&int_status);

	it->INV_OIS1         = int_status.int1_status0.int1_status_aux1_drdy;
	it->INV_OIS1_AGC_RDY = int_status.int1_status0.int1_status_aux1_agc_rdy;

	return status;
}

int inv_imu_set_aux1_accel_fsr(inv_imu_transport_t *t, fs_sel_aux_accel_fs_sel_t fsr)
{
	int           status = INV_IMU_OK;
	fs_sel_aux1_t fs_sel_aux1;

	status |= inv_imu_read_reg(t, FS_SEL_AUX1, 1, (uint8_t *)&fs_sel_aux1);
	fs_sel_aux1.accel_aux1_fs_sel = fsr;
	status |= inv_imu_write_reg(t, FS_SEL_AUX1, 1, (uint8_t *)&fs_sel_aux1);

	return status;
}

int inv_imu_get_aux1_accel_fsr(inv_imu_transport_t *t, fs_sel_aux_accel_fs_sel_t *fsr)
{
	int           status = INV_IMU_OK;
	fs_sel_aux1_t fs_sel_aux1;

	status |= inv_imu_read_reg(t, FS_SEL_AUX1, 1, (uint8_t *)&fs_sel_aux1);
	*fsr = (fs_sel_aux_accel_fs_sel_t)fs_sel_aux1.accel_aux1_fs_sel;

	return status;
}

int inv_imu_set_aux1_gyro_mode(inv_imu_transport_t *t, uint8_t en)
{
	int             status = INV_IMU_OK;
	pwr_mgmt_aux1_t pwr_mgmt_aux1;

	status |= inv_imu_read_reg(t, PWR_MGMT_AUX1, 1, (uint8_t *)&pwr_mgmt_aux1);
	pwr_mgmt_aux1.gyro_aux1_en = en;
	status |= inv_imu_write_reg(t, PWR_MGMT_AUX1, 1, (uint8_t *)&pwr_mgmt_aux1);

	return status;
}

int inv_imu_set_aux1_gyro_fsr(inv_imu_transport_t *t, fs_sel_aux_gyro_fs_sel_t fsr)
{
	int           status = INV_IMU_OK;
	fs_sel_aux1_t fs_sel_aux1;

	status |= inv_imu_read_reg(t, FS_SEL_AUX1, 1, (uint8_t *)&fs_sel_aux1);
	fs_sel_aux1.gyro_aux1_fs_sel = fsr;
	status |= inv_imu_write_reg(t, FS_SEL_AUX1, 1, (uint8_t *)&fs_sel_aux1);

	return status;
}

int inv_imu_get_aux1_gyro_fsr(inv_imu_transport_t *t, fs_sel_aux_gyro_fs_sel_t *fsr)
{
	int           status = INV_IMU_OK;
	fs_sel_aux1_t fs_sel_aux1;

	status |= inv_imu_read_reg(t, FS_SEL_AUX1, 1, (uint8_t *)&fs_sel_aux1);
	*fsr = (fs_sel_aux_gyro_fs_sel_t)fs_sel_aux1.gyro_aux1_fs_sel;

	return status;
}

int inv_imu_get_aux1_register_data(inv_imu_transport_t *t, inv_imu_sensor_data_t *sensor_data)
{
	int status = INV_IMU_OK;

	status |= inv_imu_read_reg(t, ACCEL_DATA_X1_AUX1, sizeof(inv_imu_sensor_data_t),
	                           (uint8_t *)sensor_data);
	return status;
}

#if INV_IMU_INT2_PIN_SUPPORTED
int inv_imu_set_aux1_pin_config_int(inv_imu_transport_t *t, const inv_imu_int_pin_config_t *conf)
{
	int            status = INV_IMU_OK;
	int1_config2_t int2_config2;

	status |= inv_imu_read_reg(t, INT2_CONFIG2, 1, (uint8_t *)&int2_config2);

	/* Use `int1_config2_t` for both INT1 and INT2 as bit location are the same */
	int2_config2.int1_polarity = conf->int_polarity;
	int2_config2.int1_mode     = conf->int_mode;
	int2_config2.int1_drive    = conf->int_drive;

	status |= inv_imu_write_reg(t, INT2_CONFIG2, 1, (uint8_t *)&int2_config2);

	return status;
}
#endif

#endif
