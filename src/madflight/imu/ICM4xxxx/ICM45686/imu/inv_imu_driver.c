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

#include "./inv_imu_driver.h"
#include "./inv_imu_version.h"

void inv_imu_sleep_us(inv_imu_device_t *s, uint32_t us)
{
	if (s->transport.sleep_us != NULL)
		s->transport.sleep_us(us);
}

int inv_imu_soft_reset(inv_imu_device_t *s)
{
	int                 status = INV_IMU_OK;
	intf_config1_ovrd_t intf_config1_ovrd;
	drive_config0_t     drive_config0;
	reg_misc2_t         reg_misc2 = { 0 };
	int1_status0_t      int1_status0;

	/* Save INTF_CONFIG1_OVRD register */
	status |= inv_imu_read_reg(s, INTF_CONFIG1_OVRD, 1, (uint8_t *)&intf_config1_ovrd);
	/* Save DRIVE_CONFIG0 register */
	status |= inv_imu_read_reg(s, DRIVE_CONFIG0, 1, (uint8_t *)&drive_config0);

	/* Trigger soft reset */
	reg_misc2.soft_rst = 1;
	status |= inv_imu_write_reg(s, REG_MISC2, 1, (uint8_t *)&reg_misc2);

	/* Wait 1 ms for soft reset to be effective */
	inv_imu_sleep_us(s, 1000);

	/* Restore DRIVE_CONFIG0 register */
	status |= inv_imu_write_reg(s, DRIVE_CONFIG0, 1, (uint8_t *)&drive_config0);
	/* Restore INTF_CONFIG1_OVRD register */
	status |= inv_imu_write_reg(s, INTF_CONFIG1_OVRD, 1, (uint8_t *)&intf_config1_ovrd);

	/* Clear the RESET_DONE interrupt */
	status |= inv_imu_read_reg(s, INT1_STATUS0, 1, (uint8_t *)&int1_status0);
	if (int1_status0.int1_status_reset_done != 1)
		return INV_IMU_ERROR; /* Return an error if RESET_DONE is not set */

	return status;
}

int inv_imu_get_who_am_i(inv_imu_device_t *s, uint8_t *who_am_i)
{
	int status;

	status = inv_imu_read_reg(s, WHO_AM_I, 1, who_am_i);

	/* AN-000364
	 * In I2C mode, after chip power-up, the host should perform one retry
	 * on the very first I2C transaction if it receives a NACK 
	 */
	if (s->transport.serif_type == UI_I2C && status)
		status = inv_imu_read_reg(s, WHO_AM_I, 1, who_am_i);

	return status;
}

int inv_imu_set_accel_mode(inv_imu_device_t *s, pwr_mgmt0_accel_mode_t accel_mode)
{
	int         status = INV_IMU_OK;
	pwr_mgmt0_t pwr_mgmt0;

	status |= inv_imu_get_endianness(s);

	status |= inv_imu_read_reg(s, PWR_MGMT0, 1, (uint8_t *)&pwr_mgmt0);
	pwr_mgmt0.accel_mode = accel_mode;
	status |= inv_imu_write_reg(s, PWR_MGMT0, 1, (uint8_t *)&pwr_mgmt0);

	return status;
}

int inv_imu_set_gyro_mode(inv_imu_device_t *s, pwr_mgmt0_gyro_mode_t gyro_mode)
{
	int         status = INV_IMU_OK;
	pwr_mgmt0_t pwr_mgmt0;

	status |= inv_imu_get_endianness(s);

	status |= inv_imu_read_reg(s, PWR_MGMT0, 1, (uint8_t *)&pwr_mgmt0);
	pwr_mgmt0.gyro_mode = gyro_mode;
	status |= inv_imu_write_reg(s, PWR_MGMT0, 1, (uint8_t *)&pwr_mgmt0);

	return status;
}

int inv_imu_set_accel_frequency(inv_imu_device_t *s, const accel_config0_accel_odr_t frequency)
{
	int             status = INV_IMU_OK;
	accel_config0_t accel_config0;

	status |= inv_imu_read_reg(s, ACCEL_CONFIG0, 1, (uint8_t *)&accel_config0);
	accel_config0.accel_odr = frequency;
	status |= inv_imu_write_reg(s, ACCEL_CONFIG0, 1, (uint8_t *)&accel_config0);

	return status;
}

int inv_imu_set_gyro_frequency(inv_imu_device_t *s, const gyro_config0_gyro_odr_t frequency)
{
	int            status = INV_IMU_OK;
	gyro_config0_t gyro_config0;

	status |= inv_imu_read_reg(s, GYRO_CONFIG0, 1, (uint8_t *)&gyro_config0);
	gyro_config0.gyro_odr = frequency;
	status |= inv_imu_write_reg(s, GYRO_CONFIG0, 1, (uint8_t *)&gyro_config0);

	return status;
}

int inv_imu_set_accel_fsr(inv_imu_device_t *s, accel_config0_accel_ui_fs_sel_t accel_fsr)
{
	int             status = INV_IMU_OK;
	accel_config0_t accel_config0;

	status |= inv_imu_read_reg(s, ACCEL_CONFIG0, 1, (uint8_t *)&accel_config0);
	accel_config0.accel_ui_fs_sel = accel_fsr;
	status |= inv_imu_write_reg(s, ACCEL_CONFIG0, 1, (uint8_t *)&accel_config0);

	return status;
}

int inv_imu_set_gyro_fsr(inv_imu_device_t *s, gyro_config0_gyro_ui_fs_sel_t gyro_fsr)
{
	int            status = INV_IMU_OK;
	gyro_config0_t gyro_config0;

	status |= inv_imu_read_reg(s, GYRO_CONFIG0, 1, (uint8_t *)&gyro_config0);
	gyro_config0.gyro_ui_fs_sel = gyro_fsr;
	status |= inv_imu_write_reg(s, GYRO_CONFIG0, 1, (uint8_t *)&gyro_config0);

	return status;
}

int inv_imu_set_gyro_ln_bw(inv_imu_device_t *s, ipreg_sys1_reg_172_gyro_ui_lpfbw_sel_t gyr_bw)
{
	int                  status = INV_IMU_OK;
	ipreg_sys1_reg_172_t ipreg_sys1_reg_172;

	status |= inv_imu_read_reg(s, IPREG_SYS1_REG_172, 1, (uint8_t *)&ipreg_sys1_reg_172);
	ipreg_sys1_reg_172.gyro_ui_lpfbw_sel = gyr_bw;
	status |= inv_imu_write_reg(s, IPREG_SYS1_REG_172, 1, (uint8_t *)&ipreg_sys1_reg_172);

	return status;
}

int inv_imu_set_gyro_lp_avg(inv_imu_device_t *s, ipreg_sys1_reg_170_gyro_lp_avg_sel_t gyr_avg)
{
	int                  status = INV_IMU_OK;
	ipreg_sys1_reg_170_t ipreg_sys1_reg_170;

	status |= inv_imu_read_reg(s, IPREG_SYS1_REG_170, 1, (uint8_t *)&ipreg_sys1_reg_170);
	ipreg_sys1_reg_170.gyro_lp_avg_sel = gyr_avg;
	status |= inv_imu_write_reg(s, IPREG_SYS1_REG_170, 1, (uint8_t *)&ipreg_sys1_reg_170);
	return status;
}

int inv_imu_set_accel_lp_avg(inv_imu_device_t *s, ipreg_sys2_reg_129_accel_lp_avg_sel_t acc_avg)
{
	int                  status = INV_IMU_OK;
	ipreg_sys2_reg_129_t ipreg_sys2_reg_129;

	status |= inv_imu_read_reg(s, IPREG_SYS2_REG_129, 1, (uint8_t *)&ipreg_sys2_reg_129);
	ipreg_sys2_reg_129.accel_lp_avg_sel = acc_avg;
	status |= inv_imu_write_reg(s, IPREG_SYS2_REG_129, 1, (uint8_t *)&ipreg_sys2_reg_129);

	return status;
}

int inv_imu_set_accel_ln_bw(inv_imu_device_t *s, ipreg_sys2_reg_131_accel_ui_lpfbw_t acc_bw)
{
	int                  status = INV_IMU_OK;
	ipreg_sys2_reg_131_t ipreg_sys2_reg_131;

	status |= inv_imu_read_reg(s, IPREG_SYS2_REG_131, 1, (uint8_t *)&ipreg_sys2_reg_131);
	ipreg_sys2_reg_131.accel_ui_lpfbw_sel = acc_bw;
	status |= inv_imu_write_reg(s, IPREG_SYS2_REG_131, 1, (uint8_t *)&ipreg_sys2_reg_131);

	return status;
}

int inv_imu_get_register_data(inv_imu_device_t *s, inv_imu_sensor_data_t *data)
{
	int status = INV_IMU_OK;

	status |= inv_imu_read_reg(s, ACCEL_DATA_X1_UI, sizeof(inv_imu_sensor_data_t), (uint8_t *)data);

	/* Format accel data from sensor registers. */
	FORMAT_16_BITS_DATA(s->endianness_data, (uint8_t *)&data->accel_data[0],
	                    (uint16_t *)&data->accel_data[0]);
	FORMAT_16_BITS_DATA(s->endianness_data, (uint8_t *)&data->accel_data[1],
	                    (uint16_t *)&data->accel_data[1]);
	FORMAT_16_BITS_DATA(s->endianness_data, (uint8_t *)&data->accel_data[2],
	                    (uint16_t *)&data->accel_data[2]);
	/* Format gyro data from sensor registers. */
	FORMAT_16_BITS_DATA(s->endianness_data, (uint8_t *)&data->gyro_data[0],
	                    (uint16_t *)&data->gyro_data[0]);
	FORMAT_16_BITS_DATA(s->endianness_data, (uint8_t *)&data->gyro_data[1],
	                    (uint16_t *)&data->gyro_data[1]);
	FORMAT_16_BITS_DATA(s->endianness_data, (uint8_t *)&data->gyro_data[2],
	                    (uint16_t *)&data->gyro_data[2]);
	/* Format temperature data from sensor registers. */
	FORMAT_16_BITS_DATA(s->endianness_data, (uint8_t *)&data->temp_data,
	                    (uint16_t *)&data->temp_data);

	return status;
}

int inv_imu_set_fifo_config(inv_imu_device_t *s, const inv_imu_fifo_config_t *fifo_config)
{
	int             status = INV_IMU_OK;
	fifo_configx_t  cfg;
	smc_control_0_t smc_control_0;
	uint8_t         fifo_frame_size = 0;

	/* `fifo_depth` must be a valid value. */
	if (fifo_config->fifo_depth != FIFO_CONFIG0_FIFO_DEPTH_MAX &&
	    fifo_config->fifo_depth != FIFO_CONFIG0_FIFO_DEPTH_APEX)
		return INV_IMU_ERROR_BAD_ARG;

	status |= inv_imu_read_reg(s, FIFO_CONFIG0, 6, (uint8_t *)&cfg);

	/* Disable FIFO to safely apply configuration */
	cfg.fifo_config3.fifo_if_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, FIFO_CONFIG3, 1, (uint8_t *)&cfg.fifo_config3);
	cfg.fifo_config0.fifo_mode = FIFO_CONFIG0_FIFO_MODE_BYPASS;
	status |= inv_imu_write_reg(s, FIFO_CONFIG0, 1, (uint8_t *)&cfg.fifo_config0);

	/* Set FIFO depth */
	cfg.fifo_config0.fifo_depth = fifo_config->fifo_depth;

	/* Set WM */
	cfg.fifo_config1_0 = (uint8_t)fifo_config->fifo_wm_th;
	cfg.fifo_config1_1 = (uint8_t)(fifo_config->fifo_wm_th >> 8);

	/* Set fifo WM triggering condition: interrupts when equal or greater than threshold */
	cfg.fifo_config2.fifo_wr_wm_gt_th = FIFO_CONFIG2_FIFO_WR_WM_EQ_OR_GT_TH;

	/* Set which sensors go to FIFO */
	cfg.fifo_config3.fifo_hires_en = fifo_config->hires_en;
	cfg.fifo_config3.fifo_gyro_en  = fifo_config->gyro_en;
	cfg.fifo_config3.fifo_accel_en = fifo_config->accel_en;

	/* Enable timestamp in FIFO if 16 or 20 bytes mode */
	status |= inv_imu_read_reg(s, SMC_CONTROL_0, 1, (uint8_t *)&smc_control_0);
	if ((fifo_config->fifo_mode != FIFO_CONFIG0_FIFO_MODE_BYPASS) &&
	    ((fifo_config->accel_en && fifo_config->gyro_en) || fifo_config->hires_en)) {
		/* 16 or 20 bytes mode */
		cfg.fifo_config4.fifo_tmst_fsync_en = INV_IMU_ENABLE;
		smc_control_0.tmst_en               = INV_IMU_ENABLE;
	} else {
		/* 8 bytes mode */
		cfg.fifo_config4.fifo_tmst_fsync_en = INV_IMU_DISABLE;
		smc_control_0.tmst_en               = INV_IMU_DISABLE;
	}
	status |= inv_imu_write_reg(s, SMC_CONTROL_0, 1, (uint8_t *)&smc_control_0);

	/* Apply configuration */
	status |= inv_imu_write_reg(s, FIFO_CONFIG0, 6, (uint8_t *)&cfg);

	/* Set expected fifo_mode */
	cfg.fifo_config0.fifo_mode = fifo_config->fifo_mode;

	if (fifo_config->fifo_mode == FIFO_CONFIG0_FIFO_MODE_BYPASS) {
		/* 
		 * Disabling FIFO:
		 *  - Set `fifo_if_en` to 0
		 *  - Set `fifo_mode` to BYPASS
		 */
		/* `fifo_if_en` */
		cfg.fifo_config3.fifo_if_en = INV_IMU_DISABLE;
		status |= inv_imu_write_reg(s, FIFO_CONFIG3, 1, (uint8_t *)&cfg.fifo_config3);
		/* `fifo_mode` */
		status |= inv_imu_write_reg(s, FIFO_CONFIG0, 1, (uint8_t *)&cfg.fifo_config0);
	} else {
		/* 
		 * Enabling FIFO:
		 *  - Set `fifo_mode`
		 *  - Set `fifo_if_en` to 1
		 */
		/* `fifo_mode` */
		status |= inv_imu_write_reg(s, FIFO_CONFIG0, 1, (uint8_t *)&cfg.fifo_config0);
		/* `fifo_if_en` */
		cfg.fifo_config3.fifo_if_en = INV_IMU_ENABLE;
		status |= inv_imu_write_reg(s, FIFO_CONFIG3, 1, (uint8_t *)&cfg.fifo_config3);
	}

	/* Calculate FIFO frame size */
	if (fifo_config->hires_en) {
		fifo_frame_size = 20;
	} else {
		if (fifo_config->accel_en)
			fifo_frame_size += 8;
		if (fifo_config->gyro_en)
			fifo_frame_size += 8;
	}

	s->fifo_frame_size = fifo_frame_size;

	return status;
}

int inv_imu_get_fifo_config(inv_imu_device_t *s, inv_imu_fifo_config_t *fifo_config)
{
	int            status = INV_IMU_OK;
	fifo_configx_t cfg;

	status |= inv_imu_read_reg(s, FIFO_CONFIG0, 5, (uint8_t *)&cfg);

	/* FIFO_CONFIG0 */
	fifo_config->fifo_mode = (fifo_config0_fifo_mode_t)cfg.fifo_config0.fifo_mode;
	if (cfg.fifo_config0.fifo_depth == FIFO_CONFIG0_FIFO_DEPTH_MAX ||
	    cfg.fifo_config0.fifo_depth == FIFO_CONFIG0_FIFO_DEPTH_APEX) {
		/* `fifo_depth` is valid, return it */
		fifo_config->fifo_depth = (fifo_config0_fifo_depth_t)cfg.fifo_config0.fifo_depth;
	} else {
		/*
		 * `fifo_depth` is invalid so force it to FIFO_CONFIG0_FIFO_DEPTH_APEX.
		 * It will be eventually applied when calling `inv_imu_set_fifo_config`.
		 */
		fifo_config->fifo_depth = FIFO_CONFIG0_FIFO_DEPTH_APEX;
	}

	/* FIFO_CONFIG1_0 and FIFO_CONFIG1_1*/
	fifo_config->fifo_wm_th = (uint16_t)((uint16_t)cfg.fifo_config1_1 << 8) | cfg.fifo_config1_0;

	/* FIFO_CONFIG3 */
	fifo_config->hires_en = cfg.fifo_config3.fifo_hires_en;
	fifo_config->gyro_en  = cfg.fifo_config3.fifo_gyro_en;
	fifo_config->accel_en = cfg.fifo_config3.fifo_accel_en;

	return status;
}

int inv_imu_flush_fifo(inv_imu_device_t *s)
{
	int            status = INV_IMU_OK;
	fifo_config2_t fifo_config2;
	int            timeout_us = 1000000; /* 1 sec */

	status |= inv_imu_read_reg(s, FIFO_CONFIG2, 1, (uint8_t *)&fifo_config2);
	fifo_config2.fifo_flush = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, FIFO_CONFIG2, 1, (uint8_t *)&fifo_config2);
	inv_imu_sleep_us(s, 10);

	/* Wait for FIFO flush (idle bit will go high at appropriate time and unlock flush) */
	while (status == INV_IMU_OK) {
		status |= inv_imu_read_reg(s, FIFO_CONFIG2, 1, (uint8_t *)&fifo_config2);
		if (fifo_config2.fifo_flush != INV_IMU_ENABLE)
			break;

		inv_imu_sleep_us(s, 100);
		timeout_us -= 100;

		if (timeout_us <= 0)
			return INV_IMU_ERROR_TIMEOUT;
	}

	return status;
}

int inv_imu_get_frame_count(inv_imu_device_t *s, uint16_t *frame_count)
{
	int status = INV_IMU_OK;

	status |= inv_imu_read_reg(s, FIFO_COUNT_0, 2, (uint8_t *)frame_count);

	/*
	 * Errata AN-000364 (2.2)
	 * Read FIFO_COUNT value twice, and use the second value.
	 */
	status |= inv_imu_read_reg(s, FIFO_COUNT_0, 2, (uint8_t *)frame_count);

	FORMAT_16_BITS_DATA(s->endianness_data, (uint8_t *)frame_count, frame_count);

	return status;
}

int inv_imu_get_fifo_frame(inv_imu_device_t *s, inv_imu_fifo_data_t *data)
{
	int     status = INV_IMU_OK;
	uint8_t frame[20];
	int16_t reg16[3];

	status |= inv_imu_read_reg(s, FIFO_DATA, s->fifo_frame_size, frame);

	data->header.Byte = frame[0];

	switch (s->fifo_frame_size) {
	case 8:
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[1], &data->byte_8.sensor_data[0]);
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[3], &data->byte_8.sensor_data[1]);
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[5], &data->byte_8.sensor_data[2]);
		data->byte_8.temp_data = frame[7]; // Temp data is only one byte
		break;
	case 16:
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[1], &data->byte_16.accel_data[0]);
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[3], &data->byte_16.accel_data[1]);
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[5], &data->byte_16.accel_data[2]);
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[7], &data->byte_16.gyro_data[0]);
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[9], &data->byte_16.gyro_data[1]);
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[11], &data->byte_16.gyro_data[2]);
		data->byte_16.temp_data = frame[13]; // Temp data is only one byte
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[14], &data->byte_16.timestamp);
		break;
	case 20:
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[1], &reg16[0]);
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[3], &reg16[1]);
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[5], &reg16[2]);
		data->byte_20.accel_data[0] = (reg16[0] << 4) | (frame[17] >> 4);
		data->byte_20.accel_data[1] = (reg16[1] << 4) | (frame[18] >> 4);
		data->byte_20.accel_data[2] = (reg16[2] << 4) | (frame[19] >> 4);
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[7], &reg16[0]);
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[9], &reg16[1]);
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[11], &reg16[2]);
		data->byte_20.gyro_data[0] = (reg16[0] << 4) | (frame[17] & 0x0F);
		data->byte_20.gyro_data[1] = (reg16[1] << 4) | (frame[18] & 0x0F);
		data->byte_20.gyro_data[2] = (reg16[2] << 4) | (frame[19] & 0x0F);
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[13], &data->byte_20.temp_data);
		FORMAT_16_BITS_DATA(s->endianness_data, &frame[15], &data->byte_20.timestamp);
		break;
	default:
		data->header.Byte = 0;
		return INV_IMU_ERROR;
	}

	return status;
}

int inv_imu_set_config_int(inv_imu_device_t *s, const inv_imu_int_num_t num,
                           const inv_imu_int_state_t *it)
{
	int            status = INV_IMU_OK;
	intx_configx_t intx_configx;
	uint32_t       reg;

	switch (num) {
	case INV_IMU_INT1:
		reg = INT1_CONFIG0;
		break;
	case INV_IMU_INT2:
		reg = INT2_CONFIG0;
		break;
	default:
		return INV_IMU_ERROR_BAD_ARG;
	}

	/* Use defines from INT1 as bit locations are the same */
	/* INTX_CONFIG0 */
	intx_configx.int1_config0.int1_status_en_fifo_full    = it->INV_FIFO_FULL;
	intx_configx.int1_config0.int1_status_en_fifo_ths     = it->INV_FIFO_THS;
	intx_configx.int1_config0.int1_status_en_drdy         = it->INV_UI_DRDY;
	intx_configx.int1_config0.int1_status_en_ap_fsync     = it->INV_UI_FSYNC;
	intx_configx.int1_config0.int1_status_en_ap_agc_rdy   = it->INV_AGC_RDY;
	intx_configx.int1_config0.int1_status_en_aux1_agc_rdy = it->INV_OIS1_AGC_RDY;
	intx_configx.int1_config0.int1_status_en_reset_done   = it->INV_RESET_DONE;

	/* INTX_CONFIG1 */
	intx_configx.int1_config1.int1_status_en_pll_rdy          = it->INV_PLL_RDY;
	intx_configx.int1_config1.int1_status_en_wom_x            = it->INV_WOM_X;
	intx_configx.int1_config1.int1_status_en_wom_y            = it->INV_WOM_Y;
	intx_configx.int1_config1.int1_status_en_wom_z            = it->INV_WOM_Z;
	intx_configx.int1_config1.int1_status_en_i3c_protocol_err = it->INV_I3C_PROT_ERR;
	intx_configx.int1_config1.int1_status_en_i2cm_done        = it->INV_I2CM_DONE;
	intx_configx.int1_config1.int1_status_en_apex_event       = it->INV_EDMP_EVENT;

	status |= inv_imu_write_reg(s, reg, 2, (uint8_t *)&intx_configx);

	return status;
}

int inv_imu_get_config_int(inv_imu_device_t *s, const inv_imu_int_num_t num,
                           inv_imu_int_state_t *it)
{
	int            status = INV_IMU_OK;
	intx_configx_t intx_configx;
	uint32_t       reg;

	switch (num) {
	case INV_IMU_INT1:
		reg = INT1_CONFIG0;
		break;
	case INV_IMU_INT2:
		reg = INT2_CONFIG0;
		break;
	default:
		return INV_IMU_ERROR_BAD_ARG;
	}

	status |= inv_imu_read_reg(s, reg, 2, (uint8_t *)&intx_configx);

	/* Use defines from INT1 as bit locations are the same */
	/* INTX_CONFIG0 */
	it->INV_FIFO_FULL    = intx_configx.int1_config0.int1_status_en_fifo_full;
	it->INV_FIFO_THS     = intx_configx.int1_config0.int1_status_en_fifo_ths;
	it->INV_UI_DRDY      = intx_configx.int1_config0.int1_status_en_drdy;
	it->INV_UI_FSYNC     = intx_configx.int1_config0.int1_status_en_ap_fsync;
	it->INV_AGC_RDY      = intx_configx.int1_config0.int1_status_en_ap_agc_rdy;
	it->INV_OIS1_AGC_RDY = intx_configx.int1_config0.int1_status_en_aux1_agc_rdy;
	it->INV_RESET_DONE   = intx_configx.int1_config0.int1_status_en_reset_done;

	/* INTX_CONFIG1 */
	it->INV_PLL_RDY      = intx_configx.int1_config1.int1_status_en_pll_rdy;
	it->INV_WOM_X        = intx_configx.int1_config1.int1_status_en_wom_x;
	it->INV_WOM_Y        = intx_configx.int1_config1.int1_status_en_wom_y;
	it->INV_WOM_Z        = intx_configx.int1_config1.int1_status_en_wom_z;
	it->INV_I3C_PROT_ERR = intx_configx.int1_config1.int1_status_en_i3c_protocol_err;
	it->INV_I2CM_DONE    = intx_configx.int1_config1.int1_status_en_i2cm_done;
	it->INV_EDMP_EVENT   = intx_configx.int1_config1.int1_status_en_apex_event;

	return status;
}

int inv_imu_set_pin_config_int(inv_imu_device_t *s, const inv_imu_int_num_t num,
                               const inv_imu_int_pin_config_t *conf)
{
	int            status = INV_IMU_OK;
	int1_config2_t int1_config2;
	uint32_t       reg;

	switch (num) {
	case INV_IMU_INT1:
		reg = INT1_CONFIG2;
		break;
	case INV_IMU_INT2:
		reg = INT2_CONFIG2;
		break;
	default:
		return INV_IMU_ERROR_BAD_ARG;
	}

	status |= inv_imu_read_reg(s, reg, 1, (uint8_t *)&int1_config2);

	/* Use `int1_config2_t` for both INT1 and INT2 as bit location are the same */
	int1_config2.int1_polarity = conf->int_polarity;
	int1_config2.int1_mode     = conf->int_mode;
	int1_config2.int1_drive    = conf->int_drive;

	status |= inv_imu_write_reg(s, reg, 1, (uint8_t *)&int1_config2);

	return status;
}

int inv_imu_get_int_status(inv_imu_device_t *s, const inv_imu_int_num_t num,
                           inv_imu_int_state_t *it)
{
	int            status = INV_IMU_OK;
	intx_statusx_t int_status;
	uint32_t       reg;

	switch (num) {
	case INV_IMU_INT1:
		reg = INT1_STATUS0;
		break;
	case INV_IMU_INT2:
		reg = INT2_STATUS0;
		break;
	default:
		return INV_IMU_ERROR_BAD_ARG;
	}

	/* Read APEX interrupt status */
	status |= inv_imu_read_reg(s, reg, 2, (uint8_t *)&int_status);

	it->INV_FIFO_FULL    = int_status.int1_status0.int1_status_fifo_full;
	it->INV_FIFO_THS     = int_status.int1_status0.int1_status_fifo_ths;
	it->INV_UI_DRDY      = int_status.int1_status0.int1_status_drdy;
	it->INV_OIS1         = int_status.int1_status0.int1_status_aux1_drdy;
	it->INV_UI_FSYNC     = int_status.int1_status0.int1_status_ap_fsync;
	it->INV_AGC_RDY      = int_status.int1_status0.int1_status_ap_agc_rdy;
	it->INV_OIS1_AGC_RDY = int_status.int1_status0.int1_status_aux1_agc_rdy;
	it->INV_RESET_DONE   = int_status.int1_status0.int1_status_reset_done;

	it->INV_PLL_RDY      = int_status.int1_status1.int1_status_pll_rdy;
	it->INV_WOM_X        = int_status.int1_status1.int1_status_wom_x;
	it->INV_WOM_Y        = int_status.int1_status1.int1_status_wom_y;
	it->INV_WOM_Z        = int_status.int1_status1.int1_status_wom_z;
	it->INV_I3C_PROT_ERR = int_status.int1_status1.int1_status_i3c_protocol_err;
	it->INV_I2CM_DONE    = int_status.int1_status1.int1_status_i2cm_done;
	it->INV_EDMP_EVENT   = int_status.int1_status1.int1_status_apex_event;

	return status;
}

int inv_imu_get_endianness(inv_imu_device_t *s)
{
	int         status = INV_IMU_OK;
	sreg_ctrl_t sreg_ctrl;

	status |= inv_imu_read_reg(s, SREG_CTRL, 1, (uint8_t *)&sreg_ctrl);
	if (!status)
		s->endianness_data = sreg_ctrl.sreg_data_endian_sel;

	return status;
}

int inv_imu_select_accel_lp_clk(inv_imu_device_t *s, smc_control_0_accel_lp_clk_sel_t clk_sel)
{
	int             status = INV_IMU_OK;
	smc_control_0_t smc_control_0;

	status |= inv_imu_read_reg(s, SMC_CONTROL_0, 1, (uint8_t *)&smc_control_0);
	smc_control_0.accel_lp_clk_sel = clk_sel;
	status |= inv_imu_write_reg(s, SMC_CONTROL_0, 1, (uint8_t *)&smc_control_0);

	return status;
}

const char *inv_imu_get_version(void)
{
	return INV_IMU_VERSION_STRING;
}
