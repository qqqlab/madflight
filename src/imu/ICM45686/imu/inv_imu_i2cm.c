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

#include "./inv_imu_i2cm.h"

int inv_imu_init_i2cm(inv_imu_device_t *s)
{
	int                         status = INV_IMU_OK;
	ioc_pad_scenario_aux_ovrd_t ioc_pad_scenario_aux_ovrd;

	/* Force AUX1 in I2CM master mode */
	status |=
	    inv_imu_read_reg(s, IOC_PAD_SCENARIO_AUX_OVRD, 1, (uint8_t *)&ioc_pad_scenario_aux_ovrd);
	ioc_pad_scenario_aux_ovrd.aux1_mode_ovrd     = 1;
	ioc_pad_scenario_aux_ovrd.aux1_mode_ovrd_val = 1;
	status |=
	    inv_imu_write_reg(s, IOC_PAD_SCENARIO_AUX_OVRD, 1, (uint8_t *)&ioc_pad_scenario_aux_ovrd);

	return status;
}

int inv_imu_uninit_i2cm(inv_imu_device_t *s)
{
	int                         status = INV_IMU_OK;
	ioc_pad_scenario_aux_ovrd_t ioc_pad_scenario_aux_ovrd;

	/* Restore AUX1 to trimmed factory value : SPI slave, I2CM master or I2CM bypass */
	status |=
	    inv_imu_read_reg(s, IOC_PAD_SCENARIO_AUX_OVRD, 1, (uint8_t *)&ioc_pad_scenario_aux_ovrd);
	ioc_pad_scenario_aux_ovrd.aux1_mode_ovrd = 0;
	status |=
	    inv_imu_write_reg(s, IOC_PAD_SCENARIO_AUX_OVRD, 1, (uint8_t *)&ioc_pad_scenario_aux_ovrd);

	return status;
}

static int configure_i2cm(inv_imu_device_t *s, uint8_t slave_id, uint8_t is_last_sensor_id,
                          uint8_t *wdata_idx, uint8_t *i2cm_command_idx,
                          const inv_imu_i2c_master_cfg_t *cfg_es)
{
	int status = INV_IMU_OK;

	/* Setup I2C slave address targeted */
	status |= inv_imu_write_reg(s, I2CM_DEV_PROFILE1 + 2 * slave_id, 1, &cfg_es->i2c_addr);

	for (uint8_t op_id = 0; op_id < cfg_es->op_cnt; op_id++) {
		i2cm_command_0_t i2cm_command_0 = {
			.endflag_0 = 0x00,
			.ch_sel_0  = slave_id,
			/* Set Command i2cm_command_idx to read/write using channel slave_id */
			.r_w_0      = cfg_es->op[op_id].r_n_w,
			.burstlen_0 = cfg_es->op[op_id].len
		};

		if (cfg_es->op[op_id].r_n_w) {
			/*
			 * Setup register address to read from, there can only be 1 read sequence
			 * per slave config requested 
			 */
			status |= inv_imu_write_reg(s, I2CM_DEV_PROFILE0 + 2 * slave_id, 1,
			                            &cfg_es->op[op_id].reg_addr);
		} else {
			if (cfg_es->op[op_id].len) {
				/* I2CM can write at most 6 bytes = register address + 5 payload bytes */
				if ((*wdata_idx + cfg_es->op[op_id].len) > 5)
					return INV_IMU_ERROR;
				/* Load register address to write to */
				status |= inv_imu_write_reg(s, I2CM_WR_DATA0 + *wdata_idx, 1,
				                            &cfg_es->op[op_id].reg_addr);
				(*wdata_idx)++;
				/* Load value to write */
				status |= inv_imu_write_reg(s, I2CM_WR_DATA0 + *wdata_idx, cfg_es->op[op_id].len,
				                            cfg_es->op[op_id].wdata);
				*wdata_idx += cfg_es->op[op_id].len;

				/* In case of write, burst size is augmented by 1 byte due to register address */
				i2cm_command_0.burstlen_0++;
			}
		}

		/* Set end flag when this is last i2c operation to be performed */
		if (is_last_sensor_id && (cfg_es->op_cnt == (op_id + 1)))
			i2cm_command_0.endflag_0 = 0x01;
		status |=
		    inv_imu_write_reg(s, I2CM_COMMAND_0 + *i2cm_command_idx, 1, (uint8_t *)&i2cm_command_0);
		(*i2cm_command_idx)++;
	}

	return status;
}

int inv_imu_configure_i2cm(inv_imu_device_t *s, const inv_imu_i2c_master_cfg_t *cfg_es0,
                           const inv_imu_i2c_master_cfg_t *cfg_es1)
{
	int     status           = INV_IMU_OK;
	uint8_t wdata_idx        = 0;
	uint8_t i2cm_command_idx = 0;

	/* There can't be more than 4 operations requested */
	if (cfg_es0 && cfg_es1) {
		if ((cfg_es0->op_cnt + cfg_es1->op_cnt) > 4)
			return INV_IMU_ERROR_BAD_ARG;
	} else if (cfg_es0) {
		if (cfg_es0->op_cnt > 4)
			return INV_IMU_ERROR_BAD_ARG;
	} else if (cfg_es1) {
		if (cfg_es1->op_cnt > 4)
			return INV_IMU_ERROR_BAD_ARG;
	} else {
		return INV_IMU_ERROR_BAD_ARG;
	}

	if (cfg_es0)
		status |= configure_i2cm(s, 0, cfg_es1 ? 0 : 1 /* is_last_sensor_id */, &wdata_idx,
		                         &i2cm_command_idx, cfg_es0);

	if (cfg_es1)
		status |=
		    configure_i2cm(s, 1, 1 /* is_last_sensor_id */, &wdata_idx, &i2cm_command_idx, cfg_es1);

	return status;
}

int inv_imu_i2cm_clock_force(inv_imu_device_t *s, uint8_t on_off)
{
	int         status = INV_IMU_OK;
	reg_misc1_t reg_misc1;

	/* I2C master requires either RCOSC or PLL clock to be ON */
	status |= inv_imu_read_reg(s, REG_MISC1, 1, (uint8_t *)&reg_misc1);
	if (on_off)
		/* Force RCOSC to be turned ON */
		reg_misc1.osc_id_ovrd = REG_MISC1_OSC_ID_OVRD_RCOSC;
	else
		/* Do not force RCOSC anymore, clock will be OFF if no sensor is enabled */
		reg_misc1.osc_id_ovrd = REG_MISC1_OSC_ID_OVRD_OFF;
	status |= inv_imu_write_reg(s, REG_MISC1, 1, (uint8_t *)&reg_misc1);

	return status;
}

int inv_imu_start_i2cm_ops(inv_imu_device_t *s, uint8_t fast_mode)
{
	int            status = INV_IMU_OK;
	i2cm_control_t i2cm_control;

	/* Start the I2CM command */
	status |= inv_imu_read_reg(s, I2CM_CONTROL, 1, (uint8_t *)&i2cm_control);
	i2cm_control.i2cm_go    = 1;
	i2cm_control.i2cm_speed = !fast_mode;
	status |= inv_imu_write_reg(s, I2CM_CONTROL, 1, (uint8_t *)&i2cm_control);

	return status;
}

int inv_imu_get_i2cm_data(inv_imu_device_t *s, uint8_t *rdata, uint8_t len)
{
	int status = INV_IMU_OK;

	if (len && rdata)
		status |= inv_imu_read_reg(s, I2CM_RD_DATA0, len, rdata);
	else
		return INV_IMU_ERROR_BAD_ARG;

	return status;
}
