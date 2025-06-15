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

#include "./inv_imu_driver_advanced.h"

/* Static functions */
static int configure_serial_interface(inv_imu_device_t *s);
static int init_fifo_compression(inv_imu_device_t *s);
#if INV_IMU_FSYNC_SUPPORTED
static int init_fsync_tag(inv_imu_device_t *s);
#endif

/* Fail build if `condition` is true */
#define BUILD_BUG_ON(condition) ((void)sizeof(char[1 - 2 * !!(condition)]))

int inv_imu_adv_init(inv_imu_device_t *s)
{
	int     status = INV_IMU_OK;
	uint8_t whoami;

	/* Ensure `inv_imu_adv_var_t` fits within `adv_var` */
	BUILD_BUG_ON(sizeof(s->adv_var) < sizeof(inv_imu_adv_var_t));

	/* Verify required callback are assigned */
	if (s->transport.read_reg == NULL || s->transport.write_reg == NULL ||
	    s->transport.sleep_us == NULL)
		return INV_IMU_ERROR;

	/* Wait 3 ms to ensure device is properly supplied  */
	inv_imu_sleep_us(s, 3000);

	/* Configure IMU depending on the serial interface */
	status |= configure_serial_interface(s);
	if (status)
		return status;

	/* Read and check whoami */
	status |= inv_imu_get_who_am_i(s, &whoami);
	if (whoami != INV_IMU_WHOAMI)
		return INV_IMU_ERROR;

	/* Reset device */
	status |= inv_imu_adv_device_reset(s);

	return status;
}

int inv_imu_adv_device_reset(inv_imu_device_t *s)
{
	int                status = INV_IMU_OK;
	inv_imu_adv_var_t *e      = (inv_imu_adv_var_t *)s->adv_var;

	status |= inv_imu_soft_reset(s);

	/* Read endianness for further processing */
	status |= inv_imu_get_endianness(s); /* Set `endianness_data` variable */

	/* Set default FIFO configuration */
	e->fifo_is_used = INV_IMU_DISABLE; /* FIFO disabled by default */
	e->fifo_comp_en = INV_IMU_DISABLE; /* FIFO compression disabled by default */
	e->fifo_mode    = FIFO_CONFIG0_FIFO_MODE_BYPASS; /* FIFO in BYPASS by default */

	/* From driver layer */
	s->fifo_frame_size = 0; /* Init at 0 by default */

	/* Initialize FIFO compression variables */
	status |= init_fifo_compression(s);

#if INV_IMU_FSYNC_SUPPORTED
	/* Initialize FSYNC tag variables */
	status |= init_fsync_tag(s);
#endif

	return status;
}

int inv_imu_adv_enable_accel_lp(inv_imu_device_t *s)
{
	return inv_imu_set_accel_mode(s, PWR_MGMT0_ACCEL_MODE_LP);
}

int inv_imu_adv_enable_accel_ln(inv_imu_device_t *s)
{
	return inv_imu_set_accel_mode(s, PWR_MGMT0_ACCEL_MODE_LN);
}

int inv_imu_adv_disable_accel(inv_imu_device_t *s)
{
	int                      status = INV_IMU_OK;
	const inv_imu_adv_var_t *e      = (const inv_imu_adv_var_t *)s->adv_var;
	pwr_mgmt0_t              pwr_mgmt0;
	int                      clear_fifo = 0;

	status |= inv_imu_read_reg(s, PWR_MGMT0, 1, (uint8_t *)&pwr_mgmt0);

	/* 
	 * Check if accel is the last one enabled.
	 * If it is, set `clear_fifo` flag to reset fifo after accel is actually turned off.
	 */
	if ((pwr_mgmt0.gyro_mode == PWR_MGMT0_GYRO_MODE_OFF) && e->fifo_is_used) {
		clear_fifo = 1;
	}

	/* Turn off accel */
	status |= inv_imu_set_accel_mode(s, PWR_MGMT0_ACCEL_MODE_OFF);

	if (clear_fifo) {
		/* Reset FIFO to guarantee no data is left after turning off all sensors. */
		status |= inv_imu_adv_reset_fifo(s);
	}

	return status;
}

int inv_imu_adv_enable_gyro_ln(inv_imu_device_t *s)
{
	return inv_imu_set_gyro_mode(s, PWR_MGMT0_GYRO_MODE_LN);
}

int inv_imu_adv_enable_gyro_lp(inv_imu_device_t *s)
{
	return inv_imu_set_gyro_mode(s, PWR_MGMT0_GYRO_MODE_LP);
}

int inv_imu_adv_disable_gyro(inv_imu_device_t *s)
{
	int                      status = INV_IMU_OK;
	const inv_imu_adv_var_t *e      = (const inv_imu_adv_var_t *)s->adv_var;
	pwr_mgmt0_t              pwr_mgmt0;
	int                      clear_fifo = 0;

	status |= inv_imu_read_reg(s, PWR_MGMT0, 1, (uint8_t *)&pwr_mgmt0);

	/* 
	 * Check if gyro is the last one enabled.
	 * If it is, set `clear_fifo` flag to reset fifo after gyro is actually turned off.
	 */
	if ((pwr_mgmt0.accel_mode == PWR_MGMT0_ACCEL_MODE_OFF) && e->fifo_is_used) {
		clear_fifo = 1;
	}

	/* Turn off gyro */
	status |= inv_imu_set_gyro_mode(s, PWR_MGMT0_GYRO_MODE_OFF);

	if (clear_fifo) {
		/* Reset FIFO to guarantee no data is left after turning off all sensors. */
		status |= inv_imu_adv_reset_fifo(s);
	}

	return status;
}

#if INV_IMU_INT2_PIN_SUPPORTED
int inv_imu_adv_set_int2_pin_usage(inv_imu_device_t *                             s,
                                   ioc_pad_scenario_ovrd_pads_int2_cfg_ovrd_val_t usage)
{
	int                     status = INV_IMU_OK;
	ioc_pad_scenario_ovrd_t ioc_pad_scenario_ovrd;
#if INV_IMU_AUX2_SUPPORTED
	ioc_pad_scenario_aux_ovrd_t ioc_pad_scenario_aux_ovrd;

	/* pads_int2_cfg_ovrd_val is effective only when aux2_enable is 0. */
	status |=
	    inv_imu_read_reg(s, IOC_PAD_SCENARIO_AUX_OVRD, 1, (uint8_t *)&ioc_pad_scenario_aux_ovrd);
	ioc_pad_scenario_aux_ovrd.aux2_enable_ovrd     = 1;
	ioc_pad_scenario_aux_ovrd.aux2_enable_ovrd_val = 0;
	status |=
	    inv_imu_write_reg(s, IOC_PAD_SCENARIO_AUX_OVRD, 1, (uint8_t *)&ioc_pad_scenario_aux_ovrd);
#endif

	status |= inv_imu_read_reg(s, IOC_PAD_SCENARIO_OVRD, 1, (uint8_t *)&ioc_pad_scenario_ovrd);
	ioc_pad_scenario_ovrd.pads_int2_cfg_ovrd     = 1;
	ioc_pad_scenario_ovrd.pads_int2_cfg_ovrd_val = (uint8_t)usage;
	status |= inv_imu_write_reg(s, IOC_PAD_SCENARIO_OVRD, 1, (uint8_t *)&ioc_pad_scenario_ovrd);

	return status;
}
#endif /* INV_IMU_INT2_PIN_SUPPORTED */

#if INV_IMU_FSYNC_SUPPORTED
int inv_imu_adv_configure_fsync_ap_tag(inv_imu_device_t *s, fsync_config0_ap_fsync_sel_t sensor_tag)
{
	int                status = INV_IMU_OK;
	inv_imu_adv_var_t *e      = (inv_imu_adv_var_t *)s->adv_var;
	fsync_config0_t    fsync_config0;

	e->fsync_tag = sensor_tag;

	status |= inv_imu_read_reg(s, FSYNC_CONFIG0, 1, (uint8_t *)&fsync_config0);
	fsync_config0.ap_fsync_sel = (uint8_t)sensor_tag;
	status |= inv_imu_write_reg(s, FSYNC_CONFIG0, 1, (uint8_t *)&fsync_config0);

	return status;
}

int inv_imu_adv_enable_fsync(inv_imu_device_t *s)
{
	int               status = INV_IMU_OK;
	tmst_wom_config_t tmst_wom_config;
	smc_control_0_t   smc_control_0;

	status |= inv_imu_read_reg(s, TMST_WOM_CONFIG, 1, (uint8_t *)&tmst_wom_config);
	tmst_wom_config.tmst_delta_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, TMST_WOM_CONFIG, 1, (uint8_t *)&tmst_wom_config);

	status |= inv_imu_read_reg(s, SMC_CONTROL_0, 1, (uint8_t *)&smc_control_0);
	smc_control_0.tmst_fsync_en = INV_IMU_ENABLE;
	smc_control_0.tmst_en       = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, SMC_CONTROL_0, 1, (uint8_t *)&smc_control_0);

	return status;
}

int inv_imu_adv_disable_fsync(inv_imu_device_t *s)
{
	int               status = INV_IMU_OK;
	smc_control_0_t   smc_control_0;
	tmst_wom_config_t tmst_wom_config;

	status |= inv_imu_read_reg(s, SMC_CONTROL_0, 1, (uint8_t *)&smc_control_0);
	smc_control_0.tmst_fsync_en = INV_IMU_DISABLE;
	smc_control_0.tmst_en       = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, SMC_CONTROL_0, 1, (uint8_t *)&smc_control_0);

	status |= inv_imu_read_reg(s, TMST_WOM_CONFIG, 1, (uint8_t *)&tmst_wom_config);
	tmst_wom_config.tmst_delta_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, TMST_WOM_CONFIG, 1, (uint8_t *)&tmst_wom_config);

	return status;
}
#endif

int inv_imu_adv_configure_wom(inv_imu_device_t *s, const uint8_t wom_x_th, const uint8_t wom_y_th,
                              const uint8_t wom_z_th, tmst_wom_config_wom_int_mode_t wom_int,
                              tmst_wom_config_wom_int_dur_t wom_dur)
{
	int               status = INV_IMU_OK;
	uint8_t           data[3];
	tmst_wom_config_t tmst_wom_config;

	data[0] = wom_x_th; // Set X threshold
	data[1] = wom_y_th; // Set Y threshold
	data[2] = wom_z_th; // Set Z threshold
	status |= inv_imu_write_reg(s, ACCEL_WOM_X_THR, sizeof(data), &data[0]);

	/*
	 * Compare current sample with the previous sample and WOM from the 3 axis
	 * are ORed or ANDed to produce WOM signal.
	 */
	status |= inv_imu_read_reg(s, TMST_WOM_CONFIG, 1, (uint8_t *)&tmst_wom_config);
	tmst_wom_config.wom_mode     = TMST_WOM_CONFIG_WOM_MODE_CMP_PREV;
	tmst_wom_config.wom_int_mode = (uint8_t)wom_int;
	tmst_wom_config.wom_int_dur  = (uint8_t)wom_dur;
	status |= inv_imu_write_reg(s, TMST_WOM_CONFIG, 1, (uint8_t *)&tmst_wom_config);

	return status;
}

int inv_imu_adv_enable_wom(inv_imu_device_t *s)
{
	int               status = INV_IMU_OK;
	tmst_wom_config_t tmst_wom_config;

	/* Enable WOM */
	status |= inv_imu_read_reg(s, TMST_WOM_CONFIG, 1, (uint8_t *)&tmst_wom_config);
	tmst_wom_config.wom_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, TMST_WOM_CONFIG, 1, (uint8_t *)&tmst_wom_config);

	return status;
}

int inv_imu_adv_disable_wom(inv_imu_device_t *s)
{
	int               status = INV_IMU_OK;
	tmst_wom_config_t tmst_wom_config;

	/* Disable WOM */
	status |= inv_imu_read_reg(s, TMST_WOM_CONFIG, 1, (uint8_t *)&tmst_wom_config);
	tmst_wom_config.wom_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, TMST_WOM_CONFIG, 1, (uint8_t *)&tmst_wom_config);

	return status;
}

int inv_imu_adv_get_data_from_registers(inv_imu_device_t *s)
{
	int                      status = INV_IMU_OK;
	const inv_imu_adv_var_t *e      = (const inv_imu_adv_var_t *)s->adv_var;
	inv_imu_sensor_event_t   event  = { 0 };
	uint8_t                  data[ACCEL_DATA_SIZE + GYRO_DATA_SIZE + TEMP_DATA_SIZE];
#if INV_IMU_FSYNC_SUPPORTED
	uint8_t fsync_tag_in_accel = 0;
	uint8_t fsync_tag_in_gyro  = 0;
	uint8_t fsync_tag_in_temp  = 0;
#endif

	/* Read sensor data from registers. */
	status |= inv_imu_read_reg(s, ACCEL_DATA_X1_UI, sizeof(data), data);

	/* Read accel data from sensor registers. */
	FORMAT_16_BITS_DATA(s->endianness_data, &data[0], (uint16_t *)&event.accel[0]);
	FORMAT_16_BITS_DATA(s->endianness_data, &data[2], (uint16_t *)&event.accel[1]);
	FORMAT_16_BITS_DATA(s->endianness_data, &data[4], (uint16_t *)&event.accel[2]);

	if ((event.accel[0] != INVALID_VALUE_FIFO) && (event.accel[1] != INVALID_VALUE_FIFO) &&
	    (event.accel[2] != INVALID_VALUE_FIFO)) {
		event.sensor_mask |= (1 << INV_SENSOR_ACCEL);
	}

	/* Gyro */
	FORMAT_16_BITS_DATA(s->endianness_data, &data[6], (uint16_t *)&event.gyro[0]);
	FORMAT_16_BITS_DATA(s->endianness_data, &data[8], (uint16_t *)&event.gyro[1]);
	FORMAT_16_BITS_DATA(s->endianness_data, &data[10], (uint16_t *)&event.gyro[2]);
	if ((event.gyro[0] != INVALID_VALUE_FIFO) && (event.gyro[1] != INVALID_VALUE_FIFO) &&
	    (event.gyro[2] != INVALID_VALUE_FIFO)) {
		event.sensor_mask |= (1 << INV_SENSOR_GYRO);
	}

	/* Temperature */
	FORMAT_16_BITS_DATA(s->endianness_data, &data[12], (uint16_t *)&event.temperature);
	if (event.temperature != INVALID_VALUE_FIFO)
		event.sensor_mask |= (1 << INV_SENSOR_TEMPERATURE);

#if INV_IMU_FSYNC_SUPPORTED
	/* 
	 * Check if fsync flag is set and then get FSYNC counter.
	 */
	switch (e->fsync_tag) {
	case FSYNC_CONFIG0_AP_FSYNC_ACCEL_X:
		fsync_tag_in_accel =
		    (event.sensor_mask & (1 << INV_SENSOR_ACCEL)) && (event.accel[0] & 0x1);
		break;
	case FSYNC_CONFIG0_AP_FSYNC_ACCEL_Y:
		fsync_tag_in_accel =
		    (event.sensor_mask & (1 << INV_SENSOR_ACCEL)) && (event.accel[1] & 0x1);
		break;
	case FSYNC_CONFIG0_AP_FSYNC_ACCEL_Z:
		fsync_tag_in_accel =
		    (event.sensor_mask & (1 << INV_SENSOR_ACCEL)) && (event.accel[2] & 0x1);
		break;
	case FSYNC_CONFIG0_AP_FSYNC_GYRO_X:
		fsync_tag_in_gyro = (event.sensor_mask & (1 << INV_SENSOR_GYRO)) && (event.gyro[0] & 0x1);
		break;
	case FSYNC_CONFIG0_AP_FSYNC_GYRO_Y:
		fsync_tag_in_gyro = (event.sensor_mask & (1 << INV_SENSOR_GYRO)) && (event.gyro[1] & 0x1);
		break;
	case FSYNC_CONFIG0_AP_FSYNC_GYRO_Z:
		fsync_tag_in_gyro = (event.sensor_mask & (1 << INV_SENSOR_GYRO)) && (event.gyro[2] & 0x1);
		break;
	case FSYNC_CONFIG0_AP_FSYNC_TEMP:
		fsync_tag_in_temp =
		    (event.sensor_mask & (1 << INV_SENSOR_TEMPERATURE)) && (event.temperature & 0x1);
		break;
	default:
		break;
	}

	/* Sensor data register is configured to expose fsync flag. If fsync flag is set, process fsync counter. */
	if (fsync_tag_in_accel || fsync_tag_in_gyro || fsync_tag_in_temp) {
		uint8_t fsync_count[2];

		/*
		 * Read 16bits fsync counter containing time elapsed between last FSYNC interrupt and last ODR event.
		 * Fsync delta time depends on data endianness as counter is read over 2 registers and timestamp resolution.
		 */
		status |= inv_imu_read_reg(s, TMST_FSYNCH, 2, fsync_count);
		FORMAT_16_BITS_DATA(s->endianness_data, &fsync_count[0],
		                    (uint16_t *)&event.timestamp_fsync);

		event.sensor_mask |= (1 << INV_SENSOR_FSYNC_EVENT);
	}
#endif

	/* call sensor event callback */
	if (e->sensor_event_cb)
		e->sensor_event_cb(&event);

	return status;
}

/** @brief Parse a FIFO frame and generate a sensor event.
 *         Applies when compression is disabled.
 *  @param[in] s      Pointer to device.
 *  @param[in] frame  Data to parse.
 */
static int parse_fifo_frame(inv_imu_device_t *s, uint8_t *frame)
{
	int                      status    = INV_IMU_OK;
	const inv_imu_adv_var_t *e         = (const inv_imu_adv_var_t *)s->adv_var;
	uint16_t                 frame_idx = 0;
	const fifo_header_t *    header;
	const fifo_header2_t *   header2;
	inv_imu_sensor_event_t   event;
	uint8_t                  fifo_32bytes = s->fifo_frame_size == 32 ? 1 : 0;

	event.sensor_mask = 0;
	header2           = 0;
	header            = (const fifo_header_t *)&(frame[frame_idx]);
	frame_idx += FIFO_HEADER_SIZE;

	/* Read header2 if present */
	if (header->bits.ext_header) {
		header2 = (const fifo_header2_t *)&frame[frame_idx];
		frame_idx += FIFO_HEADER_SIZE;
	}

	/* Read accel data */
	if (header->bits.accel_bit || fifo_32bytes) {
		FORMAT_16_BITS_DATA(s->endianness_data, &(frame[0 + frame_idx]),
		                    (uint16_t *)&event.accel[0]);
		FORMAT_16_BITS_DATA(s->endianness_data, &(frame[2 + frame_idx]),
		                    (uint16_t *)&event.accel[1]);
		FORMAT_16_BITS_DATA(s->endianness_data, &(frame[4 + frame_idx]),
		                    (uint16_t *)&event.accel[2]);
		frame_idx += ACCEL_DATA_SIZE;
	}

	/* Read gyro data */
	if (header->bits.gyro_bit || fifo_32bytes) {
		FORMAT_16_BITS_DATA(s->endianness_data, &(frame[0 + frame_idx]),
		                    (uint16_t *)&event.gyro[0]);
		FORMAT_16_BITS_DATA(s->endianness_data, &(frame[2 + frame_idx]),
		                    (uint16_t *)&event.gyro[1]);
		FORMAT_16_BITS_DATA(s->endianness_data, &(frame[4 + frame_idx]),
		                    (uint16_t *)&event.gyro[2]);
		frame_idx += GYRO_DATA_SIZE;
	}

	if (header2) {
		/* Read External Sensor 0 */
		if (header2->bits.es0_en || fifo_32bytes) {
			uint8_t es0_data_size =
			    header2->bits.es0_6b_9b ? FIFO_ES0_9B_DATA_SIZE : FIFO_ES0_6B_DATA_SIZE;
			for (int j = 0; j < es0_data_size; j++)
				event.es0[j] = frame[j + frame_idx];
			/* es1 are always 9 bytes after es0 */
			frame_idx += FIFO_ES0_9B_DATA_SIZE;

			if (header2->bits.es0_vld)
				event.sensor_mask |= (1 << INV_SENSOR_ES0);
		}
		/* Read External Sensor 1 */
		if (header2->bits.es1_en || fifo_32bytes) {
			for (int j = 0; j < FIFO_ES1_DATA_SIZE; j++)
				event.es1[j] = frame[j + frame_idx];
			frame_idx += FIFO_ES1_DATA_SIZE;

			if (header2->bits.es1_vld)
				event.sensor_mask |= (1 << INV_SENSOR_ES1);
		}
	}

	if ((header->bits.accel_bit) || (header->bits.gyro_bit) || fifo_32bytes) {
		if (header->bits.twentybits_bit && !fifo_32bytes) {
			FORMAT_16_BITS_DATA(s->endianness_data, &(frame[0 + frame_idx]),
			                    (uint16_t *)&event.temperature);
			frame_idx += FIFO_TEMP_DATA_SIZE + FIFO_TEMP_HIGH_RES_SIZE;

			if (event.temperature != INVALID_VALUE_FIFO)
				event.sensor_mask |= (1 << INV_SENSOR_TEMPERATURE);
		} else {
			event.temperature = (int8_t)frame[0 + frame_idx];
			frame_idx += FIFO_TEMP_DATA_SIZE;

			if (event.temperature != INVALID_VALUE_FIFO_1B)
				event.sensor_mask |= (1 << INV_SENSOR_TEMPERATURE);
		}
	}

	if ((header->bits.timestamp_bit) || (header->bits.fsync_bit) || fifo_32bytes) {
		FORMAT_16_BITS_DATA(s->endianness_data, &(frame[0 + frame_idx]), &event.timestamp_fsync);
		frame_idx += FIFO_TS_FSYNC_SIZE;

		if (header->bits.fsync_bit)
			event.sensor_mask |= (1 << INV_SENSOR_FSYNC_EVENT);
	}

	if (header->bits.accel_bit) {
		if ((event.accel[0] != INVALID_VALUE_FIFO) && (event.accel[1] != INVALID_VALUE_FIFO) &&
		    (event.accel[2] != INVALID_VALUE_FIFO)) {
			event.sensor_mask |= (1 << INV_SENSOR_ACCEL);

			if (header->bits.twentybits_bit && !fifo_32bytes) {
				event.accel_high_res[0] = (frame[0 + frame_idx] >> 4) & 0xF;
				event.accel_high_res[1] = (frame[1 + frame_idx] >> 4) & 0xF;
				event.accel_high_res[2] = (frame[2 + frame_idx] >> 4) & 0xF;
			}
		}
	}

	if (header->bits.gyro_bit) {
		if ((event.gyro[0] != INVALID_VALUE_FIFO) && (event.gyro[1] != INVALID_VALUE_FIFO) &&
		    (event.gyro[2] != INVALID_VALUE_FIFO)) {
			event.sensor_mask |= (1 << INV_SENSOR_GYRO);

			if (header->bits.twentybits_bit && !fifo_32bytes) {
				event.gyro_high_res[0] = (frame[0 + frame_idx]) & 0xF;
				event.gyro_high_res[1] = (frame[1 + frame_idx]) & 0xF;
				event.gyro_high_res[2] = (frame[2 + frame_idx]) & 0xF;
			}
		}
	}

	/* call sensor event callback */
	if (e->sensor_event_cb)
		e->sensor_event_cb(&event);

	return status;
}

/** @brief Decode one event from a compressed frame and generate a sensor event .
 *  @param[in] s          Pointer to device.
 *  @param[in] frame      Data to parse.
 *  @param[in] event_num  Event number to decode.
 */
static int decode_compressed_event(inv_imu_device_t *s, uint8_t *frame, uint8_t event_num)
{
	int                       status        = INV_IMU_OK;
	inv_imu_adv_var_t *       e             = (inv_imu_adv_var_t *)s->adv_var;
	const fifo_comp_header_t *comp_header   = (const fifo_comp_header_t *)&frame[0];
	const fifo_comp_decode_t *decode_header = (const fifo_comp_decode_t *)&frame[1];
	uint8_t                   validity_mask;
	int8_t                    diff_s0[3];
	int8_t                    diff_s1[3];
	int8_t                    diff_s2;
	inv_imu_sensor_event_t    event;

	event.sensor_mask = 0;

	switch (event_num) {
	case 1:
		validity_mask = 0x1;
		switch (comp_header->bits.comp_ratio) {
		case FIFO_COMP_X2_COMPRESSION:
			diff_s0[0] = (int8_t)(frame[2]);
			diff_s0[1] = (int8_t)(frame[3]);
			diff_s0[2] = (int8_t)(frame[4]);
			diff_s1[0] = (int8_t)(frame[8]);
			diff_s1[1] = (int8_t)(frame[9]);
			diff_s1[2] = (int8_t)(frame[10]);
			diff_s2    = (int8_t)(frame[14]);
			break;
		case FIFO_COMP_X3_COMPRESSION:
			diff_s0[0] = INT5_TO_INT8(frame[2] & 0x1F);
			diff_s0[1] = INT5_TO_INT8(((frame[3] & 0x03) << 3) | ((frame[2] & 0xE0) >> 5));
			diff_s0[2] = INT5_TO_INT8((frame[3] & 0x7C) >> 2);
			diff_s1[0] = INT5_TO_INT8(frame[8] & 0x1F);
			diff_s1[1] = INT5_TO_INT8(((frame[9] & 0x03) << 3) | ((frame[8] & 0xE0) >> 5));
			diff_s1[2] = INT5_TO_INT8((frame[9] & 0x7C) >> 2);
			diff_s2    = INT5_TO_INT8(frame[14] & 0x1F);
			break;
		case FIFO_COMP_X4_COMPRESSION:
			diff_s0[0] = INT4_TO_INT8(frame[2] & 0x0F);
			diff_s0[1] = INT4_TO_INT8((frame[2] & 0xF0) >> 4);
			diff_s0[2] = INT4_TO_INT8(frame[3] & 0x0F);
			diff_s1[0] = INT4_TO_INT8(frame[8] & 0x0F);
			diff_s1[1] = INT4_TO_INT8((frame[8] & 0xF0) >> 4);
			diff_s1[2] = INT4_TO_INT8(frame[9] & 0x0F);
			diff_s2    = INT4_TO_INT8(frame[14] & 0x0F);
			break;
		default:
			return INV_IMU_ERROR_BAD_ARG;
		}
		break;
	case 2:
		validity_mask = 0x2;
		switch (comp_header->bits.comp_ratio) {
		case FIFO_COMP_X2_COMPRESSION:
			diff_s0[0] = (int8_t)(frame[5]);
			diff_s0[1] = (int8_t)(frame[6]);
			diff_s0[2] = (int8_t)(frame[7]);
			diff_s1[0] = (int8_t)(frame[11]);
			diff_s1[1] = (int8_t)(frame[12]);
			diff_s1[2] = (int8_t)(frame[13]);
			diff_s2    = (int8_t)(frame[15]);
			break;
		case FIFO_COMP_X3_COMPRESSION:
			diff_s0[0] = INT5_TO_INT8(frame[4] & 0x1F);
			diff_s0[1] = INT5_TO_INT8(((frame[5] & 0x03) << 3) | ((frame[4] & 0xE0) >> 5));
			diff_s0[2] = INT5_TO_INT8((frame[5] & 0x7C) >> 2);
			diff_s1[0] = INT5_TO_INT8(frame[10] & 0x1F);
			diff_s1[1] = INT5_TO_INT8(((frame[11] & 0x03) << 3) | ((frame[10] & 0xE0) >> 5));
			diff_s1[2] = INT5_TO_INT8((frame[11] & 0x7C) >> 2);
			diff_s2    = INT5_TO_INT8(((frame[15] & 0x03) << 3) | ((frame[14] & 0xE0) >> 5));
			break;
		case FIFO_COMP_X4_COMPRESSION:
			diff_s0[0] = INT4_TO_INT8((frame[3] & 0xF0) >> 4);
			diff_s0[1] = INT4_TO_INT8(frame[4] & 0x0F);
			diff_s0[2] = INT4_TO_INT8((frame[4] & 0xF0) >> 4);
			diff_s1[0] = INT4_TO_INT8((frame[9] & 0xF0) >> 4);
			diff_s1[1] = INT4_TO_INT8(frame[10] & 0x0F);
			diff_s1[2] = INT4_TO_INT8((frame[10] & 0xF0) >> 4);
			diff_s2    = INT4_TO_INT8((frame[14] & 0xF0) >> 4);
			break;
		default:
			return INV_IMU_ERROR_BAD_ARG;
		}
		break;
	case 3:
		validity_mask = 0x4;
		switch (comp_header->bits.comp_ratio) {
		case FIFO_COMP_X2_COMPRESSION:
			return INV_IMU_ERROR_BAD_ARG;
		case FIFO_COMP_X3_COMPRESSION:
			diff_s0[0] = INT5_TO_INT8(frame[6] & 0x1F);
			diff_s0[1] = INT5_TO_INT8(((frame[7] & 0x03) << 3) | ((frame[6] & 0xE0) >> 5));
			diff_s0[2] = INT5_TO_INT8((frame[7] & 0x7C) >> 2);
			diff_s1[0] = INT5_TO_INT8(frame[12] & 0x1F);
			diff_s1[1] = INT5_TO_INT8(((frame[13] & 0x03) << 3) | ((frame[12] & 0xE0) >> 5));
			diff_s1[2] = INT5_TO_INT8((frame[13] & 0x7C) >> 2);
			diff_s2    = INT5_TO_INT8((frame[15] & 0x7C) >> 2);
			break;
		case FIFO_COMP_X4_COMPRESSION:
			diff_s0[0] = INT4_TO_INT8(frame[5] & 0x0F);
			diff_s0[1] = INT4_TO_INT8((frame[5] & 0xF0) >> 4);
			diff_s0[2] = INT4_TO_INT8(frame[6] & 0x0F);
			diff_s1[0] = INT4_TO_INT8(frame[11] & 0x0F);
			diff_s1[1] = INT4_TO_INT8((frame[11] & 0xF0) >> 4);
			diff_s1[2] = INT4_TO_INT8(frame[12] & 0x0F);
			diff_s2    = INT4_TO_INT8(frame[15] & 0x0F);
			break;
		default:
			return INV_IMU_ERROR_BAD_ARG;
		}
		break;
	case 4:
		validity_mask = 0x8;
		switch (comp_header->bits.comp_ratio) {
		case FIFO_COMP_X2_COMPRESSION:
		case FIFO_COMP_X3_COMPRESSION:
			return INV_IMU_ERROR_BAD_ARG;
		case FIFO_COMP_X4_COMPRESSION:
			diff_s0[0] = INT4_TO_INT8((frame[6] & 0xF0) >> 4);
			diff_s0[1] = INT4_TO_INT8(frame[7] & 0x0F);
			diff_s0[2] = INT4_TO_INT8((frame[7] & 0xF0) >> 4);
			diff_s1[0] = INT4_TO_INT8((frame[12] & 0xF0) >> 4);
			diff_s1[1] = INT4_TO_INT8(frame[13] & 0x0F);
			diff_s1[2] = INT4_TO_INT8((frame[13] & 0xF0) >> 4);
			diff_s2    = INT4_TO_INT8((frame[15] & 0xF0) >> 4);
			break;
		default:
			return INV_IMU_ERROR_BAD_ARG;
		}
		break;
	default:
		return INV_IMU_ERROR_BAD_ARG;
	}

	/* Accel */
	if (comp_header->bits.accel_bit) /* accel_en */ {
		if ((decode_header->bits.valid_samples_a & validity_mask) && e->accel_baseline_found) {
			/* Reconstruct accel data */
			event.accel[0] = e->accel_baseline[0] + diff_s0[0];
			event.accel[1] = e->accel_baseline[1] + diff_s0[1];
			event.accel[2] = e->accel_baseline[2] + diff_s0[2];

			/* Set `sensor_mask` */
			event.sensor_mask |= (1 << INV_SENSOR_ACCEL);

			/* Update baseline for next event */
			e->accel_baseline[0] = event.accel[0];
			e->accel_baseline[1] = event.accel[1];
			e->accel_baseline[2] = event.accel[2];
		} else {
			/* Set invalid code for accel data */
			event.accel[0] = INVALID_VALUE_FIFO;
			event.accel[1] = INVALID_VALUE_FIFO;
			event.accel[2] = INVALID_VALUE_FIFO;
		}
	}

	/* Gyro */
	if (comp_header->bits.gyro_bit) /* gyro_en */ {
		if ((decode_header->bits.valid_samples_g & validity_mask) && e->gyro_baseline_found) {
			/* 
			 * Reconstruct gyro data 
			 * Use `s1` if accel is also enabled, otherwise, use `s0`
			 */
			event.gyro[0] =
			    e->gyro_baseline[0] + (comp_header->bits.accel_bit ? diff_s1[0] : diff_s0[0]);
			event.gyro[1] =
			    e->gyro_baseline[1] + (comp_header->bits.accel_bit ? diff_s1[1] : diff_s0[1]);
			event.gyro[2] =
			    e->gyro_baseline[2] + (comp_header->bits.accel_bit ? diff_s1[2] : diff_s0[2]);

			/* Set `sensor_mask` */
			event.sensor_mask |= (1 << INV_SENSOR_GYRO);

			/* Update baseline for next event */
			e->gyro_baseline[0] = event.gyro[0];
			e->gyro_baseline[1] = event.gyro[1];
			e->gyro_baseline[2] = event.gyro[2];
		} else {
			event.gyro[0] = INVALID_VALUE_FIFO;
			event.gyro[1] = INVALID_VALUE_FIFO;
			event.gyro[2] = INVALID_VALUE_FIFO;
		}
	}

	/* 
	 * Temperature 
	 * In compressed frames, temperature is only available if accel and gyro are enabled 
	 */
	event.temperature = INVALID_VALUE_FIFO_1B;
	if (comp_header->bits.accel_bit && comp_header->bits.gyro_bit) /* accel_en + gyro_en */ {
		if (((decode_header->bits.valid_samples_a & validity_mask) ||
		     (decode_header->bits.valid_samples_g & validity_mask)) &&
		    e->temp_baseline_found) {
			/* Reconstruct temp data */
			event.temperature = e->temp_baseline + diff_s2;

			/* Set `sensor_mask` */
			event.sensor_mask |= (1 << INV_SENSOR_TEMPERATURE);

			/* Update baseline for next event */
			e->temp_baseline = event.temperature;
		}
	}

	/* Notify event */
	if (e->sensor_event_cb)
		e->sensor_event_cb(&event);

	return status;
}

/** @brief Parse a compressed FIFO frame and generate a sensor event 
 *         for each event in the frame (up to 4).
 *  @param[in] s      Pointer to device.
 *  @param[in] frame  Data to parse.
 */
static int parse_compressed_fifo_frame(inv_imu_device_t *s, uint8_t *frame)
{
	int status = INV_IMU_OK;

	const fifo_comp_header_t *header    = (const fifo_comp_header_t *)&frame[0];
	uint8_t                   event_num = 1;

	for (uint8_t i = FIFO_COMP_1_SAMPLE_IN_FRAME; i <= header->bits.tot_sample; i++) {
		status |= decode_compressed_event(s, frame, event_num);
		event_num++;
	}

	return status;
}

/** @brief Parse an uncompressed FIFO frame and generate a sensor event.
 *         Applies for uncompressed frames generated when FIFO compression is enabled.
 *  @param[in] s      Pointer to device.
 *  @param[in] frame  Data to parse.
 */
static int parse_uncompressed_fifo_frame(inv_imu_device_t *s, uint8_t *frame)
{
	int                    status    = INV_IMU_OK;
	inv_imu_adv_var_t *    e         = (inv_imu_adv_var_t *)s->adv_var;
	uint16_t               frame_idx = 0;
	const fifo_header_t *  header;
	inv_imu_sensor_event_t event;

	event.sensor_mask = 0;
	header            = (const fifo_header_t *)&(frame[frame_idx]);
	frame_idx += FIFO_HEADER_SIZE;

	/* If `ext_header` is 1, this is not an uncompressed frame */
	if (header->bits.ext_header)
		return INV_IMU_ERROR;

	/* Init sensor mask and timestamp */
	event.sensor_mask     = 0;
	event.timestamp_fsync = 0;

	if (header->bits.accel_bit) {
		/* 
		 * Accel is available in frame 
		 * Do not use `FORMAT_16_BITS_DATA` as endianness is forced 
		 * to little endian when compression is enabled 
		 */
		event.accel[0] = frame[1 + frame_idx] << 8 | frame[0 + frame_idx];
		event.accel[1] = frame[3 + frame_idx] << 8 | frame[2 + frame_idx];
		event.accel[2] = frame[5 + frame_idx] << 8 | frame[4 + frame_idx];
		frame_idx += ACCEL_DATA_SIZE;

		if ((event.accel[0] != INVALID_VALUE_FIFO) && (event.accel[1] != INVALID_VALUE_FIFO) &&
		    (event.accel[2] != INVALID_VALUE_FIFO)) {
			/* Set baseline if event is valid */
			e->accel_baseline[0] = event.accel[0];
			e->accel_baseline[1] = event.accel[1];
			e->accel_baseline[2] = event.accel[2];

			e->accel_baseline_found = 1;

			event.sensor_mask |= (1 << INV_SENSOR_ACCEL);
		}
	}

	if (header->bits.gyro_bit) {
		/* 
		 * Gyro is available in frame 
		 * Do not use `FORMAT_16_BITS_DATA` as endianness is forced 
		 * to little endian when compression is enabled 
		 */
		event.gyro[0] = frame[1 + frame_idx] << 8 | frame[0 + frame_idx];
		event.gyro[1] = frame[3 + frame_idx] << 8 | frame[2 + frame_idx];
		event.gyro[2] = frame[5 + frame_idx] << 8 | frame[4 + frame_idx];
		frame_idx += GYRO_DATA_SIZE;

		if ((event.gyro[0] != INVALID_VALUE_FIFO) && (event.gyro[1] != INVALID_VALUE_FIFO) &&
		    (event.gyro[2] != INVALID_VALUE_FIFO)) {
			/* Set baseline if event is valid */
			e->gyro_baseline[0] = event.gyro[0];
			e->gyro_baseline[1] = event.gyro[1];
			e->gyro_baseline[2] = event.gyro[2];

			e->gyro_baseline_found = 1;

			event.sensor_mask |= (1 << INV_SENSOR_GYRO);
		}
	}

	if ((header->bits.accel_bit) || (header->bits.gyro_bit)) {
		/* Temperature is available in frame */
		event.temperature = (int8_t)frame[0 + frame_idx];
		frame_idx += FIFO_TEMP_DATA_SIZE;

		if (event.temperature != INVALID_VALUE_FIFO_1B) {
			e->temp_baseline       = event.temperature;
			e->temp_baseline_found = 1;
			event.sensor_mask |= (1 << INV_SENSOR_TEMPERATURE);
		}
	}

	if ((header->bits.accel_bit) && (header->bits.gyro_bit)) {
		/* Timestamp is available in frame */
		event.timestamp_fsync = frame[1 + frame_idx] << 8 | frame[0 + frame_idx];
		frame_idx += FIFO_TS_FSYNC_SIZE;
	}

	if (e->sensor_event_cb)
		e->sensor_event_cb(&event);

	return status;
}

int inv_imu_adv_get_data_from_fifo(inv_imu_device_t *s, uint8_t fifo_data[FIFO_MIRRORING_SIZE],
                                   uint16_t *fifo_count)
{
	int                      status = INV_IMU_OK;
	const inv_imu_adv_var_t *e      = (const inv_imu_adv_var_t *)s->adv_var;

	/* Read FIFO count */
	status |= inv_imu_get_frame_count(s, fifo_count);

	/*
	 * AN-000364: When operating in FIFO streaming mode, if FIFO threshold interrupt is triggered
	 * with M number of FIFO frames accumulated in the FIFO buffer, the host should only read the
	 * first M-1 number of FIFO frames
	 */
	if (e->fifo_mode == FIFO_CONFIG0_FIFO_MODE_STREAM)
		(*fifo_count)--;

	/* Read FIFO data */
	status |= inv_imu_read_reg(s, FIFO_DATA, *fifo_count * s->fifo_frame_size, fifo_data);
	return status;
}

int inv_imu_adv_parse_fifo_data(inv_imu_device_t *s, const uint8_t fifo_data[FIFO_MIRRORING_SIZE],
                                const uint16_t fifo_count)
{
	int                      status   = INV_IMU_OK;
	const inv_imu_adv_var_t *e        = (const inv_imu_adv_var_t *)s->adv_var;
	uint16_t                 fifo_idx = 0;

	/* Foreach packet in the FIFO */
	for (uint16_t i = 0; i < fifo_count; i++) {
		uint8_t frame[32] = { 0 };

		/* Create frame */
		for (int j = 0; j < s->fifo_frame_size; j++)
			frame[j] = fifo_data[fifo_idx + j];
		fifo_idx += s->fifo_frame_size;

		if (e->fifo_comp_en) {
			const fifo_header_t *header = (const fifo_header_t *)&(frame[0]);

			if (!header->bits.ext_header) /* Frame is not compressed */
				status |= parse_uncompressed_fifo_frame(s, frame);
			else /* Frame is compressed */
				status |= parse_compressed_fifo_frame(s, frame);
		} else {
			status |= parse_fifo_frame(s, frame);
		}
	}

	return status;
}

uint32_t inv_imu_adv_convert_odr_bitfield_to_us(uint32_t odr_bitfield)
{
	switch (odr_bitfield) {
	case ACCEL_CONFIG0_ACCEL_ODR_6400_HZ:
		return 156;
	case ACCEL_CONFIG0_ACCEL_ODR_3200_HZ:
		return 312;
	case ACCEL_CONFIG0_ACCEL_ODR_1600_HZ:
		return 625;
	case ACCEL_CONFIG0_ACCEL_ODR_800_HZ:
		return 1250;
	case ACCEL_CONFIG0_ACCEL_ODR_400_HZ:
		return 2500;
	case ACCEL_CONFIG0_ACCEL_ODR_200_HZ:
		return 5000;
	case ACCEL_CONFIG0_ACCEL_ODR_100_HZ:
		return 10000;
	case ACCEL_CONFIG0_ACCEL_ODR_50_HZ:
		return 20000;
	case ACCEL_CONFIG0_ACCEL_ODR_25_HZ:
		return 40000;
	case ACCEL_CONFIG0_ACCEL_ODR_12_5_HZ:
		return 80000;
	case ACCEL_CONFIG0_ACCEL_ODR_6_25_HZ:
		return 160000;
	case ACCEL_CONFIG0_ACCEL_ODR_3_125_HZ:
		return 320000;
	case ACCEL_CONFIG0_ACCEL_ODR_1_5625_HZ:
	default:
		return 640000;
	}
}

int inv_imu_adv_get_accel_fsr(inv_imu_device_t *s, accel_config0_accel_ui_fs_sel_t *accel_fsr)
{
	int                      status = INV_IMU_OK;
	const inv_imu_adv_var_t *e      = (const inv_imu_adv_var_t *)s->adv_var;
	fifo_config3_t           fifo_config3;

	status |= inv_imu_read_reg(s, FIFO_CONFIG3, 1, (uint8_t *)&fifo_config3);

	if (e->fifo_is_used == INV_IMU_ENABLE && (fifo_config3.fifo_hires_en == INV_IMU_ENABLE)) {
		/* FIFO is used and High Resolution mode is enabled */
#if INV_IMU_HIGH_FSR_SUPPORTED
		*accel_fsr = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_32_G;
#else
		*accel_fsr = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_16_G;
#endif
	} else {
		accel_config0_t accel_config0;
		status |= inv_imu_read_reg(s, ACCEL_CONFIG0, 1, (uint8_t *)&accel_config0);
		*accel_fsr = (accel_config0_accel_ui_fs_sel_t)accel_config0.accel_ui_fs_sel;
	}

	return status;
}

int inv_imu_adv_get_gyro_fsr(inv_imu_device_t *s, gyro_config0_gyro_ui_fs_sel_t *gyro_fsr)
{
	int                      status = INV_IMU_OK;
	const inv_imu_adv_var_t *e      = (const inv_imu_adv_var_t *)s->adv_var;
	fifo_config3_t           fifo_config3;

	status |= inv_imu_read_reg(s, FIFO_CONFIG3, 1, (uint8_t *)&fifo_config3);

	if (e->fifo_is_used == INV_IMU_ENABLE && (fifo_config3.fifo_hires_en == INV_IMU_ENABLE)) {
		/* FIFO is used and High Resolution mode is enabled */
#if INV_IMU_HIGH_FSR_SUPPORTED
		*gyro_fsr = GYRO_CONFIG0_GYRO_UI_FS_SEL_4000_DPS;
#else
		*gyro_fsr = GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS;
#endif
	} else {
		gyro_config0_t gyro_config0;
		status |= inv_imu_read_reg(s, GYRO_CONFIG0, 1, (uint8_t *)&gyro_config0);
		*gyro_fsr = (gyro_config0_gyro_ui_fs_sel_t)gyro_config0.gyro_ui_fs_sel;
	}

	return status;
}

int inv_imu_adv_set_timestamp_resolution(inv_imu_device_t *                 s,
                                         const tmst_wom_config_tmst_resol_t timestamp_resol)
{
	int               status = INV_IMU_OK;
	tmst_wom_config_t tmst_wom_config;

	status |= inv_imu_read_reg(s, TMST_WOM_CONFIG, 1, (uint8_t *)&tmst_wom_config);
	tmst_wom_config.tmst_resol = (uint8_t)timestamp_resol;
	status |= inv_imu_write_reg(s, TMST_WOM_CONFIG, 1, (uint8_t *)&tmst_wom_config);

	return status;
}

int inv_imu_adv_reset_fifo(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;

	status |= inv_imu_flush_fifo(s);
	status |= init_fifo_compression(s);

	return status;
}

int inv_imu_adv_get_fifo_config(inv_imu_device_t *s, inv_imu_adv_fifo_config_t *conf)
{
	int                   status = INV_IMU_OK;
	fifo_configx_t        cfg;
	odr_decimate_config_t odr_decimate_config;

	status |= inv_imu_read_reg(s, FIFO_CONFIG0, 6, (uint8_t *)&cfg);

	/* FIFO_CONFIG0 */
	conf->base_conf.fifo_mode = (fifo_config0_fifo_mode_t)cfg.fifo_config0.fifo_mode;
	if (cfg.fifo_config0.fifo_depth == FIFO_CONFIG0_FIFO_DEPTH_MAX ||
	    cfg.fifo_config0.fifo_depth == FIFO_CONFIG0_FIFO_DEPTH_APEX ||
	    cfg.fifo_config0.fifo_depth == FIFO_CONFIG0_FIFO_DEPTH_GAF) {
		/* `fifo_depth` is valid, return it */
		conf->base_conf.fifo_depth = (fifo_config0_fifo_depth_t)cfg.fifo_config0.fifo_depth;
	} else {
		/*
		 * `fifo_depth` is invalid so force it to FIFO_CONFIG0_FIFO_DEPTH_APEX.
		 * It will eventually be applied when calling `inv_imu_set_fifo_config`.
		 */
		conf->base_conf.fifo_depth = FIFO_CONFIG0_FIFO_DEPTH_APEX;
	}

	/* FIFO_CONFIG1_0 and FIFO_CONFIG1_1*/
	conf->base_conf.fifo_wm_th = (uint16_t)((uint16_t)cfg.fifo_config1_1 << 8) | cfg.fifo_config1_0;

	/* FIFO_CONFIG2 */
	conf->fifo_wr_wm_gt_th = (fifo_config2_fifo_wr_wm_gt_th_t)cfg.fifo_config2.fifo_wr_wm_gt_th;

	/* FIFO_CONFIG3 */
	conf->es1_en             = cfg.fifo_config3.fifo_es1_en;
	conf->es0_en             = cfg.fifo_config3.fifo_es0_en;
	conf->base_conf.hires_en = cfg.fifo_config3.fifo_hires_en;
	conf->base_conf.gyro_en  = cfg.fifo_config3.fifo_gyro_en;
	conf->base_conf.accel_en = cfg.fifo_config3.fifo_accel_en;

	/* FIFO_CONFIG4 */
	conf->comp_nc_flow_cfg =
	    (fifo_config4_fifo_comp_nc_flow_cfg_t)cfg.fifo_config4.fifo_comp_nc_flow_cfg;
	conf->comp_en       = cfg.fifo_config4.fifo_comp_en;
	conf->tmst_fsync_en = cfg.fifo_config4.fifo_tmst_fsync_en;
	conf->es0_6b_9b     = (fifo_config4_fifo_es0_6b_9b_t)cfg.fifo_config4.fifo_es0_6b_9b;

	status |= inv_imu_read_reg(s, ODR_DECIMATE_CONFIG, 1, (uint8_t *)&odr_decimate_config);

	/* ODR_DECIMATE_CONFIG */
	conf->gyro_dec = (odr_decimate_config_gyro_fifo_odr_dec_t)odr_decimate_config.gyro_fifo_odr_dec;
	conf->accel_dec =
	    (odr_decimate_config_accel_fifo_odr_dec_t)odr_decimate_config.accel_fifo_odr_dec;

	return status;
}

int inv_imu_adv_set_fifo_config(inv_imu_device_t *s, const inv_imu_adv_fifo_config_t *conf)
{
	int                   status = INV_IMU_OK;
	inv_imu_adv_var_t *   e      = (inv_imu_adv_var_t *)s->adv_var;
	fifo_configx_t        cfg;
	odr_decimate_config_t odr_decimate_config;
	smc_control_0_t       smc_control_0;
	uint8_t               fifo_frame_size = 0;

	/* External Sensors, FIFO compression and hires mode are all exclusive */
	uint8_t conf_cnt = (conf->es1_en || conf->es0_en) + conf->comp_en + conf->base_conf.hires_en;
	if (conf_cnt > 1)
		return INV_IMU_ERROR_BAD_ARG;

	/* `fifo_depth` must be a valid value. */
	if (conf->base_conf.fifo_depth != FIFO_CONFIG0_FIFO_DEPTH_MAX &&
	    conf->base_conf.fifo_depth != FIFO_CONFIG0_FIFO_DEPTH_APEX &&
	    conf->base_conf.fifo_depth != FIFO_CONFIG0_FIFO_DEPTH_GAF)
		return INV_IMU_ERROR_BAD_ARG;

	status |= inv_imu_read_reg(s, FIFO_CONFIG0, 6, (uint8_t *)&cfg);
	status |= inv_imu_read_reg(s, ODR_DECIMATE_CONFIG, 1, (uint8_t *)&odr_decimate_config);

	/* Disable fifo compression only if it was enabled */
	if (cfg.fifo_config4.fifo_comp_en == INV_IMU_ENABLE) {
		pwr_mgmt0_t pwr_mgmt0;
		uint32_t    accel_odr = UINT32_MAX;
		uint32_t    gyro_odr  = UINT32_MAX;

		/* Retreive fatest ODR */
		status |= inv_imu_read_reg(s, PWR_MGMT0, 1, (uint8_t *)&pwr_mgmt0);

		if (pwr_mgmt0.accel_mode != PWR_MGMT0_ACCEL_MODE_OFF) {
			accel_config0_t accel_config0;
			status |= inv_imu_read_reg(s, ACCEL_CONFIG0, 1, (uint8_t *)&accel_config0);
			accel_odr = inv_imu_adv_convert_odr_bitfield_to_us(accel_config0.accel_odr);
		}

		if (pwr_mgmt0.gyro_mode != PWR_MGMT0_GYRO_MODE_OFF) {
			gyro_config0_t gyro_config0;
			status |= inv_imu_read_reg(s, GYRO_CONFIG0, 1, (uint8_t *)&gyro_config0);
			gyro_odr = inv_imu_adv_convert_odr_bitfield_to_us(gyro_config0.gyro_odr);
		}

		cfg.fifo_config4.fifo_comp_en = INV_IMU_DISABLE;
		status |= inv_imu_write_reg(s, FIFO_CONFIG4, 1, (uint8_t *)&cfg.fifo_config4);

		/* Wait 2 ODR */
		if (accel_odr != UINT32_MAX || gyro_odr != UINT32_MAX)
			inv_imu_sleep_us(s, 2 * (accel_odr < gyro_odr ? accel_odr : gyro_odr));
	}

	/* Disable FIFO to safely apply configuration */
	cfg.fifo_config3.fifo_if_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, FIFO_CONFIG3, 1, (uint8_t *)&cfg.fifo_config3);
	cfg.fifo_config0.fifo_mode = (uint8_t)FIFO_CONFIG0_FIFO_MODE_BYPASS;
	status |= inv_imu_write_reg(s, FIFO_CONFIG0, 1, (uint8_t *)&cfg.fifo_config0);

	/* Set FIFO depth */
	cfg.fifo_config0.fifo_depth = (uint8_t)conf->base_conf.fifo_depth;

	/* Set WM */
	cfg.fifo_config1_0 = (uint8_t)conf->base_conf.fifo_wm_th;
	cfg.fifo_config1_1 = (uint8_t)(conf->base_conf.fifo_wm_th >> 8);

	/* Set fifo WM triggering condition */
	cfg.fifo_config2.fifo_wr_wm_gt_th = (uint8_t)conf->fifo_wr_wm_gt_th;

	/* Set which sensors go to FIFO */
	cfg.fifo_config3.fifo_es1_en   = conf->es1_en;
	cfg.fifo_config3.fifo_es0_en   = conf->es0_en;
	cfg.fifo_config3.fifo_hires_en = conf->base_conf.hires_en;
	cfg.fifo_config3.fifo_gyro_en  = conf->base_conf.gyro_en;
	cfg.fifo_config3.fifo_accel_en = conf->base_conf.accel_en;

	/* Set compression configuration, timestamp and buffer size for ES0 */
	cfg.fifo_config4.fifo_comp_nc_flow_cfg = (uint8_t)conf->comp_nc_flow_cfg;
	cfg.fifo_config4.fifo_tmst_fsync_en    = conf->tmst_fsync_en;
	cfg.fifo_config4.fifo_es0_6b_9b        = (uint8_t)conf->es0_6b_9b;

	/* Set FIFO decimation */
	odr_decimate_config.accel_fifo_odr_dec = (uint8_t)conf->accel_dec;
	odr_decimate_config.gyro_fifo_odr_dec  = (uint8_t)conf->gyro_dec;

	/* Apply configuration */
	status |= inv_imu_write_reg(s, FIFO_CONFIG0, 6, (uint8_t *)&cfg);
	status |= inv_imu_write_reg(s, ODR_DECIMATE_CONFIG, 1, (uint8_t *)&odr_decimate_config);

	/* Turn on Timestamp if needed */
	status |= inv_imu_read_reg(s, SMC_CONTROL_0, 1, (uint8_t *)&smc_control_0);
	if (conf->tmst_fsync_en) {
		smc_control_0.tmst_en       = INV_IMU_ENABLE;
		smc_control_0.tmst_fsync_en = INV_IMU_ENABLE;
	} else {
		smc_control_0.tmst_en       = INV_IMU_DISABLE;
		smc_control_0.tmst_fsync_en = INV_IMU_DISABLE;
	}
	status |= inv_imu_write_reg(s, SMC_CONTROL_0, 1, (uint8_t *)&smc_control_0);

	/* Set expected fifo_mode */
	cfg.fifo_config0.fifo_mode = (uint8_t)conf->base_conf.fifo_mode;

	/* Save FIFO mode so we can read M-1 frames when in streaming mode (AN-000364) */
	e->fifo_mode = conf->base_conf.fifo_mode;

	if (conf->base_conf.fifo_mode == FIFO_CONFIG0_FIFO_MODE_BYPASS) {
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

		e->fifo_comp_en = INV_IMU_DISABLE;
		e->fifo_is_used = INV_IMU_DISABLE;
	} else {
		/* 
		 * Enabling FIFO:
		 *  - Set `fifo_mode`
		 *  - Set `fifo_if_en` to 1
		 *  - Set `fifo_comp_en` (only applied if fifo_mode != BYPASS)
		 */
		/* `fifo_mode` */
		status |= inv_imu_write_reg(s, FIFO_CONFIG0, 1, (uint8_t *)&cfg.fifo_config0);
		/* `fifo_if_en` */
		cfg.fifo_config3.fifo_if_en = INV_IMU_ENABLE;
		status |= inv_imu_write_reg(s, FIFO_CONFIG3, 1, (uint8_t *)&cfg.fifo_config3);
		/* `fifo_comp_en` */
		cfg.fifo_config4.fifo_comp_en = conf->comp_en;
		status |= inv_imu_write_reg(s, FIFO_CONFIG4, 1, (uint8_t *)&cfg.fifo_config4);

		e->fifo_comp_en = conf->comp_en;
		e->fifo_is_used = INV_IMU_ENABLE;
	}

	/* Calculate FIFO frame size */
	if (!conf->es1_en && !conf->es0_en) {
		/* External sensor(s) disabled */
		if (conf->base_conf.hires_en) {
			fifo_frame_size = 20;
		} else {
			if (conf->base_conf.accel_en)
				fifo_frame_size += 8;
			if (conf->base_conf.gyro_en)
				fifo_frame_size += 8;
		}
	} else {
		/* External sensor(s) enabled */
		if (conf->base_conf.accel_en || conf->base_conf.gyro_en) { /* Accel and/or Gyro enabled */
			fifo_frame_size = 32;
		} else { /* Accel and Gyro are disabled */
			if (conf->es1_en && conf->es0_en) /* External Sensors 0 and 1 enabled */
				fifo_frame_size = 20;
			else /* Only one External Sensor enabled */
				fifo_frame_size = 16;
		}
	}
	s->fifo_frame_size = fifo_frame_size;

	status |= init_fifo_compression(s);

	return status;
}

uint32_t inv_imu_adv_get_timestamp_resolution_us(inv_imu_device_t *s)
{
	int               status = INV_IMU_OK;
	tmst_wom_config_t tmst_wom_config;

	status |= inv_imu_read_reg(s, TMST_WOM_CONFIG, 1, (uint8_t *)&tmst_wom_config);

	if (status != INV_IMU_OK)
		return 0;

	if (tmst_wom_config.tmst_resol == TMST_WOM_CONFIG_TMST_RESOL_16_US)
		return 16;
	else if (tmst_wom_config.tmst_resol == TMST_WOM_CONFIG_TMST_RESOL_1_US)
		return 1;

	// Should not happen, return 0
	return 0;
}

#if INV_IMU_CLKIN_SUPPORTED
int inv_imu_adv_enable_clkin_rtc(inv_imu_device_t *s)
{
	int                  status = INV_IMU_OK;
	rtc_config_t         otp_heater_rtc_config;
	sifs_i3c_stc_cfg_t   sifs_i3c_stc_cfg;
	ipreg_sys2_reg_123_t ipreg_sys2_reg_123;
	ipreg_sys1_reg_166_t ipreg_sys1_reg_166;

	/* Both I3CSM STC and CLKIN use the same interpolator. I3CSM STC has higher priority over the CLKIN.
	 * To use the CLKIN, user must disable the I3CSM STC by setting the I3C_STC_MODE = 0
	 */
	status |= inv_imu_read_reg(s, SIFS_I3C_STC_CFG, 1, (uint8_t *)&sifs_i3c_stc_cfg);
	sifs_i3c_stc_cfg.i3c_stc_mode = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, SIFS_I3C_STC_CFG, 1, (uint8_t *)&sifs_i3c_stc_cfg);

	/* Same as in I3CSM STC mode, the ACCEL_SRC_CTRL[1:0] and GYRO_SRC_CTRL[1:0]
	 * must be set to 2’b10 (FIR and interpolator on)
	 */
	status |= inv_imu_read_reg(s, IPREG_SYS2_REG_123, 1, (uint8_t *)&ipreg_sys2_reg_123);
	ipreg_sys2_reg_123.accel_src_ctrl = IPREG_SYS2_REG_123_ACCEL_SRC_CTRL_INTERPOLATOR_ON_FIR_ON;
	status |= inv_imu_write_reg(s, IPREG_SYS2_REG_123, 1, (uint8_t *)&ipreg_sys2_reg_123);

	status |= inv_imu_read_reg(s, IPREG_SYS1_REG_166, 1, (uint8_t *)&ipreg_sys1_reg_166);
	ipreg_sys1_reg_166.gyro_src_ctrl = IPREG_SYS1_REG_166_GYRO_SRC_CTRL_INTERPOLATOR_ON_FIR_ON;
	status |= inv_imu_write_reg(s, IPREG_SYS1_REG_166, 1, (uint8_t *)&ipreg_sys1_reg_166);

	status |= inv_imu_read_reg(s, RTC_CONFIG, 1, (uint8_t *)&otp_heater_rtc_config);
	otp_heater_rtc_config.rtc_mode = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, RTC_CONFIG, 1, (uint8_t *)&otp_heater_rtc_config);

	return status;
}

int inv_imu_adv_disable_clkin_rtc(inv_imu_device_t *s)
{
	int          status = INV_IMU_OK;
	rtc_config_t otp_heater_rtc_config;

	status |= inv_imu_read_reg(s, RTC_CONFIG, 1, (uint8_t *)&otp_heater_rtc_config);
	otp_heater_rtc_config.rtc_mode = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, RTC_CONFIG, 1, (uint8_t *)&otp_heater_rtc_config);

	return status;
}
#endif /* INV_IMU_CLKIN_SUPPORTED */

int inv_imu_adv_power_up_sram(inv_imu_device_t *s)
{
	int               status = INV_IMU_OK;
	fifo_sram_sleep_t fifo_sram_sleep;

	status |= inv_imu_read_reg(s, FIFO_SRAM_SLEEP, 1, (uint8_t *)&fifo_sram_sleep);
	fifo_sram_sleep.fifo_gsleep_shared_sram = 3;
	status |= inv_imu_write_reg(s, FIFO_SRAM_SLEEP, 1, (uint8_t *)&fifo_sram_sleep);

	return status;
}

int inv_imu_adv_power_down_sram(inv_imu_device_t *s)
{
	int               status = INV_IMU_OK;
	fifo_sram_sleep_t fifo_sram_sleep;

	status |= inv_imu_read_reg(s, FIFO_SRAM_SLEEP, 1, (uint8_t *)&fifo_sram_sleep);
	fifo_sram_sleep.fifo_gsleep_shared_sram = 0;
	status |= inv_imu_write_reg(s, FIFO_SRAM_SLEEP, 1, (uint8_t *)&fifo_sram_sleep);

	return status;
}

int inv_imu_adv_set_endianness(inv_imu_device_t *s, sreg_ctrl_sreg_data_endian_sel_t endianness)
{
	int         status = INV_IMU_OK;
	sreg_ctrl_t sreg_ctrl;

	status |= inv_imu_read_reg(s, SREG_CTRL, 1, (uint8_t *)&sreg_ctrl);
	sreg_ctrl.sreg_data_endian_sel = (uint8_t)endianness;
	status |= inv_imu_write_reg(s, SREG_CTRL, 1, (uint8_t *)&sreg_ctrl);

	if (!status)
		s->endianness_data = (uint8_t)endianness;

	return status;
}

/*
 * Static functions definition
 */
static int configure_serial_interface(inv_imu_device_t *s)
{
	int                 status            = INV_IMU_OK;
	intf_config1_ovrd_t intf_config1_ovrd = { 0 };

	switch (s->transport.serif_type) {
	case UI_I2C:
		break; /* Nothing to do */

	case UI_SPI4:
		/* Enable SPI 3/4 overide and set 4-wire mode */
		intf_config1_ovrd.ap_spi_34_mode_ovrd = INV_IMU_ENABLE;
		intf_config1_ovrd.ap_spi_34_mode_ovrd_val =
		    INTF_CONFIG1_OVRD_AP_SPI_34_MODE_OVRD_VAL_4_WIRE;
		status |= inv_imu_write_reg(s, INTF_CONFIG1_OVRD, 1, (uint8_t *)&intf_config1_ovrd);
		break;

	case UI_SPI3:
		/* Enable SPI 3/4 overide and set 3-wire mode */
		intf_config1_ovrd.ap_spi_34_mode_ovrd = INV_IMU_ENABLE;
		intf_config1_ovrd.ap_spi_34_mode_ovrd_val =
		    INTF_CONFIG1_OVRD_AP_SPI_34_MODE_OVRD_VAL_3_WIRE;
		status |= inv_imu_write_reg(s, INTF_CONFIG1_OVRD, 1, (uint8_t *)&intf_config1_ovrd);
		break;

	default:
		return INV_IMU_ERROR_BAD_ARG;
	}

	return status;
}

static int init_fifo_compression(inv_imu_device_t *s)
{
	inv_imu_adv_var_t *e = (inv_imu_adv_var_t *)s->adv_var;

	for (int i = 0; i < 3; i++) {
		e->accel_baseline[i] = 0x8000;
		e->gyro_baseline[i]  = 0x8000;
	}
	e->temp_baseline = 0x8000;

	e->accel_baseline_found = 0;
	e->gyro_baseline_found  = 0;
	e->temp_baseline_found  = 0;

	return 0;
}

#if INV_IMU_FSYNC_SUPPORTED
static int init_fsync_tag(inv_imu_device_t *s)
{
	int                status = INV_IMU_OK;
	inv_imu_adv_var_t *e      = (inv_imu_adv_var_t *)s->adv_var;
	fsync_config0_t    fsync_config0;

	status |= inv_imu_read_reg(s, FSYNC_CONFIG0, 1, (uint8_t *)&fsync_config0);
	e->fsync_tag = (fsync_config0_ap_fsync_sel_t)fsync_config0.ap_fsync_sel;

	return status;
}
#endif
