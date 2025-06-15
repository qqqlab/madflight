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

#include "./inv_imu_edmp.h"
#include "./inv_imu_edmp_compass.h"
#include "./inv_imu_edmp_defs.h"

int inv_imu_edmp_compass_init(inv_imu_device_t *s)
{
	int            status    = INV_IMU_OK;
	static uint8_t ram_img[] = {
#include "./edmp_ram_extended_features_image.h"
	};

	status |= inv_imu_edmp_init_apex(s);

	status |=
	    inv_imu_write_sram(s, (uint32_t)EDMP_RAM_FEATURE_PRGM_RAM_BASE, sizeof(ram_img), ram_img);

	return status;
}

int inv_imu_edmp_compass_enable_soft_iron_cor(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en1_t edmp_apex_en1;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	edmp_apex_en1.soft_hard_iron_corr_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);

	return status;
}

int inv_imu_edmp_compass_disable_soft_iron_cor(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en1_t edmp_apex_en1;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	edmp_apex_en1.soft_hard_iron_corr_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);

	return status;
}

int inv_imu_edmp_compass_set_frequency(inv_imu_device_t *                  s,
                                       const dmp_ext_sen_odr_cfg_ext_odr_t frequency)
{
	int                   status = INV_IMU_OK;
	dmp_ext_sen_odr_cfg_t dmp_ext_sen_odr_cfg;
	int_i2cm_source_t     int_i2cm_source;

	/* Get data from EDMP and FIFO : enable external sensor */
	status |= inv_imu_read_reg(s, DMP_EXT_SEN_ODR_CFG, 1, (uint8_t *)&dmp_ext_sen_odr_cfg);
	dmp_ext_sen_odr_cfg.ext_sensor_en = 1;
	dmp_ext_sen_odr_cfg.ext_odr       = frequency;
	status |= inv_imu_write_reg(s, DMP_EXT_SEN_ODR_CFG, 1, (uint8_t *)&dmp_ext_sen_odr_cfg);

	/* Trigger I2CM by internal odr event */
	/* int_status_i2cm_smc_ext_odr_en = 1, enable external sensor odr interrupt */
	status |= inv_imu_read_reg(s, INT_I2CM_SOURCE, 1, (uint8_t *)&int_i2cm_source);
	int_i2cm_source.int_status_i2cm_smc_ext_odr_en = 1;
	status |= inv_imu_write_reg(s, INT_I2CM_SOURCE, 1, (uint8_t *)&int_i2cm_source);

	return status;
}

int inv_imu_edmp_compass_enable_es(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	uint8_t         data;
	edmp_apex_enx_t cfg;

	/* Enable compass as external sensor for eDMP */
	data = 1;
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ES_RAM_IMAGE_EN, &data);
	data = 1;
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ES0_COMPASS_EN, &data);

	/* Optimize power when APEX features is at minimum */
	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 2, (uint8_t *)&cfg);
	if (cfg.edmp_apex_en0.pedo_en || cfg.edmp_apex_en0.tilt_en || cfg.edmp_apex_en0.ff_en ||
	    cfg.edmp_apex_en0.smd_en || cfg.edmp_apex_en0.tap_en || cfg.edmp_apex_en0.r2w_en ||
	    cfg.edmp_apex_en1.basic_smd_en)
		data = 0;
	else
		data = 1;
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ES_POWER_MODE, &data);

	return status;
}

int inv_imu_edmp_compass_disable_es(inv_imu_device_t *s)
{
	int     status = INV_IMU_OK;
	uint8_t data;

	/* Disable compass as external sensor for eDMP */
	data = 0;
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ES_RAM_IMAGE_EN, &data);
	data = 0;
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_ES0_COMPASS_EN, &data);

	return status;
}
