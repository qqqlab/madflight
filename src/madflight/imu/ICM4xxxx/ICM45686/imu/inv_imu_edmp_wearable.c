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
#include "./inv_imu_edmp_wearable.h"
#include "./inv_imu_edmp_defs.h"

/* B2S specific memory map */
#define EDMP_B2S_MOUNTING_MATRIX      0x8FE
#define EDMP_B2S_MOUNTING_MATRIX_SIZE 0x1

int inv_imu_edmp_b2s_init(inv_imu_device_t *s)
{
	int            status    = INV_IMU_OK;
	static uint8_t ram_img[] = {
#include "./edmp_ram_extended_features_image.h"
	};

	status |= inv_imu_edmp_init_apex(s);

	status |=
	    inv_imu_write_sram(s, (uint32_t)EDMP_RAM_FEATURE_PRGM_RAM_BASE, sizeof(ram_img), ram_img);

	status |= inv_imu_edmp_disable_r2w(s);

	return status;
}

int inv_imu_edmp_b2s_get_parameters(inv_imu_device_t *s, inv_imu_edmp_b2s_parameters_t *b2s_params)
{
	int status = INV_IMU_OK;

	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_B2S_MOUNTING_MATRIX,
	                                 (uint8_t *)&b2s_params->b2s_mounting_matrix);

	return status;
}

int inv_imu_edmp_b2s_set_parameters(inv_imu_device_t *                   s,
                                    const inv_imu_edmp_b2s_parameters_t *b2s_params)
{
	int status = INV_IMU_OK;

	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_B2S_MOUNTING_MATRIX,
	                                  (uint8_t *)&b2s_params->b2s_mounting_matrix);

	return status;
}

int inv_imu_edmp_b2s_enable(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	fifo_config0_t  fifo_config0;
	edmp_apex_en1_t edmp_apex_en1;

	status |= inv_imu_read_reg(s, FIFO_CONFIG0, 1, (uint8_t *)&fifo_config0);
	if (fifo_config0.fifo_depth == FIFO_CONFIG0_FIFO_DEPTH_MAX)
		return INV_IMU_ERROR;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	edmp_apex_en1.feature3_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);

	return status;
}

int inv_imu_edmp_b2s_disable(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en1_t edmp_apex_en1;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	edmp_apex_en1.feature3_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);

	return status;
}
