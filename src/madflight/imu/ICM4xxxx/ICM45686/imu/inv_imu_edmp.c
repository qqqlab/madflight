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
#include "./inv_imu_edmp_defs.h"

#define EDMP_ROM_START_ADDR_IRQ0 EDMP_ROM_BASE
#define EDMP_ROM_START_ADDR_IRQ1 (EDMP_ROM_BASE + 0x04)
#define EDMP_ROM_START_ADDR_IRQ2 (EDMP_ROM_BASE + 0x08)

static int check_dmp_odr_decimation(inv_imu_device_t *s);

int inv_imu_edmp_set_frequency(inv_imu_device_t *s, const dmp_ext_sen_odr_cfg_apex_odr_t frequency)
{
	int                   status = INV_IMU_OK;
	dmp_ext_sen_odr_cfg_t dmp_ext_sen_odr_cfg;

	status |= inv_imu_read_reg(s, DMP_EXT_SEN_ODR_CFG, 1, (uint8_t *)&dmp_ext_sen_odr_cfg);
	dmp_ext_sen_odr_cfg.apex_odr = frequency;
	status |= inv_imu_write_reg(s, DMP_EXT_SEN_ODR_CFG, 1, (uint8_t *)&dmp_ext_sen_odr_cfg);

	return status;
}

int inv_imu_edmp_init_apex(inv_imu_device_t *s)
{
	int                status = INV_IMU_OK;
	apex_buffer_mgmt_t apex_buffer_mgmt;
	fifo_sram_sleep_t  fifo_sram_sleep;
	uint8_t            value;

	/* Configure DMP address registers */
	status |= inv_imu_edmp_configure(s);

	/*
	 * Initialize read and write pointers for pedometer to buffer full condition. EDMP will always 
	 * write step count in EDMP_PED_STEP_CNT_BUF2.
	 * Initialize read and write pointers for freefall to 0.
	 */
	status |= inv_imu_read_reg(s, APEX_BUFFER_MGMT, 1, (uint8_t *)&apex_buffer_mgmt);
	apex_buffer_mgmt.step_count_host_rptr  = 2;
	apex_buffer_mgmt.step_count_edmp_wptr  = 0;
	apex_buffer_mgmt.ff_duration_host_rptr = 0;
	apex_buffer_mgmt.ff_duration_edmp_wptr = 0;
	status |= inv_imu_write_reg(s, APEX_BUFFER_MGMT, 1, (uint8_t *)&apex_buffer_mgmt);

	/* Same impl as inv_imu_adv_power_up_sram, duplicated here to prevent dependency */
	status |= inv_imu_read_reg(s, FIFO_SRAM_SLEEP, 1, (uint8_t *)&fifo_sram_sleep);
	fifo_sram_sleep.fifo_gsleep_shared_sram = 0x03;
	status |= inv_imu_write_reg(s, FIFO_SRAM_SLEEP, 1, (uint8_t *)&fifo_sram_sleep);

	/* Clear SRAM */
	value = 0;
	for (int i = 0; i < EDMP_ROM_DATA_SIZE; i++)
		status |= inv_imu_write_sram(s, (uint32_t)EDMP_RAM_BASE + i, 1, &value);

	status |= inv_imu_edmp_recompute_apex_decimation(s);

	return status;
}

int inv_imu_edmp_recompute_apex_decimation(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	uint8_t         value;
	edmp_apex_en0_t save_edmp_apex_en0;
	edmp_apex_en1_t save_edmp_apex_en1;
	edmp_apex_en0_t edmp_apex_en0 = { 0 };
	edmp_apex_en1_t edmp_apex_en1 = { 0 };
	reg_host_msg_t  reg_host_msg;

	/*
	 * Check that DMP is turned OFF before requesting init APEX and save DMP enabled bits before
	 * requesting init procedure
	 */
	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&save_edmp_apex_en0);
	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&save_edmp_apex_en1);
	if (save_edmp_apex_en1.edmp_enable)
		return INV_IMU_ERROR;

	/*
	 * Make sure that all DMP interrupts are masked by default, to not trigger unexpected algorithm
	 *  execution when initialization is done if any sensor is running
	 */
	value = 0x3F;
	status |= inv_imu_write_reg(s, STATUS_MASK_PIN_0_7, 1, &value);
	status |= inv_imu_write_reg(s, STATUS_MASK_PIN_8_15, 1, &value);
	status |= inv_imu_write_reg(s, STATUS_MASK_PIN_16_23, 1, &value);

	/* Trigger EDMP with on-demand mode */
	status |= inv_imu_edmp_unmask_int_src(s, INV_IMU_EDMP_INT0, EDMP_INT_SRC_ON_DEMAND_MASK);

	/*
	 * Request to execute init procedure, make sure init is the only feature enabled
	 * (overwrite previously saved config)
	 */
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en1.init_en     = INV_IMU_ENABLE;
	edmp_apex_en1.edmp_enable = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);

	status |= inv_imu_read_reg(s, REG_HOST_MSG, 1, (uint8_t *)&reg_host_msg);
	reg_host_msg.edmp_on_demand_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, REG_HOST_MSG, 1, (uint8_t *)&reg_host_msg);

	/* Wait 200 us to give enough time for EMDP to start running */
	inv_imu_sleep_us(s, 200);

	/* Wait for DMP execution to complete */
	status |= inv_imu_edmp_wait_for_idle(s);

	/* Reset states */
	status |= inv_imu_edmp_mask_int_src(s, INV_IMU_EDMP_INT0, EDMP_INT_SRC_ON_DEMAND_MASK);

	/* Restore original DMP state, with DMP necessarily disabled as it was checked at the beginning of this function */
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&save_edmp_apex_en0);
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&save_edmp_apex_en1);

	status |= inv_imu_edmp_unmask_int_src(s, INV_IMU_EDMP_INT0, EDMP_INT_SRC_ACCEL_DRDY_MASK);

	return status;
}

int inv_imu_edmp_get_apex_parameters(inv_imu_device_t *s, inv_imu_edmp_apex_parameters_t *p)
{
	int             status = INV_IMU_OK;
	edmp_apex_en1_t edmp_apex_en1;

	/* Pedometer */
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_PED_AMP_TH, (uint8_t *)&p->ped_amp_th);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_PED_STEP_CNT_TH, (uint8_t *)&p->ped_step_cnt_th);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_PED_STEP_DET_TH, (uint8_t *)&p->ped_step_det_th);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_PED_SB_TIMER_TH, (uint8_t *)&p->ped_sb_timer_th);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_PED_HI_EN_TH, (uint8_t *)&p->ped_hi_en_th);
	status |=
	    INV_IMU_READ_EDMP_SRAM(s, EDMP_PED_SENSITIVITY_MODE, (uint8_t *)&p->ped_sensitivity_mode);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_PED_LOW_EN_AMP_TH, (uint8_t *)&p->ped_low_en_amp_th);

	/* Tilt */
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TILT_WAIT_TIME, (uint8_t *)&p->tilt_wait_time);

#if INV_IMU_USE_BASIC_SMD
	/* Basic SMD */
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_BASICSMD_WIN, (uint8_t *)&p->basicsmd_win);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_BASICSMD_WIN_WAIT, (uint8_t *)&p->basicsmd_win_wait);
#else
	/* SMD */
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_SMD_SENSITIVITY, (uint8_t *)&p->smd_sensitivity);
#endif

	/* R2W */
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_R2W_SLEEP_TIME_OUT, (uint8_t *)&p->r2w_sleep_time_out);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_R2W_SLEEP_GESTURE_DELAY,
	                                 (uint8_t *)&p->r2w_sleep_gesture_delay);
	status |=
	    INV_IMU_READ_EDMP_SRAM(s, EDMP_R2W_MOUNTING_MATRIX, (uint8_t *)&p->r2w_mounting_matrix);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_R2W_GRAVITY_FILTER_GAIN,
	                                 (uint8_t *)&p->r2w_gravity_filter_gain);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_R2W_MOTION_THR_ANGLE_COSINE,
	                                 (uint8_t *)&p->r2w_motion_th_angle_cosine);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_R2W_MOTION_THR_TIMER_FAST,
	                                 (uint8_t *)&p->r2w_motion_th_timer_fast);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_R2W_MOTION_THR_TIMER_SLOW,
	                                 (uint8_t *)&p->r2w_motion_th_timer_slow);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_R2W_MOTION_PREV_GRAVITY_TIMEOUT,
	                                 (uint8_t *)&p->r2w_motion_prev_gravity_timeout);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_R2W_LAST_GRAVITY_MOTION_TIMER,
	                                 (uint8_t *)&p->r2w_last_gravity_motion_timer);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_R2W_LAST_GRAVITY_TIMEOUT,
	                                 (uint8_t *)&p->r2w_last_gravity_timeout);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_R2W_GESTURE_VALIDITY_TIMEOUT,
	                                 (uint8_t *)&p->r2w_gesture_validity_timeout);

	/* Freefall */
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_LOWG_PEAK_TH, (uint8_t *)&p->lowg_peak_th);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_LOWG_PEAK_TH_HYST, (uint8_t *)&p->lowg_peak_th_hyst);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_LOWG_TIME_TH, (uint8_t *)&p->lowg_time_th);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_HIGHG_PEAK_TH, (uint8_t *)&p->highg_peak_th);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_HIGHG_PEAK_TH_HYST, (uint8_t *)&p->highg_peak_th_hyst);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_HIGHG_TIME_TH, (uint8_t *)&p->highg_time_th);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_FF_MIN_DURATION, (uint8_t *)&p->ff_min_duration);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_FF_MAX_DURATION, (uint8_t *)&p->ff_max_duration);
	status |=
	    INV_IMU_READ_EDMP_SRAM(s, EDMP_FF_DEBOUNCE_DURATION, (uint8_t *)&p->ff_debounce_duration);

	/* Tap */
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_MIN_JERK, (uint8_t *)&p->tap_min_jerk);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_TMAX, (uint8_t *)&p->tap_tmax);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_TMIN, (uint8_t *)&p->tap_tmin);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_MAX_PEAK_TOL, (uint8_t *)&p->tap_max_peak_tol);
	status |=
	    INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_SMUDGE_REJECT_THR, (uint8_t *)&p->tap_smudge_reject_th);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_TAVG, (uint8_t *)&p->tap_tavg);

	/* CalMag */
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_SOFT_IRON_SENSITIVITY_MATRIX,
	                                 (uint8_t *)p->soft_iron_sensitivity_matrix);
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_HARD_IRON_OFFSET, (uint8_t *)p->hard_iron_offset);

	/* Power save */
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_POWER_SAVE_TIME, (uint8_t *)&p->power_save_time);
	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	p->power_save_en = edmp_apex_en1.power_save_en ? INV_IMU_ENABLE : INV_IMU_DISABLE;

	return status;
}

int inv_imu_edmp_set_apex_parameters(inv_imu_device_t *s, const inv_imu_edmp_apex_parameters_t *p)
{
	int             status = INV_IMU_OK;
	edmp_apex_enx_t cfg;

	/* DMP cannot be configured if it is running, hence make sure all APEX algorithms are off */
	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 2, (uint8_t *)&cfg);
	if (cfg.edmp_apex_en0.pedo_en || cfg.edmp_apex_en0.tilt_en || cfg.edmp_apex_en0.ff_en ||
	    cfg.edmp_apex_en0.smd_en || cfg.edmp_apex_en0.tap_en || cfg.edmp_apex_en0.r2w_en ||
	    cfg.edmp_apex_en1.basic_smd_en || cfg.edmp_apex_en1.soft_hard_iron_corr_en)
		return INV_IMU_ERROR;

	/* Pedometer */
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_PED_AMP_TH, (uint8_t *)&p->ped_amp_th);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_PED_STEP_CNT_TH, (uint8_t *)&p->ped_step_cnt_th);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_PED_PREV_STEP_CNT_TH,
	                                  (uint8_t *)&p->ped_step_cnt_th); /* same as step_cnt_th */
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_PED_STEP_DET_TH, (uint8_t *)&p->ped_step_det_th);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_PED_SB_TIMER_TH, (uint8_t *)&p->ped_sb_timer_th);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_PED_HI_EN_TH, (uint8_t *)&p->ped_hi_en_th);
	status |=
	    INV_IMU_WRITE_EDMP_SRAM(s, EDMP_PED_SENSITIVITY_MODE, (uint8_t *)&p->ped_sensitivity_mode);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_PED_LOW_EN_AMP_TH, (uint8_t *)&p->ped_low_en_amp_th);

	/* Tilt */
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_TILT_WAIT_TIME, (uint8_t *)&p->tilt_wait_time);

#if INV_IMU_USE_BASIC_SMD
	/* Basic SMD */
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_BASICSMD_WIN, (uint8_t *)&p->basicsmd_win);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_BASICSMD_WIN_WAIT, (uint8_t *)&p->basicsmd_win_wait);
#else
	/* SMD */
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_SMD_SENSITIVITY, (uint8_t *)&p->smd_sensitivity);
#endif

	/* R2W */
	status |=
	    INV_IMU_WRITE_EDMP_SRAM(s, EDMP_R2W_SLEEP_TIME_OUT, (uint8_t *)&p->r2w_sleep_time_out);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_R2W_SLEEP_GESTURE_DELAY,
	                                  (uint8_t *)&p->r2w_sleep_gesture_delay);
	status |=
	    INV_IMU_WRITE_EDMP_SRAM(s, EDMP_R2W_MOUNTING_MATRIX, (uint8_t *)&p->r2w_mounting_matrix);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_R2W_GRAVITY_FILTER_GAIN,
	                                  (uint8_t *)&p->r2w_gravity_filter_gain);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_R2W_MOTION_THR_ANGLE_COSINE,
	                                  (uint8_t *)&p->r2w_motion_th_angle_cosine);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_R2W_MOTION_THR_TIMER_FAST,
	                                  (uint8_t *)&p->r2w_motion_th_timer_fast);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_R2W_MOTION_THR_TIMER_SLOW,
	                                  (uint8_t *)&p->r2w_motion_th_timer_slow);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_R2W_MOTION_PREV_GRAVITY_TIMEOUT,
	                                  (uint8_t *)&p->r2w_motion_prev_gravity_timeout);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_R2W_LAST_GRAVITY_MOTION_TIMER,
	                                  (uint8_t *)&p->r2w_last_gravity_motion_timer);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_R2W_LAST_GRAVITY_TIMEOUT,
	                                  (uint8_t *)&p->r2w_last_gravity_timeout);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_R2W_GESTURE_VALIDITY_TIMEOUT,
	                                  (uint8_t *)&p->r2w_gesture_validity_timeout);

	/* Free Fall */
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_LOWG_PEAK_TH, (uint8_t *)&p->lowg_peak_th);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_LOWG_PEAK_TH_HYST, (uint8_t *)&p->lowg_peak_th_hyst);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_LOWG_TIME_TH, (uint8_t *)&p->lowg_time_th);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_HIGHG_PEAK_TH, (uint8_t *)&p->highg_peak_th);
	status |=
	    INV_IMU_WRITE_EDMP_SRAM(s, EDMP_HIGHG_PEAK_TH_HYST, (uint8_t *)&p->highg_peak_th_hyst);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_HIGHG_TIME_TH, (uint8_t *)&p->highg_time_th);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_FF_MIN_DURATION, (uint8_t *)&p->ff_min_duration);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_FF_MAX_DURATION, (uint8_t *)&p->ff_max_duration);
	status |=
	    INV_IMU_WRITE_EDMP_SRAM(s, EDMP_FF_DEBOUNCE_DURATION, (uint8_t *)&p->ff_debounce_duration);

	/* Tap */
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_TAP_MIN_JERK, (uint8_t *)&p->tap_min_jerk);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_TAP_TMAX, (uint8_t *)&p->tap_tmax);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_TAP_TMIN, (uint8_t *)&p->tap_tmin);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_TAP_MAX_PEAK_TOL, (uint8_t *)&p->tap_max_peak_tol);
	status |=
	    INV_IMU_WRITE_EDMP_SRAM(s, EDMP_TAP_SMUDGE_REJECT_THR, (uint8_t *)&p->tap_smudge_reject_th);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_TAP_TAVG, (uint8_t *)&p->tap_tavg);

	/* CalMag */
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_SOFT_IRON_SENSITIVITY_MATRIX,
	                                  (uint8_t *)p->soft_iron_sensitivity_matrix);
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_HARD_IRON_OFFSET, (uint8_t *)p->hard_iron_offset);

	/* Power save */
	status |= INV_IMU_WRITE_EDMP_SRAM(s, EDMP_POWER_SAVE_TIME, (uint8_t *)&p->power_save_time);
	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&cfg.edmp_apex_en1);
	cfg.edmp_apex_en1.power_save_en = p->power_save_en;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&cfg.edmp_apex_en1);

	return status;
}

int inv_imu_edmp_get_config_int_apex(inv_imu_device_t *s, inv_imu_edmp_int_state_t *it)
{
	int                status = INV_IMU_OK;
	int_apex_configx_t cfg;

	status |= inv_imu_read_reg(s, INT_APEX_CONFIG0, 2, (uint8_t *)&cfg);
	/* INT_APEX_CONFIG0 */
	it->INV_TAP           = !cfg.int_apex_config0.int_status_mask_pin_tap_detect;
	it->INV_HIGHG         = !cfg.int_apex_config0.int_status_mask_pin_high_g_det;
	it->INV_LOWG          = !cfg.int_apex_config0.int_status_mask_pin_low_g_det;
	it->INV_TILT_DET      = !cfg.int_apex_config0.int_status_mask_pin_tilt_det;
	it->INV_STEP_CNT_OVFL = !cfg.int_apex_config0.int_status_mask_pin_step_cnt_ovfl;
	it->INV_STEP_DET      = !cfg.int_apex_config0.int_status_mask_pin_step_det;
	it->INV_FF            = !cfg.int_apex_config0.int_status_mask_pin_ff_det;
	it->INV_R2W           = !cfg.int_apex_config0.int_status_mask_pin_r2w_wake_det;

	/* INT_APEX_CONFIG1 */
	it->INV_R2W_SLEEP = !cfg.int_apex_config1.int_status_mask_pin_r2w_sleep_det;
#if INV_IMU_USE_BASIC_SMD
	it->INV_SMD = !cfg.int_apex_config1.int_status_mask_pin_basic_smd;
#else
	it->INV_SMD = !cfg.int_apex_config1.int_status_mask_pin_smd_det;
#endif
	it->INV_SELF_TEST = !cfg.int_apex_config1.int_status_mask_pin_selftest_done;
	it->INV_SEC_AUTH  = !cfg.int_apex_config1.int_status_mask_pin_sa_done;

	return status;
}

int inv_imu_edmp_set_config_int_apex(inv_imu_device_t *s, const inv_imu_edmp_int_state_t *it)
{
	int                status = INV_IMU_OK;
	int_apex_configx_t cfg    = { 0 };

	/* INT_APEX_CONFIG0 */
	cfg.int_apex_config0.int_status_mask_pin_tap_detect    = !it->INV_TAP;
	cfg.int_apex_config0.int_status_mask_pin_high_g_det    = !it->INV_HIGHG;
	cfg.int_apex_config0.int_status_mask_pin_low_g_det     = !it->INV_LOWG;
	cfg.int_apex_config0.int_status_mask_pin_tilt_det      = !it->INV_TILT_DET;
	cfg.int_apex_config0.int_status_mask_pin_step_cnt_ovfl = !it->INV_STEP_CNT_OVFL;
	cfg.int_apex_config0.int_status_mask_pin_step_det      = !it->INV_STEP_DET;
	cfg.int_apex_config0.int_status_mask_pin_ff_det        = !it->INV_FF;
	cfg.int_apex_config0.int_status_mask_pin_r2w_wake_det  = !it->INV_R2W;

	/* INT_APEX_CONFIG1 */
	cfg.int_apex_config1.int_status_mask_pin_r2w_sleep_det = !it->INV_R2W_SLEEP;
#if INV_IMU_USE_BASIC_SMD
	cfg.int_apex_config1.int_status_mask_pin_basic_smd = !it->INV_SMD;
	cfg.int_apex_config1.int_status_mask_pin_smd_det   = 1;
#else
	cfg.int_apex_config1.int_status_mask_pin_smd_det   = !it->INV_SMD;
	cfg.int_apex_config1.int_status_mask_pin_basic_smd = 1;
#endif
	cfg.int_apex_config1.int_status_mask_pin_selftest_done = !it->INV_SELF_TEST;
	cfg.int_apex_config1.int_status_mask_pin_sa_done       = !it->INV_SEC_AUTH;

	status |= inv_imu_write_reg(s, INT_APEX_CONFIG0, 2, (uint8_t *)&cfg);

	return status;
}

int inv_imu_edmp_enable(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en1_t edmp_apex_en1;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	edmp_apex_en1.edmp_enable = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);

	return status;
}

int inv_imu_edmp_disable(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en1_t edmp_apex_en1;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	edmp_apex_en1.edmp_enable = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);

	return status;
}

/*
 * Check if inv_imu_edmp_set_frequency() was called without recomputing APEX decimation
 * thanks to inv_imu_edmp_recompute_apex_decimation().
 * This function will compare edmp decimation rate in IMU SRAM as computed at APEX init step vs the
 * DMP ODR currently written in IMU register.
 * Returns INV_IMU_ERROR_EDMP_ODR if inv_imu_edmp_recompute_apex_decimation() should have been
 * called.
 */
static int check_dmp_odr_decimation(inv_imu_device_t *s)
{
	int                            status = INV_IMU_OK;
	uint8_t                        dmp_decim_rate_from_sram;
	dmp_ext_sen_odr_cfg_t          dmp_ext_sen_odr_cfg;
	dmp_ext_sen_odr_cfg_apex_odr_t apex_odr;

	status |= inv_imu_read_sram(s, 0x460 /* edmp decimation rate address in SRAM */, 1,
	                            &dmp_decim_rate_from_sram);
	status |= inv_imu_read_reg(s, DMP_EXT_SEN_ODR_CFG, 1, (uint8_t *)&dmp_ext_sen_odr_cfg);
	apex_odr = (dmp_ext_sen_odr_cfg_apex_odr_t)dmp_ext_sen_odr_cfg.apex_odr;
	switch (dmp_decim_rate_from_sram) {
	case 15:
		if (apex_odr == DMP_EXT_SEN_ODR_CFG_APEX_ODR_800_HZ)
			return status;
		else
			return INV_IMU_ERROR_EDMP_ODR;
	case 7:
		if (apex_odr == DMP_EXT_SEN_ODR_CFG_APEX_ODR_400_HZ)
			return status;
		else
			return INV_IMU_ERROR_EDMP_ODR;
	case 3:
		if (apex_odr == DMP_EXT_SEN_ODR_CFG_APEX_ODR_200_HZ)
			return status;
		else
			return INV_IMU_ERROR_EDMP_ODR;
	case 1:
		if (apex_odr == DMP_EXT_SEN_ODR_CFG_APEX_ODR_100_HZ)
			return status;
		else
			return INV_IMU_ERROR_EDMP_ODR;
	case 0:
		if ((apex_odr == DMP_EXT_SEN_ODR_CFG_APEX_ODR_50_HZ) ||
		    (apex_odr == DMP_EXT_SEN_ODR_CFG_APEX_ODR_25_HZ))
			return status;
		else
			return INV_IMU_ERROR_EDMP_ODR;
	default:
		return INV_IMU_ERROR;
	}
}

int inv_imu_edmp_enable_pedometer(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	status |= check_dmp_odr_decimation(s);
	if (status)
		return status;

	/* Make sure pedometer is not already enabled to prevent messing up pointers */
	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	if (edmp_apex_en0.pedo_en)
		return status;

	/* Enable Pedometer */
	edmp_apex_en0.pedo_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_disable_pedometer(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en0.pedo_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_enable_smd(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
#if INV_IMU_USE_BASIC_SMD
	edmp_apex_en1_t edmp_apex_en1;

	status |= check_dmp_odr_decimation(s);
	if (status)
		return status;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	edmp_apex_en1.basic_smd_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
#else
	edmp_apex_en0_t edmp_apex_en0;

	status |= check_dmp_odr_decimation(s);
	if (status)
		return status;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en0.smd_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
#endif
	return status;
}

int inv_imu_edmp_disable_smd(inv_imu_device_t *s)
{
	int status = INV_IMU_OK;
#if INV_IMU_USE_BASIC_SMD
	edmp_apex_en1_t edmp_apex_en1;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
	edmp_apex_en1.basic_smd_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN1, 1, (uint8_t *)&edmp_apex_en1);
#else
	edmp_apex_en0_t edmp_apex_en0;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en0.smd_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
#endif
	return status;
}

int inv_imu_edmp_enable_tilt(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	status |= check_dmp_odr_decimation(s);
	if (status)
		return status;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en0.tilt_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_disable_tilt(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en0.tilt_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_enable_r2w(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	status |= check_dmp_odr_decimation(s);
	if (status)
		return status;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en0.r2w_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_disable_r2w(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en0.r2w_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_enable_tap(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	status |= check_dmp_odr_decimation(s);
	if (status)
		return status;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en0.tap_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_disable_tap(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en0.tap_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_enable_ff(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	status |= check_dmp_odr_decimation(s);
	if (status)
		return status;

	/* Make sure freefall is not already enabled to prevent messing up pointers */
	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	if (edmp_apex_en0.ff_en)
		return status;

	/* Enable freefall */
	edmp_apex_en0.ff_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_disable_ff(inv_imu_device_t *s)
{
	int             status = INV_IMU_OK;
	edmp_apex_en0_t edmp_apex_en0;

	status |= inv_imu_read_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);
	edmp_apex_en0.ff_en = INV_IMU_DISABLE;
	status |= inv_imu_write_reg(s, EDMP_APEX_EN0, 1, (uint8_t *)&edmp_apex_en0);

	return status;
}

int inv_imu_edmp_get_int_apex_status(inv_imu_device_t *s, inv_imu_edmp_int_state_t *it)
{
	int                status = INV_IMU_OK;
	int_apex_statusx_t st;

	/* Read APEX interrupt status */
	status |= inv_imu_read_reg(s, INT_APEX_STATUS0, 2, (uint8_t *)&st);

	it->INV_TAP           = st.int_apex_status0.int_status_tap_det;
	it->INV_HIGHG         = st.int_apex_status0.int_status_high_g_det;
	it->INV_LOWG          = st.int_apex_status0.int_status_low_g_det;
	it->INV_TILT_DET      = st.int_apex_status0.int_status_tilt_det;
	it->INV_STEP_CNT_OVFL = st.int_apex_status0.int_status_step_cnt_ovfl;
	it->INV_STEP_DET      = st.int_apex_status0.int_status_step_det;
	it->INV_FF            = st.int_apex_status0.int_status_ff_det;
	it->INV_R2W           = st.int_apex_status0.int_status_r2w_wake_det;
#if INV_IMU_USE_BASIC_SMD
	it->INV_SMD = st.int_apex_status1.int_status_basic_smd;
#else
	it->INV_SMD = st.int_apex_status1.int_status_smd_det;
#endif
	it->INV_R2W_SLEEP = st.int_apex_status1.int_status_r2w_sleep_det;
	it->INV_SELF_TEST = st.int_apex_status1.int_status_selftest_done;
	it->INV_SEC_AUTH  = st.int_apex_status1.int_status_sa_done;

	return status;
}

int inv_imu_edmp_get_pedometer_data(inv_imu_device_t *s, inv_imu_edmp_pedometer_data_t *ped_data)
{
	int     status = INV_IMU_OK;
	uint8_t data[2];
	uint8_t retry = 0;

	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_PED_ACTIVITY_CLASS, data);
	ped_data->activity_class = (inv_imu_edmp_activity_class_t)data[0];

	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_PED_STEP_CADENCE, data);
	ped_data->step_cadence = data[0];

	/*
	 * Always read BUF2 as we forced a buffer full
	 * configuration in `inv_imu_edmp_init_apex` function.
	 */
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_PED_STEP_CNT_BUF2, data);

	/*
	 * Read value multiple times in case the buffer was written while we were reading it.
	 * If we read the same value twice consecutively, it means we have a proper value.
	 */
	while (status == INV_IMU_OK) {
		uint8_t data_verif[2];
		status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_PED_STEP_CNT_BUF2, data_verif);

		if ((data[0] == data_verif[0]) && (data[1] == data_verif[1]))
			break;

		data[0] = data_verif[0];
		data[1] = data_verif[1];

		retry++;

		if (retry > 5)
			return INV_IMU_ERROR;
	}
	ped_data->step_cnt = data[1] << 8 | data[0];

	return status;
}

int inv_imu_edmp_get_ff_data(inv_imu_device_t *s, uint16_t *freefall_duration)
{
	int                status = INV_IMU_OK;
	uint8_t            data[2];
	apex_buffer_mgmt_t apex_buffer_mgmt;
	uint8_t            edmp_wptr, host_rptr;

	status |= inv_imu_read_reg(s, APEX_BUFFER_MGMT, 1, (uint8_t *)&apex_buffer_mgmt);
	host_rptr = apex_buffer_mgmt.ff_duration_host_rptr;
	edmp_wptr = apex_buffer_mgmt.ff_duration_edmp_wptr;

	if (host_rptr == edmp_wptr)
		return INV_IMU_ERROR_EDMP_BUF_EMPTY; // No data.

	if ((host_rptr & 0x1) == 0)
		status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_FF_DURATION_BUF1, data);
	else
		status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_FF_DURATION_BUF2, data);

	host_rptr = (host_rptr + 1) & 0x03;

	apex_buffer_mgmt.ff_duration_host_rptr = host_rptr;
	status |= inv_imu_write_reg(s, APEX_BUFFER_MGMT, 1, (uint8_t *)&apex_buffer_mgmt);

	*freefall_duration = (data[1] << 8) | data[0];

	return status;
}

int inv_imu_edmp_get_tap_data(inv_imu_device_t *s, inv_imu_edmp_tap_data_t *data)
{
	int status = INV_IMU_OK;

	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_NUM, (uint8_t *)&(data->num));
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_AXIS, (uint8_t *)&(data->axis));
	status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_TAP_DIR, (uint8_t *)&(data->direction));
	if (data->num == INV_IMU_EDMP_TAP_DOUBLE)
		status |= INV_IMU_READ_EDMP_SRAM(s, EDMP_DOUBLE_TAP_TIMING,
		                                 (uint8_t *)&(data->double_tap_timing));
	else
		data->double_tap_timing = 0;

	return status;
}

int inv_imu_edmp_mask_int_src(inv_imu_device_t *s, inv_imu_edmp_int_t edmp_int_nb, uint8_t int_mask)
{
	int      status = INV_IMU_OK;
	uint32_t reg_addr;
	uint8_t  reg;

	/*
	 * Interrupt mask register for EDMP interrupts are located in 3 consecutive
	 * registers starting with STATUS_MASK_PIN_0_7 for interrupt0.
	 */
	reg_addr = STATUS_MASK_PIN_0_7 + edmp_int_nb;

	/* Set bits passed in param to mask corresponding interrupts */
	status |= inv_imu_read_reg(s, reg_addr, 1, &reg);
	reg |= int_mask;
	status |= inv_imu_write_reg(s, reg_addr, 1, &reg);

	return status;
}

int inv_imu_edmp_unmask_int_src(inv_imu_device_t *s, inv_imu_edmp_int_t edmp_int_nb,
                                uint8_t int_mask)
{
	int      status = INV_IMU_OK;
	uint32_t reg_addr;
	uint8_t  reg;

	/*
	 * Interrupt mask register for EDMP interrupts are located in 3 consecutive
	 * registers starting with STATUS_MASK_PIN_0_7 for interrupt0.
	 */
	reg_addr = STATUS_MASK_PIN_0_7 + edmp_int_nb;

	/* Clear bits passed in param to unmask corresponding interrupts */
	status |= inv_imu_read_reg(s, reg_addr, 1, &reg);
	reg &= ~(int_mask);
	status |= inv_imu_write_reg(s, reg_addr, 1, &reg);

	return status;
}

int inv_imu_edmp_configure(inv_imu_device_t *s)
{
	int      status       = INV_IMU_OK;
	uint16_t start_addr[] = { EDMP_ROM_START_ADDR_IRQ0, EDMP_ROM_START_ADDR_IRQ1,
		                      EDMP_ROM_START_ADDR_IRQ2 };
	/* Only 8 MSB of SP address is written to register */
	uint8_t stack_addr = (uint8_t)(APEX_FEATURE_STACK_END >> 8);

	/* Set Start address for 3 edmp IRQ handlers */
	status |=
	    inv_imu_write_reg(s, EDMP_PRGRM_IRQ0_0, sizeof(start_addr), (uint8_t *)&start_addr[0]);

	/* Set Stack pointer start address */
	status |= inv_imu_write_reg(s, EDMP_SP_START_ADDR, sizeof(stack_addr), (uint8_t *)&stack_addr);

	return status;
}

int inv_imu_edmp_run_ondemand(inv_imu_device_t *s, inv_imu_edmp_int_t edmp_int_nb)
{
	int            status = INV_IMU_OK;
	reg_host_msg_t reg_host_msg;

	status |= inv_imu_edmp_unmask_int_src(s, edmp_int_nb, EDMP_INT_SRC_ON_DEMAND_MASK);

	status |= inv_imu_edmp_enable(s);

	status |= inv_imu_read_reg(s, REG_HOST_MSG, 1, (uint8_t *)&reg_host_msg);
	reg_host_msg.edmp_on_demand_en = INV_IMU_ENABLE;
	status |= inv_imu_write_reg(s, REG_HOST_MSG, 1, (uint8_t *)&reg_host_msg);

	return status;
}

int inv_imu_edmp_wait_for_idle(inv_imu_device_t *s)
{
	int          status = INV_IMU_OK;
	ipreg_misc_t ipreg_misc;
	int          timeout_us = 1000000; /* 1 sec */

	/* Wait for idle == 1 (indicates EDMP is not running, e.g execution is completed) */
	while (status == INV_IMU_OK) {
		status |= inv_imu_read_reg(s, IPREG_MISC, 1, (uint8_t *)&ipreg_misc);
		if (ipreg_misc.edmp_idle != 0)
			break;

		inv_imu_sleep_us(s, 5);
		timeout_us -= 5;

		if (timeout_us <= 0)
			return INV_IMU_ERROR_TIMEOUT;
	}

	return status;
}
