/*
 *
 * Copyright (c) [2024] by InvenSense, Inc.
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

#ifndef __INV_IMU_EDMP_MEMMAP_H__
#define __INV_IMU_EDMP_MEMMAP_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ped_amp_th
 *
 * Threshold of step detection sensitivity.
 * Low values increase detection sensitivity: reduce miss-detection.
 * High values reduce detection sensitivity: reduce false-positive.
 * Unit: g in q25
 * Range: [1006632 - 3019898]
 * Default: 2080374
 */
#define EDMP_PED_AMP_TH                                         0x3f0
#define EDMP_PED_AMP_TH_SIZE                                    4

/* ped_step_cnt_th
 *
 * Minimum number of steps that must be detected before step count is incremented.
 * Low values reduce latency but increase false positives.
 * High values increase step count accuracy but increase latency
 * Unit: Number of steps
 * Range: [0-15]
 * Default: 5
 */
#define EDMP_PED_STEP_CNT_TH                                    0x3dc
#define EDMP_PED_STEP_CNT_TH_SIZE                               2

/* ped_prev_step_cnt_th
 *
 * Intermediate number of steps needed to be buffered waiting the internal counting reached the ped_step_cnt_th (when ped_prev_step_cnt_th is lower than ped_step_cnt_th).
 * Note: as soon as ped_step_cnt_th is reached ped_prev_step_cnt_th value is set to ped_step_cnt_th.
 * Unit: number of steps
 * Range: [0-15]
 * Default value: 5
 * Recommendation: set same value as ped_step_cnt_th.
 */
#define EDMP_PED_PREV_STEP_CNT_TH                               0x352
#define EDMP_PED_PREV_STEP_CNT_TH_SIZE                          2

/* ped_step_det_th
 *
 * Minimum number of steps that must be detected before step event is signaled.
 * Low values reduce latency but increase false positives.
 * High values increase step event validity but increase latency.
 * Unit: number of steps
 * Range: [0-7]
 * Default: 2
 */
#define EDMP_PED_STEP_DET_TH                                    0x3de
#define EDMP_PED_STEP_DET_TH_SIZE                               2

/* ped_sb_timer_th
 *
 * Maximum permitted time between two consecutive steps.
 * While in the step buffer state, the step buffer count resets to 0 if a new step isn't detected for this amount of time (user is considered to have "stopped walking")
 * Unit: time in samples number
 * Range: [0 - 225] 
 * Default value: 150 for ODR = 50 Hz
 * Recommendation: it is linked to stop and go use case to avoid seeing the impact of pedo_step_cnt_thr and pedo_step_det_thr
 */
#define EDMP_PED_SB_TIMER_TH                                    0x3e2
#define EDMP_PED_SB_TIMER_TH_SIZE                               2

/* ped_hi_en_th
 *
 * Threshold to classify acceleration signal as motion not due to steps
 * High values improve vibration rejection.
 * Low values improve detection.
 * Unit: g in q25
 * Range: [2949120 - 5210112]
 * Default: 3506176
 */
#define EDMP_PED_HI_EN_TH                                       0x3f8
#define EDMP_PED_HI_EN_TH_SIZE                                  4

/* ped_sensitivity_mode
 *
 * Pedometer sensitivity mode.
 * Slow walk mode improves slow walk detection (less than 1 Hz) but the number of false positives may increase
 * Range: 0: Normal 1: Slow walk
 * Default: 0
 */
#define EDMP_PED_SENSITIVITY_MODE                               0x3ec
#define EDMP_PED_SENSITIVITY_MODE_SIZE                          1

/* ped_low_en_amp_th
 *
 * Threshold to select a valid step. Used to increase step detection for slow walk use case only.
 * Unit: g in q25
 * Range: [1006632 - 3523215]
 * Default: 2684354
 */
#define EDMP_PED_LOW_EN_AMP_TH                                  0x3e8
#define EDMP_PED_LOW_EN_AMP_TH_SIZE                             4

/* ped_step_cnt_buf1
 *
 * Number of steps done since the last init of the pedometer feature.
 * Filled in alternatively with ped_step_cnt_buf2.
 * Unit: number of steps
 */
#define EDMP_PED_STEP_CNT_BUF1                                  0x9a
#define EDMP_PED_STEP_CNT_BUF1_SIZE                             2

/* ped_step_cnt_buf2
 *
 * Number of steps done since the last init of the pedometer feature.
 * Filled in alternatively with ped_step_cnt_buf1.
 * Unit: number of steps
 */
#define EDMP_PED_STEP_CNT_BUF2                                  0x9c
#define EDMP_PED_STEP_CNT_BUF2_SIZE                             2

/* ped_step_cadence
 *
 * Instant step cadence measured by the algorithm
 * Unit: 4*number of samples between two consecutive steps. Cadency (step/s) = (ped_step_cadence / 4) / (pedometer_ ODR).
 */
#define EDMP_PED_STEP_CADENCE                                   0x9f
#define EDMP_PED_STEP_CADENCE_SIZE                              1

/* ped_activity_class
 *
 * Activity classification of step detected
 * Unit: Enum: unknown (0), walk(1), run(2)
 */
#define EDMP_PED_ACTIVITY_CLASS                                 0xa0
#define EDMP_PED_ACTIVITY_CLASS_SIZE                            1

/* tilt_wait_time
 *
 * Minimum duration for which the device should be tilted before signaling event.
 * Unit: time in sample number
 * Range: [0 - 65536]
 * Default: 200 for ODR = 50 Hz, 100 for ODR = 25 Hz
 */
#define EDMP_TILT_WAIT_TIME                                     0x188
#define EDMP_TILT_WAIT_TIME_SIZE                                2

/* tilt_reset_en
 *
 * Set 1 to reset tilt prior to any further tilt processing on next sensor data.
 * Unit: N/A
 * Range: [0 - 1]
 * Default: 0
 */
#define EDMP_TILT_RESET_EN                                      0x92
#define EDMP_TILT_RESET_EN_SIZE                                 1

/* quat_reset_en
 *
 * Set 1 to force reset 3-axis quaternion when next tilt reset is done. This is applicable only if tilt_reset_en is also set to 1.
 * Unit: N/A
 * Range: [0 - 1]
 * Default: 0
 */
#define EDMP_QUAT_RESET_EN                                      0x5c
#define EDMP_QUAT_RESET_EN_SIZE                                 1

/* smd_sensitivity
 *
 * Parameter to tune SMD algorithm robustness to rejection, ranging from 0 to 4 (values higher than 4 are reserved).
 * Low values increase detection rate but increase false positives.
 * High values reduce false positives but reduce detection rate (especially for transport use cases).
 * Unit: N/A
 * Ranging: [0 - 4]
 * Default: 0
 */
#define EDMP_SMD_SENSITIVITY                                    0x412
#define EDMP_SMD_SENSITIVITY_SIZE                               1

/* basicsmd_win
 *
 * The window time in number of samples to wait continuous WOM before trigger SMD
 * Unit: time in sample number
 * Range: [25 - 500] 
 * Default value: 150
 * Recommended value at 50Hz = 150
 * Recommended value at 25Hz = 75
 */
#define EDMP_BASICSMD_WIN                                       0x110
#define EDMP_BASICSMD_WIN_SIZE                                  4

/* basicsmd_win_wait
 *
 * The window time in number of samples before stop trigger SMD after WOM stop reported motion
 * Unit: time in sample number
 * Range: [25 - 500] 
 * Default value: 50
 * Recommended value at 50Hz = 50
 * Recommended value at 25Hz = 25
 */
#define EDMP_BASICSMD_WIN_WAIT                                  0x114
#define EDMP_BASICSMD_WIN_WAIT_SIZE                             4

/* r2w_sleep_time_out
 *
 * Defines the duration after wake event to report sleep event no matter if position changes or not.
 * Unit: time in ms (millisecond)
 * Range: [100 - 10000]
 * Default: 640 (equivalent to 0.64s)
 */
#define EDMP_R2W_SLEEP_TIME_OUT                                 0x21c
#define EDMP_R2W_SLEEP_TIME_OUT_SIZE                            4

/* r2w_sleep_gesture_delay
 *
 * Defines the minimal duration of sleep position before trigger the sleep event.
 * Unit: time in ms (millisecond)
 * Range: [0 - 256]
 * Default: 96 (equivalent to 0.096 s)
 */
#define EDMP_R2W_SLEEP_GESTURE_DELAY                            0x220
#define EDMP_R2W_SLEEP_GESTURE_DELAY_SIZE                       4

/* r2w_mounting_matrix
 *
 * Mounting matrix to rotate data from chip frame to device frame
 * Unit: N/A
 * Range: 3 lower bits only are used [b2 b1 b0]: 
 * -	b2 = 1 swap X and Y
 * -	b1 = 1 flip X sign
 * -	b0 = 1 flip Y sign
 * Default value: 0 (chip frame aligned with android frame)
 */
#define EDMP_R2W_MOUNTING_MATRIX                                0x224
#define EDMP_R2W_MOUNTING_MATRIX_SIZE                           4

/* r2w_gravity_filter_gain
 *
 * Gain used to filter the accelerometer to obtain an estimation of the gravity (low-pass filter), defined as : forgetting factor = Gain * SAMPLING_PERIOD / (40 * 32), and 100Hz.
 * Range: [2-16]
 * Default: 6 for ODR = 50Hz, 8 for ODR = 25Hz
 */
#define EDMP_R2W_GRAVITY_FILTER_GAIN                            0x22c
#define EDMP_R2W_GRAVITY_FILTER_GAIN_SIZE                       4

/* r2w_motion_thr_angle_cosine
 *
 * Set the minimal angle that needed to be applied to device to detect R2W
 * Unit: fixed point value q30 of cosine of the angle
 * Range: [130856211 - 1069655912], corresponding to angle between 5 and 85 degrees
 * Default: 1046221864, corresponding to an angle of 13 degrees
 */
#define EDMP_R2W_MOTION_THR_ANGLE_COSINE                        0x230
#define EDMP_R2W_MOTION_THR_ANGLE_COSINE_SIZE                   4

/* r2w_motion_thr_timer_fast
 *
 * Timer relative to the rapidity of the algorithm to trigger wake up when the orientation before motion is Y axis up (with less than 30 degrees of inclination).
 * Unit: ms (no dependency on ODR, it is managed internally by the algorithm)
 * Range: [100 - 500]
 * Default: 240
 */
#define EDMP_R2W_MOTION_THR_TIMER_FAST                          0x234
#define EDMP_R2W_MOTION_THR_TIMER_FAST_SIZE                     4

/* r2w_motion_thr_timer_slow
 *
 * Timer relative to the rapidity of the algorithm to trigger wake up when the orientation before motion is over 30 degrees on the Y axis
 * Unit: ms (no dependency on ODR, it is managed internally by the algorithm)
 * Range: [240- 1000]
 * Default: 500
 */
#define EDMP_R2W_MOTION_THR_TIMER_SLOW                          0x238
#define EDMP_R2W_MOTION_THR_TIMER_SLOW_SIZE                     4

/* r2w_motion_prev_gravity_timeout
 *
 * Time delay to update internal value of previous gravity when no motion is detected.
 * Longer time enables detection motion during slower gesture.
 * Unit: ms (no dependency on ODR, it is managed internally by the algorithm)
 * Range: [100 - 1000]
 * Default: 300
 */
#define EDMP_R2W_MOTION_PREV_GRAVITY_TIMEOUT                    0x23c
#define EDMP_R2W_MOTION_PREV_GRAVITY_TIMEOUT_SIZE               4

/* r2w_last_gravity_motion_timer
 *
 * Time delay to update the current gravity estimator when no motion is detected.
 * Unit: ms (no dependency on ODR, it is managed internally by the algorithm)
 * Range: [100 - 1000]
 * Default: 480
 */
#define EDMP_R2W_LAST_GRAVITY_MOTION_TIMER                      0x240
#define EDMP_R2W_LAST_GRAVITY_MOTION_TIMER_SIZE                 4

/* r2w_last_gravity_timeout
 *
 * Time delay to update gravity in case motion is detected all the time, force to update gravity estimator even if the device is not stable.
 * Unit: ms (no dependency on ODR, it is managed internally by the algorithm)
 * Range: [1000 - 10000]
 * Default: 2600
 */
#define EDMP_R2W_LAST_GRAVITY_TIMEOUT                           0x244
#define EDMP_R2W_LAST_GRAVITY_TIMEOUT_SIZE                      4

/* r2w_gesture_validity_timeout
 *
 * If gesture is not completed in this timeout limit, gesture is invalid.
 * Unit: ms (no dependency on ODR, it is managed internally by the algorithm)
 * Range: [100 - 1000]
 * Default: 240
 */
#define EDMP_R2W_GESTURE_VALIDITY_TIMEOUT                       0x248
#define EDMP_R2W_GESTURE_VALIDITY_TIMEOUT_SIZE                  4

/* lowg_peak_th
 *
 * Threshold for accel values below which low-g state is detected.
 * Unit: g in q12
 * Range: [128 - 4096]
 * Default: 2048
 */
#define EDMP_LOWG_PEAK_TH                                       0x13c
#define EDMP_LOWG_PEAK_TH_SIZE                                  2

/* lowg_peak_th_hyst
 *
 * Hysteresis value added to the low-g threshold after accel values get below threshold.
 * Unit: g in q12
 * Range: [128 - 1024]
 * Default: 128
 */
#define EDMP_LOWG_PEAK_TH_HYST                                  0x13e
#define EDMP_LOWG_PEAK_TH_HYST_SIZE                             2

/* lowg_time_th
 *
 * Number of samples required to enter low-g state.
 * Unit: time in samples number
 * Range: [1 - 300]
 * Default: 13 (set for default ODR = 800 Hz, equivalent to 16 ms)
 */
#define EDMP_LOWG_TIME_TH                                       0x140
#define EDMP_LOWG_TIME_TH_SIZE                                  2

/* highg_peak_th
 *
 * Threshold for accel values above which high-g state is detected.
 * Unit: g in q12
 * Range: [1024 - 32768]
 * Default: 29696
 */
#define EDMP_HIGHG_PEAK_TH                                      0x130
#define EDMP_HIGHG_PEAK_TH_SIZE                                 2

/* highg_peak_th_hyst
 *
 * Hysteresis value subtracted from the high-g threshold after exceeding it.
 * Unit: g in q12
 * Range: [128 - 1024]
 * Default: 640
 */
#define EDMP_HIGHG_PEAK_TH_HYST                                 0x132
#define EDMP_HIGHG_PEAK_TH_HYST_SIZE                            2

/* highg_time_th
 *
 * The number of samples device should stay above (highg_peak_th + highg_peak_th_hyst) before HighG state is triggered.
 * Unit: time in samples number
 * Range: [1-300]
 * Default: 1 (set for default ODR = 800 Hz, equivalent to 1.25 ms)
 */
#define EDMP_HIGHG_TIME_TH                                      0x134
#define EDMP_HIGHG_TIME_TH_SIZE                                 2

/* ff_min_duration
 *
 * Minimum freefall duration. Shorter freefalls are ignored.
 * Unit: time in samples number
 * Range: [4 - 420]
 * Default: 57 (set for default ODR = 400 Hz, equivalent to 142 ms)
 */
#define EDMP_FF_MIN_DURATION                                    0x120
#define EDMP_FF_MIN_DURATION_SIZE                               4

/* ff_max_duration
 *
 * Maximum freefall duration. Longer freefalls are ignored.
 * Unit: time in samples number
 * Range: [12 - 1040]
 * Default: 285 (set for default ODR = 400 Hz, equivalent to 712 ms)
 */
#define EDMP_FF_MAX_DURATION                                    0x124
#define EDMP_FF_MAX_DURATION_SIZE                               4

/* ff_debounce_duration
 *
 * Period after a freefall is signaled during which a new freefall will not be detected. Prevents false detection due to bounces.
 * Unit: time in samples number
 * Range: [75 - 3000]
 * Default: 800 (set for default ODR = 800 Hz, equivalent to 1 s)
 */
#define EDMP_FF_DEBOUNCE_DURATION                               0x128
#define EDMP_FF_DEBOUNCE_DURATION_SIZE                          4

/* ff_duration_buf1
 *
 * Duration of the freefall.
 * Filled in alternatively with ff_duration_buf2.
 * Unit: number of samples. Freefall duration in seconds is ff_duration_buf1 / ACCEL_ODR_Hz
 */
#define EDMP_FF_DURATION_BUF1                                   0x88
#define EDMP_FF_DURATION_BUF1_SIZE                              2

/* ff_duration_buf2
 *
 * Duration of the freefall.
 * Filled in alternatively with ff_duration_buf1.
 * Unit: number of samples. Freefall duration in seconds is ff_duration_buf2 / ACCEL_ODR_Hz
 */
#define EDMP_FF_DURATION_BUF2                                   0x8a
#define EDMP_FF_DURATION_BUF2_SIZE                              2

/* tap_min_jerk
 *
 * The minimal value of jerk to be considered as a tap candidate.
 * Unit: g in q6
 * Range: [0 - 64]
 * Default: 17
 */
#define EDMP_TAP_MIN_JERK                                       0x193
#define EDMP_TAP_MIN_JERK_SIZE                                  1

/* tap_tmax
 *
 * Size of the analysis window to detect tap events (single or double tap)
 * Unit: time in sample number
 * Range: [49 - 496]
 * Default : 99 (set for default ODR = 200 Hz, equivalent to 0.5 s)
 */
#define EDMP_TAP_TMAX                                           0x190
#define EDMP_TAP_TMAX_SIZE                                      2

/* tap_tmin
 *
 * Single tap window, sub-windows within Tmax to detect single-tap event.
 * Unit: time in sample number
 * Range: [24 - 184]
 * Default: 33 (set for default ODR = 200 Hz, equivalent to 0.165 s)
 */
#define EDMP_TAP_TMIN                                           0x192
#define EDMP_TAP_TMIN_SIZE                                      1

/* tap_max_peak_tol
 *
 * Maximum peak tolerance is the percentage of pulse amplitude to get the smudge threshold for rejection.
 * Unit: N/A
 * Range: [1 (12.5%) 2 (25.0%) 3 (37.5%) 4 (50.0 %)]
 * Default: 2
 */
#define EDMP_TAP_MAX_PEAK_TOL                                   0x195
#define EDMP_TAP_MAX_PEAK_TOL_SIZE                              1

/* tap_smudge_reject_thr
 *
 * max acceptable number of samples (jerk value) over  tap_max_peak_tol the during the Tmin window. Over this value, Tap event is rejected
 * unit: time in number of samples
 * range: [13 â€“ 92]
 * Default: 17 (set for default ODR = 200 Hz, equivalent to 0.085 s)
 */
#define EDMP_TAP_SMUDGE_REJECT_THR                              0x194
#define EDMP_TAP_SMUDGE_REJECT_THR_SIZE                         1

/* tap_tavg
 *
 * Energy measurement window size to determine the tap axis associated with the 1st tap.
 * Unit: time in sample number
 * Range: [1 ; 2 ; 4 ; 8]
 * Default: 8
 * 
 */
#define EDMP_TAP_TAVG                                           0x196
#define EDMP_TAP_TAVG_SIZE                                      1

/* tap_num
 *
 * type of the last reported TAP event:
 * 0: no tap, 1: single tap, 2: double tap
 */
#define EDMP_TAP_NUM                                            0x8d
#define EDMP_TAP_NUM_SIZE                                       1

/* tap_axis
 *
 * Indicate the axis of the tap in the device frame
 * 0: ax, 1: ay, 2: az
 */
#define EDMP_TAP_AXIS                                           0x8e
#define EDMP_TAP_AXIS_SIZE                                      1

/* tap_dir
 *
 * Indicate the direction of the tap in the device frame
 * 0: positive, 1: negative
 */
#define EDMP_TAP_DIR                                            0x8f
#define EDMP_TAP_DIR_SIZE                                       1

/* double_tap_timing
 *
 * In case of double tap, indicate the sample count between the two detected pulses. Double tap timing in seconds is double_tap_timing / ACCEL_ODR_Hz.
 */
#define EDMP_DOUBLE_TAP_TIMING                                  0x90
#define EDMP_DOUBLE_TAP_TIMING_SIZE                             1

/* soft_iron_sensitivity_matrix
 *
 * Input 3x3 calibration matrix in q14 format applied to uncalib data.
 */
#define EDMP_SOFT_IRON_SENSITIVITY_MATRIX                       0x490
#define EDMP_SOFT_IRON_SENSITIVITY_MATRIX_SIZE                  36

/* hard_iron_offset
 *
 * 3-dimension vector that is removed to magnetometer data.
 */
#define EDMP_HARD_IRON_OFFSET                                   0x4b4
#define EDMP_HARD_IRON_OFFSET_SIZE                              12

/* es0_compass_en
 *
 * Set 1 for compass support with es0, for other external sensors with es0, set 0.
 */
#define EDMP_ES0_COMPASS_EN                                     0xb9
#define EDMP_ES0_COMPASS_EN_SIZE                                1

/* es_power_mode
 *
 * Set 1 for possibility of power savings when APEX features utilization is minimal.
 * Set 0 when APEX features utilization is at maximum.
 */
#define EDMP_ES_POWER_MODE                                      0xba
#define EDMP_ES_POWER_MODE_SIZE                                 1

/* es_ram_image_en
 *
 * Set 1 to load device specific RAM image for external sensor support, otherwise set 0
 */
#define EDMP_ES_RAM_IMAGE_EN                                    0xb6
#define EDMP_ES_RAM_IMAGE_EN_SIZE                               1

/* power_save_time
 *
 * Time of inactivity after which eDMP goes in power save mode.
 * Unit: time in sample number
 * Range: [0 - 4294967295] 
 * Default value: 6400
 */
#define EDMP_POWER_SAVE_TIME                                    0xc4
#define EDMP_POWER_SAVE_TIME_SIZE                               4

/* stc_results
 *
 * Results/status from self-test run
 * bit0: AX Self-test result (0:pass 1:fail)
 * bit1: AY Self-test result (0:pass 1:fail)
 * bit2: AZ Self-test result (0:pass 1:fail)
 * bit3: GX Self-test result (0:pass 1:fail)
 * bit4: GY Self-test result (0:pass 1:fail)
 * bit5: GZ Self-test result (0:pass 1:fail)
 * bit6~7: Self-test status (0:Done 1:InProgress 2:Error)
 */
#define EDMP_STC_RESULTS                                        0x44
#define EDMP_STC_RESULTS_SIZE                                   4

/* stc_configParams
 *
 * Self-test input parameters
 * bit0: If set, enable self-test init, must be set when any of accel or gyro self-test enable bit is set (bits 2:1)
 * bit1: If set, enable accel self-test 
 * bit2: If set, enable gyro self-test
 * bit7~9: Averaging time used to perform self-test (0/1/2/3/4/5: 10/20/40/80/160/320 ms)
 * bit10~12: Tolerance between factory trim and accel self-test response (0/1/2/3/4/5/6/7: 5/10/15/20/25/30/40/50%)
 * bit13~15: Tolerance between factory trim and gyro self-test response (0/1/2/3/4/5/6/7: 5/10/15/20/25/30/40/50%)
 */
#define EDMP_STC_CONFIGPARAMS                                   0x38
#define EDMP_STC_CONFIGPARAMS_SIZE                              4

/* stc_patch_en
 *
 * Mechanism for enabling patches execution in SRAM for self-test operations
 * bit0: If set, enable SRAM patching for accel self-test phase 1
 * bit1: If set, enable SRAM patching for accel self-test phase 2
 * bit2: If set, enable SRAM patching for gyro self-test phase 1
 * bit3: If set, enable SRAM patching for gyro self-test phase 2
 */
#define EDMP_STC_PATCH_EN                                       0x3c
#define EDMP_STC_PATCH_EN_SIZE                                  4

/* stc_debug_en
 *
 * Debug capability of self-test feature. Must be set to 0 at anytime when self-test is requested.
 */
#define EDMP_STC_DEBUG_EN                                       0x40
#define EDMP_STC_DEBUG_EN_SIZE                                  4

/* gyro_x_str_ft
 *
 * Self-test response for gyro X axis
 */
#define EDMP_GYRO_X_STR_FT                                      0x0
#define EDMP_GYRO_X_STR_FT_SIZE                                 2

/* gyro_y_str_ft
 *
 * Self-test response for gyro Y axis
 */
#define EDMP_GYRO_Y_STR_FT                                      0x2
#define EDMP_GYRO_Y_STR_FT_SIZE                                 2

/* gyro_z_str_ft
 *
 * Self-test response for gyro Z axis
 */
#define EDMP_GYRO_Z_STR_FT                                      0x4
#define EDMP_GYRO_Z_STR_FT_SIZE                                 2

#ifdef __cplusplus
}
#endif

#endif // __INV_IMU_EDMP_MEMMAP_H__
