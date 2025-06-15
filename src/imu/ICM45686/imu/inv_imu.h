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

#ifndef _INV_IMU_H_
#define _INV_IMU_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup IMU IMU
 *  @brief Describes IMU
 *  @{
 */

/** @file inv_imu.h */


/* Device ID */
#define ICM45686

/* Device description */
#define INV_IMU_STRING_ID            "ICM45686"
#define INV_IMU_WHOAMI               0xE9
#define INV_IMU_HIGH_FSR_SUPPORTED   1
#define INV_IMU_FSYNC_SUPPORTED      1
#define INV_IMU_USE_BASIC_SMD        0
#define INV_IMU_INT2_PIN_SUPPORTED   1
#define INV_IMU_I2C_MASTER_SUPPORTED 1
#define INV_IMU_CLKIN_SUPPORTED      1
#define INV_IMU_AUX1_SUPPORTED       1
#define INV_IMU_AUX2_SUPPORTED       0

#ifdef __cplusplus
}
#endif

#endif /* #ifndef _INV_IMU_H_ */

/** @} */
