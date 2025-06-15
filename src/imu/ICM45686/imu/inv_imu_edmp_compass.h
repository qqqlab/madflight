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

/** @defgroup EDMPMag EDMP Compass
 *  @brief High-level functions to drive eDMP Compass features
 *  @{
 */

/** @file inv_imu_edmp_compass.h */

#ifndef _INV_IMU_COMPASS_H_
#define _INV_IMU_COMPASS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "./inv_imu_driver.h"

/** @brief  Initialize Compass usage in eDMP. 
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_compass_init(inv_imu_device_t *s);

/** @brief  Enable APEX algorithm soft hard iron correlation.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_compass_enable_soft_iron_cor(inv_imu_device_t *s);

/** @brief  Disable APEX algorithm soft hard iron correlation.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_compass_disable_soft_iron_cor(inv_imu_device_t *s);

/** @brief Configure EDMP Output Data Rate for compass data.
 *  @param[in] s         Pointer to device.
 *  @param[in] frequency The requested frequency.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_compass_set_frequency(inv_imu_device_t *                  s,
                                       const dmp_ext_sen_odr_cfg_ext_odr_t frequency);

/** @brief  Enable compass handling by eDMP as external sensor.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_compass_enable_es(inv_imu_device_t *s);

/** @brief  Disable compass handling by eDMP as external sensor.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_compass_disable_es(inv_imu_device_t *s);

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_COMPASS_H_ */

/** @} */
