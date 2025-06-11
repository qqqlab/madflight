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

/** @defgroup WEARABLE EDMP Wearable
 *  @brief High-level functions to drive eDMP Wearable features
 *  @{
 */

/** @file inv_imu_edmp_wearable.h */

#ifndef _INV_IMU_WEARABLE_H_
#define _INV_IMU_WEARABLE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "./inv_imu_driver.h"

/** @brief IMU B2S parameters definition
 */
typedef struct {
	/* B2S */
	uint32_t b2s_mounting_matrix; /**< Specifies mounting matrix to be applied to B2S raw data
                                       Set bit 2 : swap X/Y ; flip Z
                                       Set bit 1 : flip X ; flip Z
                                       Set bit 0 : flip Y ; flip Z */
} inv_imu_edmp_b2s_parameters_t;

/** @brief  Initialize B2S algorithm. 
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_b2s_init(inv_imu_device_t *s);

/** @brief  Get current B2S configuration settings. 
 *  @param[in]  s           Pointer to device.
 *  @param[out] b2s_params  Pointer to B2S configuration structure, which will hold current B2S configuration.
 *  @return                 0 on success, negative value on error.
 */
int inv_imu_edmp_b2s_get_parameters(inv_imu_device_t *s, inv_imu_edmp_b2s_parameters_t *b2s_params);

/** @brief  Set new B2S configuration settings. 
 *  @param[in]  s           Pointer to device.
 *  @param[in]  b2s_params  Pointer to B2S configuration structure, which contains new B2S configuration.
 *  @return                 0 on success, negative value on error.
 */
int inv_imu_edmp_b2s_set_parameters(inv_imu_device_t *                   s,
                                    const inv_imu_edmp_b2s_parameters_t *b2s_params);

/** @brief  Enable APEX algorithm B2S. 
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_b2s_enable(inv_imu_device_t *s);

/** @brief  Disable APEX algorithm B2S.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_edmp_b2s_disable(inv_imu_device_t *s);

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_WEARABLE_H_ */

/** @} */
