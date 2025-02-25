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

/** @defgroup I2CM I2C master
 *  @brief Basic functions to drive I2C master interface of the device
 *  @{
 */

/** @file inv_imu_i2cm.h */

#ifndef _INV_IMU_I2CM_H_
#define _INV_IMU_I2CM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "./inv_imu_driver.h"

/** @brief Description of one I2C master operation */
struct inv_imu_i2cm_optype {
	uint8_t        r_n_w;
	uint8_t        reg_addr;
	uint8_t        len;
	const uint8_t *wdata;
};

/** @brief Description of all I2C master operations for one I2C slave */
typedef struct {
	uint8_t                    op_cnt;
	uint8_t                    i2c_addr;
	struct inv_imu_i2cm_optype op[4];
} inv_imu_i2c_master_cfg_t;

/** @brief Initialize I2C master interface to communicate with external sensor.
 *  This reconfigures pad scenario.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_init_i2cm(inv_imu_device_t *s);

/** @brief Desactivate I2C master interface.
 *  This reconfigures pad scenario.
 *  @param[in] s  Pointer to device.
 *  @return       0 on success, negative value on error.
 */
int inv_imu_uninit_i2cm(inv_imu_device_t *s);

/** @brief Configure I2C master interface for 1 or 2 transactions
 *  @param[in] s        Pointer to device.
 *  @param[in] cfg_es0  I2C transaction configuration for external sensor 0.
 *  @param[in] cfg_es1  I2C transaction configuration for external sensor 1.
 *  @return             0 on success, negative value on error.
 */
int inv_imu_configure_i2cm(inv_imu_device_t *s, const inv_imu_i2c_master_cfg_t *cfg_es0,
                           const inv_imu_i2c_master_cfg_t *cfg_es1);

/** @brief Forces or releases clock configuration to fit I2C master need to have either
 *         RCOSC or PLL running.
 *  @warning If no sensor is enabled while calling inv_imu_start_i2cm_ops(), it is mandatory
 *           to turn ON explicitely clock.
 *  @param[in] s       Pointer to device.
 *  @param[in] on_off  Set to 1 to force clock ON, 0 to release it.
 *  @return            0 on success, negative value on error.
 */
int inv_imu_i2cm_clock_force(inv_imu_device_t *s, uint8_t on_off);

/** @brief Start I2C master transaction
 *  @param[in] s          Pointer to device.
 *  @param[in] fast_mode  I2C clock will be in fast mode if set, in standard mode otherwise.
 *  @return               0 on success, negative value on error.
 *  @warning It is mandatory to call inv_imu_configure_i2cm() prior to this function
 *  @warning It is mandatory to have either RCOSC or PLL running : if no sensor is enabled
 *           and inv_imu_i2cm_clock_force() was not called to force clock ON, function
 *           will return an error.
 */
int inv_imu_start_i2cm_ops(inv_imu_device_t *s, uint8_t fast_mode);

/** @brief Get data read previously through I2C master interface
 *  @param[in] s       Pointer to device.
 *  @param[out] rdata  Data read.
 *  @param[in]  len    Length of data to read in bytes.
 *  @return            0 on success, negative value on error.
 */
int inv_imu_get_i2cm_data(inv_imu_device_t *s, uint8_t *rdata, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_I2CM_H_ */

/** @} */
