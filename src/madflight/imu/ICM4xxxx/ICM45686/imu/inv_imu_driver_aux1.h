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

/** @defgroup DriverAux1 AUX1 driver
 *  @brief Basic functions to drive AUX1 interface of the device
 *  @{
 */

/** @file inv_imu_driver_aux1.h */

#ifndef _INV_IMU_DRIVER_AUX1_H_
#define _INV_IMU_DRIVER_AUX1_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "./inv_imu_defs.h"

#if INV_IMU_AUX1_SUPPORTED
// FIXME !!!
#include "./inv_imu_transport.h"

/** @brief AUX1 Interrupts definition */
typedef struct {
	/* INTx_CONFIG0 */
	uint8_t INV_OIS1;
	uint8_t INV_OIS1_AGC_RDY;
} inv_imu_aux1_int_state_t;

/** @brief Set accel power mode for AUX1 interface.
 *  @param[in] t   Pointer to transport.
 *  @param[in] en  Requested power mode (INV_IMU_ENABLE or INV_IMU_DISABLE).
 *  @return        0 on success, negative value on error.
 */
int inv_imu_set_aux1_accel_mode(inv_imu_transport_t *t, uint8_t en);

/** @brief Set data ready interrupt for AUX1 interface.
 *  @param[in] t   Pointer to transport.
 *  @param[in] en  Enable state (INV_IMU_ENABLE or INV_IMU_DISABLE).
 *  @param[in] it  Interrupt number.
 *  @return        0 on success, negative value on error.
 */
int inv_imu_set_aux1_drdy(inv_imu_transport_t *t, uint8_t en, inv_imu_int_num_t it);

/** @brief Read interrupt 1 status.
 *  @param[in] s    Pointer to device.
 *  @param[out] it  Status of each interrupt.
 *  @return         0 on success, negative value on error.
 */
int inv_imu_get_aux1_int_status(inv_imu_transport_t *t, inv_imu_aux1_int_state_t *it);

/** @brief Set accel full scale range for AUX1 interface.
 *  @param[in] t    Pointer to transport.
 *  @param[in] fsr  Requested full scale range.
 *  @return         0 on success, negative value on error.
 */
int inv_imu_set_aux1_accel_fsr(inv_imu_transport_t *t, fs_sel_aux_accel_fs_sel_t fsr);

/** @brief Get accel full scale range for AUX1 interface.
 *  @param[in] t     Pointer to transport.
 *  @param[out] fsr  Current full scale range for the AUX1 interface.
 *  @return          0 on success, negative value on error.
 */
int inv_imu_get_aux1_accel_fsr(inv_imu_transport_t *t, fs_sel_aux_accel_fs_sel_t *fsr);

/** @brief Set gyro power mode for AUX1 interface.
 *  @param[in] t   Pointer to transport.
 *  @param[in] en  Requested power mode (INV_IMU_ENABLE or INV_IMU_DISABLE).
 *  @return        0 on success, negative value on error.
 */
int inv_imu_set_aux1_gyro_mode(inv_imu_transport_t *t, uint8_t en);

/** @brief Set gyro full scale range for the AUX1 interface.
 *  @param[in] t    Pointer to transport.
 *  @param[in] fsr  Requested full scale range.
 *  @return         0 on success, negative value on error.
 */
int inv_imu_set_aux1_gyro_fsr(inv_imu_transport_t *t, fs_sel_aux_gyro_fs_sel_t fsr);

/** @brief Get gyro full scale range for the AUX1 interface.
 *  @param[in] t     Pointer to transport.
 *  @param[out] fsr  Current full scale range for the AUX1 interface.
 *  @return          0 on success, negative value on error.
 */
int inv_imu_get_aux1_gyro_fsr(inv_imu_transport_t *t, fs_sel_aux_gyro_fs_sel_t *fsr);

/** @brief Get AUX1 interface register data.
 *  @param[in] t             Pointer to transport.
 *  @param[out] sensor_data  Current accel, gyro and temperature data from the AUX1 data registers.
 *  @return                  0 on success, negative value on error.
 */
int inv_imu_get_aux1_register_data(inv_imu_transport_t *t, inv_imu_sensor_data_t *sensor_data);

#if INV_IMU_INT2_PIN_SUPPORTED
/** @brief Configure INT2 pin behavior.
 *  @param[in] s     Pointer to device.
 *  @param[in] conf  Structure with the requested configuration.
 *  @return          0 on success, negative value on error.
 */
int inv_imu_set_aux1_pin_config_int(inv_imu_transport_t *t, const inv_imu_int_pin_config_t *conf);
#endif

#endif /* INV_IMU_AUX1_SUPPORTED */

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_DRIVER_AUX1_H_ */

/** @} */
