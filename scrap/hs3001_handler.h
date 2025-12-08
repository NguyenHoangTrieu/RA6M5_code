#ifndef HS3001_HANDLER_H
#define HS3001_HANDLER_H

#include "sensor_thread.h"
#include <stdint.h>

/* Typical conversion time for 14-bit humidity + temperature (datasheet: 35 ms) */
#define HS3001_MEASUREMENT_TIME_MS   (40U)

/* Status bits in first humidity byte */
#define HS3001_STATUS_VALID          (0x00U)
#define HS3001_STATUS_STALE          (0x01U)

/* Completion flag for HS3001 I2C transfers */
extern volatile uint8_t g_hs_i2c_done;

/**
 * @brief Callback used by rm_comms for the HS3001 I2C instance.
 *
 * This is wired in FSP config as the callback for g_hs_comms_i2c_cfg.
 * It simply sets a flag when a transfer is complete so that the
 * blocking helpers can wait on it.
 */
void hs_comms_i2c_callback(rm_comms_callback_args_t * p_args);

/**
 * @brief Initialize HS3001 rm_comms I2C instance.
 *
 * This opens the rm_comms wrapper for the HS3001 sensor, using the
 * already-configured g_hs_comms_i2c_ctrl/g_hs_comms_i2c_cfg.
 */
fsp_err_t hs3001_init(void);

/**
 * @brief Perform one measurement and return raw 14-bit values.
 *
 * This function:
 *  - Sends a Measurement Request (MR) to the sensor
 *  - Waits for the conversion time
 *  - Fetches 4 bytes of data (humidity + temperature)
 *  - Extracts 14-bit humidity and temperature codes
 *
 * @param[out] p_humidity_raw  14-bit raw humidity code (0..16383)
 * @param[out] p_temp_raw      14-bit raw temperature code (0..16383)
 * @param[out] p_status        Optional pointer to status bits (0 = valid, 1 = stale). Can be NULL.
 *
 * @return FSP_SUCCESS on success, or an FSP error code.
 */
fsp_err_t hs3001_read_raw(uint16_t * p_humidity_raw,
                          uint16_t * p_temp_raw,
                          uint8_t  * p_status);

/**
 * @brief Convert raw humidity code to %RH.
 *
 * Uses the formula from the datasheet:
 *   RH[%] = (Humidity_raw / 2^14) * 100
 */
float hs3001_raw_to_humidity(uint16_t humidity_raw);

/**
 * @brief Convert raw temperature code to degrees Celsius.
 *
 * Uses the formula from the datasheet:
 *   T[°C] = (Temperature_raw / 2^14) * 165 - 40
 */
float hs3001_raw_to_temperature(uint16_t temp_raw);

/**
 * @brief High-level API: read humidity and temperature in physical units.
 *
 * @param[out] p_humidity_rh   Relative humidity in %RH.
 * @param[out] p_temperature_c Temperature in °C.
 *
 * @return FSP_SUCCESS on success, or an FSP error code.
 */
fsp_err_t hs3001_read(float * p_humidity_rh, float * p_temperature_c);

#endif /* HS3001_HANDLER_H */
