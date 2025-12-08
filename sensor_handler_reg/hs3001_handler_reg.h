#ifndef HS3001_HANDLER_REG_H
#define HS3001_HANDLER_REG_H

#include "sensor_thread.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Typical conversion time for 14-bit humidity + temperature (datasheet: 35 ms)
 */
#define HS3001_MEASUREMENT_TIME_MS (40U)

/* Status bits in first humidity byte */
#define HS3001_STATUS_VALID (0x00U)
#define HS3001_STATUS_STALE (0x01U)

/* Completion flag (kept for API compatibility with HAL, but reg version does
 * not use IRQ) */
extern volatile uint8_t g_hs_i2c_done;

/**
 * @brief Callback rm_comms (stub in reg version, only for API compatibility).
 */
void hs_comms_i2c_callback(rm_comms_callback_args_t *p_args);

fsp_err_t hs3001_init(void);

fsp_err_t hs3001_read_raw(uint16_t *p_humidity_raw, uint16_t *p_temp_raw,
                          uint8_t *p_status);

float hs3001_raw_to_humidity(uint16_t humidity_raw);
float hs3001_raw_to_temperature(uint16_t temp_raw);

fsp_err_t hs3001_read(float *p_humidity_rh, float *p_temperature_c);

#ifdef __cplusplus
}
#endif

#endif /* HS3001_HANDLER_REG_H */
