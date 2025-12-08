#include "hs3001_handler.h"

/* Use the HS I2C rm_comms instance generated in sensor_thread.c */
extern rm_comms_i2c_instance_ctrl_t g_hs_comms_i2c_ctrl;
extern const rm_comms_cfg_t g_hs_comms_i2c_cfg;

volatile uint8_t g_hs_i2c_done = 0U;

/**
 * @brief rm_comms callback for HS3001 I2C transfers.
 *
 * This is referenced by g_hs_comms_i2c_cfg as its p_callback.
 * It simply sets g_hs_i2c_done when an I2C operation completes.
 */
void hs_comms_i2c_callback(rm_comms_callback_args_t *p_args) {
  if ((NULL != p_args) &&
      (p_args->event == RM_COMMS_EVENT_OPERATION_COMPLETE)) {
    g_hs_i2c_done = 1U;
  }
}

/**
 * @brief Blocking write using rm_comms for HS3001.
 */
static fsp_err_t hs3001_i2c_write(uint8_t const *p_data, uint32_t len) {
  fsp_err_t err = RM_COMMS_I2C_Write(&g_hs_comms_i2c_ctrl, p_data, len);
  if (FSP_SUCCESS == err) {
    /* Wait until transfer is done (simple busy-wait) */
    while (0U == g_hs_i2c_done) {
      __NOP();
    }
    g_hs_i2c_done = 0U;
  }

  return err;
}

/**
 * @brief Blocking read using rm_comms for HS3001.
 */
static fsp_err_t hs3001_i2c_read(uint8_t *p_data, uint32_t len) {
  fsp_err_t err = RM_COMMS_I2C_Read(&g_hs_comms_i2c_ctrl, p_data, len);
  if (FSP_SUCCESS == err) {
    /* Wait until transfer is done (simple busy-wait) */
    while (0U == g_hs_i2c_done) {
      __NOP();
    }
    g_hs_i2c_done = 0U;
  }

  return err;
}

/* --------------------------------------------------------------------------
 *  Public APIs
 * -------------------------------------------------------------------------- */

fsp_err_t hs3001_init(void) {
  /* Open rm_comms instance for HS3001.
   * Underlying shared I2C bus is already configured in
   * g_comms_i2c_bus0_extended_cfg. */
  return RM_COMMS_I2C_Open(&g_hs_comms_i2c_ctrl, &g_hs_comms_i2c_cfg);
}

fsp_err_t hs3001_read_raw(uint16_t *p_humidity_raw, uint16_t *p_temp_raw,
                          uint8_t *p_status) {
  if ((NULL == p_humidity_raw) || (NULL == p_temp_raw)) {
    return FSP_ERR_ASSERTION;
  }

  fsp_err_t err;
  uint8_t buf[4];
  uint8_t status_bits;

  /* ------------------------------------------------------------------
   * 1) Measurement Request (MR)
   *
   * According to the datasheet, MR is issued by sending the HS3xxx
   * I2C address with WRITE and no data bytes. rm_comms accepts a
   * zero-length write for this.
   * ------------------------------------------------------------------ */
  err = hs3001_i2c_write(NULL, 0U);
  if (FSP_SUCCESS != err) {
    return err;
  }

  /* Wait for conversion to complete (14-bit RH + 14-bit T) */
  R_BSP_SoftwareDelay(HS3001_MEASUREMENT_TIME_MS, BSP_DELAY_UNITS_MILLISECONDS);

  /* ------------------------------------------------------------------
   * 2) Data Fetch (DF)
   *
   * Read 4 bytes:
   *  - Byte0: [7:6] status bits, [5:0] humidity[13:8]
   *  - Byte1: humidity[7:0]
   *  - Byte2: temperature[15:8]
   *  - Byte3: temperature[7:0], where [1:0] are undefined
   * ------------------------------------------------------------------ */
  err = hs3001_i2c_read(buf, 4U);
  if (FSP_SUCCESS != err) {
    return err;
  }

  /* Extract status bits (two MSBs of first byte) */
  status_bits = (uint8_t)((buf[0] & 0xC0U) >> 6);

  if (NULL != p_status) {
    *p_status = status_bits;
  }

  /* 14-bit humidity: ignore status bits (mask 0x3F) */
  *p_humidity_raw =
      (uint16_t)(((uint16_t)(buf[0] & 0x3FU) << 8) | (uint16_t)buf[1]);

  /* 14-bit temperature: discard lower 2 bits as per datasheet */
  uint16_t temp_word = (uint16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
  *p_temp_raw = (uint16_t)(temp_word >> 2);

  return FSP_SUCCESS;
}

float hs3001_raw_to_humidity(uint16_t humidity_raw) {
  /* RH[%] = raw / 2^14 * 100 */
  return ((float)humidity_raw * 100.0f) / 16384.0f;
}

float hs3001_raw_to_temperature(uint16_t temp_raw) {
  /* T[°C] = raw / 2^14 * 165 - 40 */
  return ((float)temp_raw * 165.0f) / 16384.0f - 40.0f;
}

fsp_err_t hs3001_read(float *p_humidity_rh, float *p_temperature_c) {
  uint16_t humidity_raw = 0U;
  uint16_t temp_raw = 0U;
  uint8_t status = 0U;

  fsp_err_t err = hs3001_read_raw(&humidity_raw, &temp_raw, &status);
  if (FSP_SUCCESS != err) {
    return err;
  }

  /* Optional: you could check status here (0 = valid, 1 = stale).
   * For now we just convert whatever we got. */

  if (NULL != p_humidity_rh) {
    *p_humidity_rh = hs3001_raw_to_humidity(humidity_raw);
  }

  if (NULL != p_temperature_c) {
    *p_temperature_c = hs3001_raw_to_temperature(temp_raw);
  }

  return FSP_SUCCESS;
}
