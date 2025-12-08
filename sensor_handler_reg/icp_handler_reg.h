#ifndef ICP_HANDLER_REG_H
#define ICP_HANDLER_REG_H

#include "sensor_thread.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ICP10101_CMD_RESET 0x805D
#define ICP10101_CMD_READ_ID 0xEFC8
#define ICP10101_CMD_MEAS_LN 0x5059
#define ICP10101_CMD_OTP_READ 0xC595
#define ICP10101_CMD_OTP_ADDR 0x0066
#define ICP10101_CMD_OTP_ADDR_CRC 0x9C
#define ICP10101_CMD_OTP_INC 0xC7F7

#define ICP10101_DELAY_RESET_MS 2
#define ICP10101_DELAY_MEASURE_LN_MS 25

#define FSP_ERR_CRC_ERROR (0x5555)

typedef struct {
    int16_t otp[4];
    float sensor_constants[4];
    float p_Pa_calib[3];
    float LUT_lower;
    float LUT_upper;
    float quadr_factor;
    float offst_factor;
} icp10101_calib_t;

extern volatile uint8_t gi2c_done;
extern icp10101_calib_t g_icp_calib;

/* I2C API - keep function names consistent with HAL version */
void icp_i2c_cb(rm_comms_callback_args_t *p_args);
fsp_err_t icp_i2c_write(const uint8_t *p_data, uint32_t len);
fsp_err_t icp_i2c_read(uint8_t *p_data, uint32_t len);
uint8_t icp_crc8(const uint8_t *data, uint8_t len);

/* ICP high-level API */
fsp_err_t icp_reset(void);
fsp_err_t icp_read_id(uint16_t *pid);
fsp_err_t icp_read_otp(icp10101_calib_t *calib);
void icp_init_calib_consts(icp10101_calib_t *calib);
void icp_calc_ABC(const icp10101_calib_t *c, int32_t t_raw, float *A, float *B,
                                    float *C);
float icp_raw_to_temp(uint16_t t_raw);
float icp_raw_to_press(const icp10101_calib_t *c, uint32_t p_raw,
                                             int32_t t_raw);
fsp_err_t icp_read_raw(uint32_t *p_raw, uint16_t *t_raw);
fsp_err_t icp10101_init(void);
fsp_err_t icp10101_read(float *p_pa, float *t_c);

/* rm_comms style callback - maintain API compatibility with HAL version */
void icp_comms_i2c_callback(rm_comms_callback_args_t *p_args);

#ifdef __cplusplus
}
#endif

#endif /* ICP_HANDLER_REG_H */
