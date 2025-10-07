#include "icp_read.h"
#include "hal_data.h"
#include <string.h>

/* CONFIGURATION: FSP driver externs (auto-generated if you use FSP configurator) */
extern rm_comms_i2c_instance_ctrl_t g_icp_comms_i2c_ctrl;
extern const rm_comms_cfg_t g_icp_comms_i2c_cfg;
extern rm_comms_uart_instance_ctrl_t g_comms_uart0_ctrl;
extern const rm_comms_cfg_t g_comms_uart0_cfg;

/* Global variables */
volatile uint8_t gi2c_done = 0;
icp10101_calib_t g_icp_calib;

/* I2C Helpers */
void icp_i2c_cb(rm_comms_callback_args_t *p_args) {
    if (p_args->event == RM_COMMS_EVENT_OPERATION_COMPLETE) {
        gi2c_done = 1;
    }
}
fsp_err_t icp_i2c_write(const uint8_t *p_data, uint32_t len) {
    fsp_err_t err = RM_COMMS_I2C_Write(&g_icp_comms_i2c_ctrl, p_data, len);
    if (FSP_SUCCESS == err) {
        while (!gi2c_done) { __NOP(); }
        gi2c_done = 0;
    }
    return err;
}
fsp_err_t icp_i2c_read(uint8_t *p_data, uint32_t len) {
    fsp_err_t err = RM_COMMS_I2C_Read(&g_icp_comms_i2c_ctrl, p_data, len);
    if (FSP_SUCCESS == err) {
        while (!gi2c_done) { __NOP(); }
        gi2c_done = 0;
    }
    return err;
}

/* CRC-8 X^8+X^5+X^4+1 (Poly 0x31, Init 0xFF) */
uint8_t icp_crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
    }
    return crc;
}

/* ICP-10101 Functions */
/* 1. Soft reset sensor */
fsp_err_t icp_reset(void) {
    uint8_t cmd[2] = { (ICP10101_CMD_RESET >> 8) & 0xFF, ICP10101_CMD_RESET & 0xFF };
    fsp_err_t err = icp_i2c_write(cmd, 2);
    if (FSP_SUCCESS == err)
        R_BSP_SoftwareDelay(ICP10101_DELAY_RESET_MS, BSP_DELAY_UNITS_MILLISECONDS);
    return err;
}

/* 2. Read product ID (optional, for verification) */
fsp_err_t icp_read_id(uint16_t *pid) {
    uint8_t cmd[2] = { (ICP10101_CMD_READ_ID >> 8) & 0xFF, ICP10101_CMD_READ_ID & 0xFF };
    uint8_t buf[3];
    fsp_err_t err = icp_i2c_write(cmd, 2); if (FSP_SUCCESS != err) return err;
    err = icp_i2c_read(buf, 3); if (FSP_SUCCESS != err) return err;
    if (icp_crc8(buf, 2) != buf[2]) return FSP_ERR_CRC_ERROR;
    *pid = (buf[0] << 8) | buf[1];
    return FSP_SUCCESS;
}

/* 3. Read OTP calibration (c1,c2,c3,c4) */
fsp_err_t icp_read_otp(icp10101_calib_t *calib) {
    uint8_t cmd[5] = {
        (ICP10101_CMD_OTP_READ >> 8) & 0xFF, ICP10101_CMD_OTP_READ & 0xFF,
        0x00, ICP10101_CMD_OTP_ADDR, ICP10101_CMD_OTP_ADDR_CRC
    };
    uint8_t inccmd[2] = { (ICP10101_CMD_OTP_INC >> 8) & 0xFF, ICP10101_CMD_OTP_INC & 0xFF };
    uint8_t buf[3];
    fsp_err_t err = icp_i2c_write(cmd, 5); if (FSP_SUCCESS != err) return err;
    R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MILLISECONDS);
    for (uint8_t i = 0; i < 4; i++) {
        err = icp_i2c_write(inccmd, 2); if (FSP_SUCCESS != err) return err;
        err = icp_i2c_read(buf, 3); if (FSP_SUCCESS != err) return err;
        if (icp_crc8(buf, 2) != buf[2]) return FSP_ERR_CRC_ERROR;
        calib->otp[i] = (int16_t)((buf[0] << 8) | buf[1]);
    }
    return FSP_SUCCESS;
}

/* 4. Init calibration factors in C float */
void icp_init_calib_consts(icp10101_calib_t *calib) {
    for (int i = 0; i < 4; i++) calib->sensor_constants[i] = (float)calib->otp[i];
    calib->p_Pa_calib[0] = 45000.f; calib->p_Pa_calib[1] = 80000.f; calib->p_Pa_calib[2] = 105000.f;
    calib->LUT_lower = 3.5f * (1 << 20);
    calib->LUT_upper = 11.5f * (1 << 20);
    calib->quadr_factor = 1.0f / 16777216.0f;
    calib->offst_factor = 2048.0f;
}

/* 5. Helper Calculate conversion constants A,B,C for pressure formula */
void icp_calc_ABC(const icp10101_calib_t *c, int32_t t_raw, float *A, float *B, float *C) {
    float t = (float)t_raw - 32768.0f;
    float s1 = c->LUT_lower + (c->sensor_constants[0] * t * t) * c->quadr_factor;
    float s2 = c->offst_factor * c->sensor_constants[3] + (c->sensor_constants[1] * t * t) * c->quadr_factor;
    float s3 = c->LUT_upper + (c->sensor_constants[2] * t * t) * c->quadr_factor;
    float *p = c->p_Pa_calib, LUT[3] = {s1, s2, s3};
    *C = (LUT[0] * LUT[1] * (p[0] - p[1]) + LUT[1] * LUT[2] * (p[1] - p[2]) + LUT[2] * LUT[0] * (p[2] - p[0])) /
         (LUT[2] * (p[0] - p[1]) + LUT[0] * (p[1] - p[2]) + LUT[1] * (p[2] - p[0]));
    *A = (p[0] * LUT[0] - p[1] * LUT[1] - (p[1] - p[0]) * (*C)) / (LUT[0] - LUT[1]);
    *B = (p[0] - (*A)) * (LUT[0] + (*C));
}

/* 6. Convert raw t_raw to temp C (datasheet 5.9) */
float icp_raw_to_temp(uint16_t t_raw) {
    return -45.0f + (175.0f / 65536.0f) * (float)t_raw;
}

/* 7. Convert raw and OTP-calib to Pa (see sample code 5.9, 5.11) */
float icp_raw_to_press(const icp10101_calib_t *c, uint32_t p_raw, int32_t t_raw) {
    float A, B, C;
    icp_calc_ABC(c, t_raw, &A, &B, &C);
    return A + (B / (C + (float)p_raw));
}

/* 8. Trigger a measurement (low noise) and read out (pressure first) */
fsp_err_t icp_read_raw(uint32_t *p_raw, uint16_t *t_raw) {
    uint8_t cmd[2] = { (ICP10101_CMD_MEAS_LN >> 8) & 0xFF, ICP10101_CMD_MEAS_LN & 0xFF };
    uint8_t readbuf[9]; // 3 Press+CRC, 3 Temp+CRC, 2 bytes not used
    fsp_err_t err = icp_i2c_write(cmd, 2); if (FSP_SUCCESS != err) return err;
    R_BSP_SoftwareDelay(ICP10101_DELAY_MEASURE_LN_MS, BSP_DELAY_UNITS_MILLISECONDS);
    err = icp_i2c_read(readbuf, 9); if (FSP_SUCCESS != err) return err;
    // Pressure [0:2] + CRC, Temp [3:5] + CRC (see datasheet comms section)
    if (icp_crc8(&readbuf[0], 2) != readbuf[2]) return FSP_ERR_CRC_ERROR;
    if (icp_crc8(&readbuf[3], 2) != readbuf[5]) return FSP_ERR_CRC_ERROR;
    // Get 24-bit p_raw, 16-bit t_raw
    *p_raw = ((uint32_t)readbuf[0] << 16) | ((uint32_t)readbuf[1] << 8) | readbuf[2];
    *t_raw = ((uint16_t)readbuf[6] << 8) | readbuf[7];
    return FSP_SUCCESS;
}

/* Task and ICP driver initialization */
fsp_err_t icp10101_init(void) {
    fsp_err_t err = RM_COMMS_I2C_Open(&g_icp_comms_i2c_ctrl, &g_icp_comms_i2c_cfg);
    if (FSP_SUCCESS != err && FSP_ERR_ALREADY_OPEN != err) return err;
    R_BSP_SoftwareDelay(10, BSP_DELAY_UNITS_MILLISECONDS);
    err = icp_reset(); if (FSP_SUCCESS != err) return err;
    uint16_t id; err = icp_read_id(&id); if (FSP_SUCCESS != err) return err;
    if ((id & 0x3F) != 0x08) return FSP_ERR_NOT_FOUND; // bits 5:0 must be 0x08 (datasheet table 15)
    err = icp_read_otp(&g_icp_calib); if (FSP_SUCCESS != err) return err;
    icp_init_calib_consts(&g_icp_calib);
    return FSP_SUCCESS;
}
fsp_err_t icp10101_read(float *p_pa, float *t_c) {
    uint32_t p_raw = 0; uint16_t t_raw = 0;
    fsp_err_t err = icp_read_raw(&p_raw, &t_raw); if (FSP_SUCCESS != err) return err;
    if (t_c != NULL) *t_c = icp_raw_to_temp(t_raw);
    if (p_pa != NULL) *p_pa = icp_raw_to_press(&g_icp_calib, p_raw, (int32_t)t_raw);
    return FSP_SUCCESS;
}

