#include "icp_handler.h"
#include "sensor_thread.h"

extern rm_comms_i2c_instance_ctrl_t g_icp_comms_i2c_ctrl;
extern const rm_comms_cfg_t g_icp_comms_i2c_cfg;
volatile uint8_t gi2c_done = 0;
icp10101_calib_t g_icp_calib;

void icp_i2c_cb(rm_comms_callback_args_t *p_args) {
    if (p_args->event == RM_COMMS_EVENT_OPERATION_COMPLETE) gi2c_done = 1;
}
fsp_err_t icp_i2c_write(const uint8_t *p_data, uint32_t len) {
    fsp_err_t err = RM_COMMS_I2C_Write(&g_icp_comms_i2c_ctrl, p_data, len);
    if (FSP_SUCCESS == err) { while (!gi2c_done) { __NOP(); } gi2c_done = 0; }
    return err;
}
fsp_err_t icp_i2c_read(uint8_t *p_data, uint32_t len) {
    fsp_err_t err = RM_COMMS_I2C_Read(&g_icp_comms_i2c_ctrl, p_data, len);
    if (FSP_SUCCESS == err) { while (!gi2c_done) { __NOP(); } gi2c_done = 0; }
    return err;
}

uint8_t icp_crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
    }
    return crc;
}

fsp_err_t icp_reset(void) {
    uint8_t cmd[2] = { (ICP10101_CMD_RESET >> 8) & 0xFF, ICP10101_CMD_RESET & 0xFF };
    fsp_err_t err = icp_i2c_write(cmd, 2);
    if (FSP_SUCCESS == err) return err;
}

fsp_err_t icp_read_id(uint16_t *pid) {
    uint8_t cmd[2] = { (ICP10101_CMD_READ_ID >> 8) & 0xFF, ICP10101_CMD_READ_ID & 0xFF };
    uint8_t buf[3];
    fsp_err_t err = icp_i2c_write(cmd, 2); if (FSP_SUCCESS != err) return err;
    err = icp_i2c_read(buf, 3); if (FSP_SUCCESS != err) return err;
    if (icp_crc8(buf, 2) != buf[2]) return FSP_ERR_CRC_ERROR;
    *pid = (buf[0] << 8) | buf[1];
    return FSP_SUCCESS;
}

fsp_err_t icp_read_otp(icp10101_calib_t *calib) {
    uint8_t cmd[5] = { (ICP10101_CMD_OTP_READ >> 8) & 0xFF, ICP10101_CMD_OTP_READ & 0xFF, 0x00,
                       ICP10101_CMD_OTP_ADDR, ICP10101_CMD_OTP_ADDR_CRC };
    uint8_t inccmd[2] = { (ICP10101_CMD_OTP_INC >> 8) & 0xFF, ICP10101_CMD_OTP_INC & 0xFF };
    uint8_t buf[3];
    fsp_err_t err = icp_i2c_write(cmd, 5); if (FSP_SUCCESS != err) return err;
    for (uint8_t i = 0; i < 4; i++) {
        err = icp_i2c_write(inccmd, 2); if (FSP_SUCCESS != err) return err;
        err = icp_i2c_read(buf, 3); if (FSP_SUCCESS != err) return err;
        if (icp_crc8(buf, 2) != buf[2]) return FSP_ERR_CRC_ERROR;
        calib->otp[i] = (int16_t)((buf[0] << 8) | buf[1]);
    }
    return FSP_SUCCESS;
}

void icp_init_calib_consts(icp10101_calib_t *calib) {
    for (int i = 0; i < 4; i++) calib->sensor_constants[i] = (float)calib->otp[i];
    calib->p_Pa_calib[0] = 45000.f; calib->p_Pa_calib[1] = 80000.f; calib->p_Pa_calib[2] = 105000.f;
    calib->LUT_lower = 3.5f * (1 << 20);
    calib->LUT_upper = 11.5f * (1 << 20);
    calib->quadr_factor = 1.0f / 16777216.0f;
    calib->offst_factor = 2048.0f;
}

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

float icp_raw_to_temp(uint16_t t_raw) {
    return -45.0f + (175.0f / 65536.0f) * (float)t_raw;
}

float icp_raw_to_press(const icp10101_calib_t *c, uint32_t p_raw, int32_t t_raw) {
    float A, B, C;
    icp_calc_ABC(c, t_raw, &A, &B, &C);
    return A + (B / (C + (float)p_raw));
}

fsp_err_t icp_read_raw(uint32_t *p_raw, uint16_t *t_raw) {
    uint8_t cmd[2] = { (ICP10101_CMD_MEAS_LN >> 8) & 0xFF, ICP10101_CMD_MEAS_LN & 0xFF };
    uint8_t readbuf[9] = {0};
    fsp_err_t err = icp_i2c_write(cmd, 2); if (FSP_SUCCESS != err) return err;
    R_BSP_SoftwareDelay(ICP10101_DELAY_MEASURE_LN_MS, BSP_DELAY_UNITS_MILLISECONDS);
    err = icp_i2c_read(readbuf, 9); if (FSP_SUCCESS != err) return err;
    if (icp_crc8(&readbuf[0], 2) != readbuf[2]) return FSP_ERR_CRC_ERROR;
    if (icp_crc8(&readbuf[3], 2) != readbuf[5]) return FSP_ERR_CRC_ERROR;
    *p_raw = ((uint32_t)readbuf[0] << 16) | ((uint32_t)readbuf[1] << 8) | readbuf[2];
    *t_raw = ((uint16_t)readbuf[6] << 8) | readbuf[7];
    return FSP_SUCCESS;
}

fsp_err_t icp10101_init(void) {
    fsp_err_t err = RM_COMMS_I2C_Open(&g_icp_comms_i2c_ctrl, &g_icp_comms_i2c_cfg);
    return err;
}
fsp_err_t icp10101_read(float *p_pa, float *t_c) {
    uint32_t p_raw = 0; uint16_t t_raw = 0;
    fsp_err_t err = icp_read_raw(&p_raw, &t_raw); if (FSP_SUCCESS != err) return err;
    if (t_c != NULL) *t_c = icp_raw_to_temp(t_raw);
    if (p_pa != NULL) *p_pa = icp_raw_to_press(&g_icp_calib, p_raw, (int32_t)t_raw);
    return FSP_SUCCESS;
}

// I2C callbacks (keep your existing ones)
void icp_comms_i2c_callback(rm_comms_callback_args_t *p_args) {
  if (p_args->event == RM_COMMS_EVENT_OPERATION_COMPLETE) {
    gi2c_done = 1;
  }
}