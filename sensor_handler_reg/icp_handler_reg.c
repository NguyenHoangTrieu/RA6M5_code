#include "icp_handler_reg.h"
#include "hal_data.h"

#define ICP10101_I2C_ADDR (0x63U)

volatile uint8_t gi2c_done = 0U;
icp10101_calib_t g_icp_calib = {0};

/* -------------------------------------------------------------------------- */
/*  Dummy callbacks for API compatibility (not used in reg version)           */
/* -------------------------------------------------------------------------- */

void icp_i2c_cb(rm_comms_callback_args_t *p_args) {
    (void)p_args;
    gi2c_done = 1U;
}

void icp_comms_i2c_callback(rm_comms_callback_args_t *p_args) {
    (void)p_args;
    gi2c_done = 1U;
}

/* -------------------------------------------------------------------------- */
/*  CRC8 (same as HAL version)                                               */
/* -------------------------------------------------------------------------- */

uint8_t icp_crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0xFFU;

    for (uint8_t i = 0U; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0U; j < 8U; j++) {
            crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x31U) : (uint8_t)(crc << 1);
        }
    }

    return crc;
}

/* -------------------------------------------------------------------------- */
/*  Low-level I2C (copied/generalized from INIT_ICP10101, I2C_RECEIVE_MULTI, */
/*  VALUE_ICP10101 in hal_entry.c)                                            */
/* -------------------------------------------------------------------------- */

static void icp_i2c_release_bus(void) {
    /* Copied from I2C_ReleaseBus in hal_entry.c */
    R_PMISC->PWPR_b.B0WI = 0;
    R_PMISC->PWPR_b.PFSWE = 1;

    /* SCL/SDA to GPIO */
    R_PFS->PORT[4].PIN[0].PmnPFS_b.PMR = 0;
    R_PFS->PORT[4].PIN[1].PmnPFS_b.PMR = 0;

    R_PMISC->PWPR_b.PFSWE = 0;
    R_PMISC->PWPR_b.B0WI = 1;

    /* Output high */
    R_PORT4->PDR_b.PDR0 = 1;
    R_PORT4->PDR_b.PDR1 = 1;

    /* Generate 9 SCL pulses */
    R_PORT4->PDR_b.PDR1 = 1;
    for (int i = 0; i < 9; i++) {
        R_PORT4->PODR_b.PODR0 = 1;
        R_BSP_SoftwareDelay(5, BSP_DELAY_UNITS_MICROSECONDS);
        R_PORT4->PODR_b.PODR0 = 0;
        R_BSP_SoftwareDelay(5, BSP_DELAY_UNITS_MICROSECONDS);
    }

    /* START -> STOP sequence */
    R_PORT4->PODR_b.PODR1 = 0;
    R_BSP_SoftwareDelay(5, BSP_DELAY_UNITS_MICROSECONDS);
    R_PORT4->PODR_b.PODR0 = 1;
    R_PORT4->PODR_b.PODR1 = 1;
}

/* Configure IIC0 for ICP10101 (same as INIT_ICP10101 in hal_entry.c) */
static void icp_i2c_hw_init(void) {
    R_MSTP->MSTPCRB_b.MSTPB9 = 0;

    R_PMISC->PWPR_b.B0WI = 0;
    R_PMISC->PWPR_b.PFSWE = 1;

    /* P400 / P401 are SCL0_A / SDA0_A */
    R_PFS->PORT[4].PIN[0].PmnPFS_b.PMR = 1;
    R_PFS->PORT[4].PIN[1].PmnPFS_b.PMR = 1;

    R_PFS->PORT[4].PIN[0].PmnPFS_b.PCR = 1;
    R_PFS->PORT[4].PIN[1].PmnPFS_b.PCR = 1;

    R_PFS->PORT[4].PIN[0].PmnPFS_b.PSEL = 0b00111;
    R_PFS->PORT[4].PIN[1].PmnPFS_b.PSEL = 0b00111;

    R_PMISC->PWPR_b.PFSWE = 0;
    R_PMISC->PWPR_b.B0WI = 1;

    /* Configure IIC0 */
    R_IIC0->ICCR1_b.ICE = 0;
    R_IIC0->ICCR1_b.IICRST = 1;
    R_IIC0->ICCR1_b.ICE = 1;

    R_IIC0->ICSER_b.SAR0E = 1;
    R_IIC0->SAR[0].U_b.FS = 0; /* 7-bit address */
    R_IIC0->SAR[0].L_b.SVA = ICP10101_I2C_ADDR;

    /* 100 kHz @ 50 MHz, same as hal_entry */
    R_IIC0->ICMR1_b.CKS = 0b011;
    R_IIC0->ICBRL_b.BRL = 0xFE;
    R_IIC0->ICBRH_b.BRH = 0xF8;

    R_IIC0->ICFER_b.NACKE = 1;
    R_IIC0->ICFER_b.SCLE = 1;
    R_IIC0->ICCR1_b.IICRST = 0;
}

fsp_err_t icp_i2c_write(const uint8_t *p_data, uint32_t len) {
    if ((NULL == p_data) || (0U == len)) {
        return FSP_ERR_ASSERTION;
    }

    while (R_IIC0->ICCR2_b.BBSY == 1U) {
        ;
    }

    R_IIC0->ICCR2_b.ST = 1U;
    while (R_IIC0->ICSR2_b.TDRE == 0U) {
        ;
    }

    /* Address + write */
    R_IIC0->ICDRT = (ICP10101_I2C_ADDR << 1) | 0U;
    while (R_IIC0->ICSR2_b.TDRE == 0U) {
        ;
    }

    for (uint32_t i = 0; i < len; i++) {
        R_IIC0->ICDRT = p_data[i];

        while (R_IIC0->ICSR2_b.TEND == 0U) {
            ;
        }

        if (R_IIC0->ICSR2_b.NACKF != 0U) {
            /* STOP + clear NACK */
            R_IIC0->ICSR2_b.STOP = 0U;
            R_IIC0->ICCR2_b.SP = 1U;
            while (R_IIC0->ICSR2_b.STOP == 0U) {
                ;
            }
            R_IIC0->ICSR2_b.NACKF = 0U;
            R_IIC0->ICSR2_b.STOP = 0U;
            return FSP_ERR_ABORTED;
        }
    }

    R_IIC0->ICSR2_b.STOP = 0U;
    R_IIC0->ICCR2_b.SP = 1U;
    while (R_IIC0->ICSR2_b.STOP == 0U) {
        ;
    }
    R_IIC0->ICSR2_b.NACKF = 0U;
    R_IIC0->ICSR2_b.STOP = 0U;

    return FSP_SUCCESS;
}

/**
 * @brief Read len bytes from ICP10101. Supports len = 3 (OTP / ID) or len = 9 (measurement LN).
 */
fsp_err_t icp_i2c_read(uint8_t *p_data, uint32_t len) {
    if ((NULL == p_data) || ((3U != len) && (9U != len))) {
        return FSP_ERR_ASSERTION;
    }

    int trash = 0;

    while (R_IIC0->ICCR2_b.BBSY == 1U) {
        ;
    }

    R_IIC0->ICCR2_b.ST = 1U;
    while (R_IIC0->ICSR2_b.TDRE == 0U) {
        ;
    }

    R_IIC0->ICDRT = (ICP10101_I2C_ADDR << 1) | 1U;
    while (R_IIC0->ICSR2_b.RDRF == 0U) {
        ;
    }

    if (R_IIC0->ICSR2_b.NACKF != 0U) {
        R_IIC0->ICSR2_b.STOP = 0U;
        R_IIC0->ICCR2_b.SP = 1U;
        trash = R_IIC0->ICDRR;
        (void)trash;
        while (R_IIC0->ICSR2_b.STOP == 0U) {
            ;
        }
        R_IIC0->ICSR2_b.NACKF = 0U;
        R_IIC0->ICSR2_b.STOP = 0U;
        return FSP_ERR_ABORTED;
    }

    /* Discard first byte */
    trash = R_IIC0->ICDRR;
    (void)trash;

    if (len == 3U) {
        /* Pattern from I2C_RECEIVE_MULTI() in hal_entry */
        while (R_IIC0->ICSR2_b.RDRF == 0U) {
            ;
        }
        R_IIC0->ICMR3_b.WAIT = 1U;
        p_data[0] = R_IIC0->ICDRR;

        while (R_IIC0->ICSR2_b.RDRF == 0U) {
            ;
        }
        R_IIC0->ICMR3_b.ACKBT = 1U;
        p_data[1] = R_IIC0->ICDRR;

        while (R_IIC0->ICSR2_b.RDRF == 0U) {
            ;
        }
        R_IIC0->ICSR2_b.STOP = 0U;
        R_IIC0->ICCR2_b.SP = 1U;
        p_data[2] = R_IIC0->ICDRR;
        R_IIC0->ICMR3_b.WAIT = 0U;
    } else /* len == 9 */
    {
        /* Pattern from VALUE_ICP10101 in hal_entry – read 9 bytes */
        for (int i = 0; i < 7; i++) {
            while (R_IIC0->ICSR2_b.RDRF == 0U) {
                ;
            }
            if (i == 6) {
                R_IIC0->ICMR3_b.WAIT = 1U; /* before Temperature_MSB */
            }
            p_data[i] = R_IIC0->ICDRR;
        }

        while (R_IIC0->ICSR2_b.RDRF == 0U) {
            ;
        }
        R_IIC0->ICMR3_b.ACKBT = 1U;
        p_data[7] = R_IIC0->ICDRR;

        while (R_IIC0->ICSR2_b.RDRF == 0U) {
            ;
        }
        R_IIC0->ICSR2_b.STOP = 0U;
        R_IIC0->ICCR2_b.SP = 1U;
        p_data[8] = R_IIC0->ICDRR;
        R_IIC0->ICMR3_b.WAIT = 0U;
    }

    while (R_IIC0->ICSR2_b.STOP == 0U) {
        ;
    }
    R_IIC0->ICSR2_b.NACKF = 0U;
    R_IIC0->ICSR2_b.STOP = 0U;

    return FSP_SUCCESS;
}

/* -------------------------------------------------------------------------- */
/*  High-level ICP10101 API (follows icp_handler.c + math from hal_entry)    */
/* -------------------------------------------------------------------------- */

fsp_err_t icp_reset(void) {
    uint8_t cmd[2] = {
            (uint8_t)((ICP10101_CMD_RESET >> 8) & 0xFFU),
            (uint8_t)(ICP10101_CMD_RESET & 0xFFU),
    };

    fsp_err_t err = icp_i2c_write(cmd, 2U);
    if (FSP_SUCCESS != err) {
        return err;
    }

    R_BSP_SoftwareDelay(ICP10101_DELAY_RESET_MS, BSP_DELAY_UNITS_MILLISECONDS);

    return FSP_SUCCESS;
}

fsp_err_t icp_read_id(uint16_t *pid) {
    if (NULL == pid) {
        return FSP_ERR_ASSERTION;
    }

    uint8_t cmd[2] = {
            (uint8_t)((ICP10101_CMD_READ_ID >> 8) & 0xFFU),
            (uint8_t)(ICP10101_CMD_READ_ID & 0xFFU),
    };
    uint8_t buf[3] = {0};

    fsp_err_t err = icp_i2c_write(cmd, 2U);
    if (FSP_SUCCESS != err) {
        return err;
    }

    err = icp_i2c_read(buf, 3U);
    if (FSP_SUCCESS != err) {
        return err;
    }

    if (icp_crc8(buf, 2U) != buf[2]) {
        return FSP_ERR_CRC_ERROR;
    }

    *pid = (uint16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);

    return FSP_SUCCESS;
}

fsp_err_t icp_read_otp(icp10101_calib_t *calib) {
    if (NULL == calib) {
        return FSP_ERR_ASSERTION;
    }

    uint8_t cmd[5] = {
            (uint8_t)((ICP10101_CMD_OTP_READ >> 8) & 0xFFU),
            (uint8_t)(ICP10101_CMD_OTP_READ & 0xFFU),
            0x00U,
            (uint8_t)ICP10101_CMD_OTP_ADDR,
            (uint8_t)ICP10101_CMD_OTP_ADDR_CRC,
    };
    uint8_t inccmd[2] = {
            (uint8_t)((ICP10101_CMD_OTP_INC >> 8) & 0xFFU),
            (uint8_t)(ICP10101_CMD_OTP_INC & 0xFFU),
    };
    uint8_t buf[3];

    fsp_err_t err = icp_i2c_write(cmd, 5U);
    if (FSP_SUCCESS != err) {
        return err;
    }

    for (uint8_t i = 0; i < 4U; i++) {
        err = icp_i2c_write(inccmd, 2U);
        if (FSP_SUCCESS != err) {
            return err;
        }

        err = icp_i2c_read(buf, 3U);
        if (FSP_SUCCESS != err) {
            return err;
        }

        if (icp_crc8(buf, 2U) != buf[2]) {
            return FSP_ERR_CRC_ERROR;
        }

        calib->otp[i] = (int16_t)(((int16_t)buf[0] << 8) | (int16_t)buf[1]);
    }

    return FSP_SUCCESS;
}

/* Math from HAL version (equivalent to init_base / calculate_conversion_constants /
 * inv_invpres_process_data) */

void icp_init_calib_consts(icp10101_calib_t *calib) {
    for (int i = 0; i < 4; i++) {
        calib->sensor_constants[i] = (float)calib->otp[i];
    }

    calib->p_Pa_calib[0] = 45000.0f;
    calib->p_Pa_calib[1] = 80000.0f;
    calib->p_Pa_calib[2] = 105000.0f;

    calib->LUT_lower = 3.5f * (float)(1 << 20);
    calib->LUT_upper = 11.5f * (float)(1 << 20);
    calib->quadr_factor = 1.0f / 16777216.0f;
    calib->offst_factor = 2048.0f;
}

void icp_calc_ABC(const icp10101_calib_t *c, int32_t t_raw, float *A, float *B,
                                    float *C) {
    float t = (float)t_raw - 32768.0f;
    float s1 = c->LUT_lower + (c->sensor_constants[0] * t * t) * c->quadr_factor;
    float s2 = c->offst_factor * c->sensor_constants[3] +
                         (c->sensor_constants[1] * t * t) * c->quadr_factor;
    float s3 = c->LUT_upper + (c->sensor_constants[2] * t * t) * c->quadr_factor;

    float LUT[3] = {s1, s2, s3};
    float *p = (float *)c->p_Pa_calib;

    *C = (LUT[0] * LUT[1] * (p[0] - p[1]) + LUT[1] * LUT[2] * (p[1] - p[2]) +
                LUT[2] * LUT[0] * (p[2] - p[0])) /
             (LUT[2] * (p[0] - p[1]) + LUT[0] * (p[1] - p[2]) +
                LUT[1] * (p[2] - p[0]));

    *A = (p[0] * LUT[0] - p[1] * LUT[1] - (p[1] - p[0]) * (*C)) /
             (LUT[0] - LUT[1]);
    *B = (p[0] - (*A)) * (LUT[0] + (*C));
}

float icp_raw_to_temp(uint16_t t_raw) {
    /* Same as inv_invpres_process_data in hal_entry: -45 + 175/65536 * T_LSB */
    return -45.0f + (175.0f / 65536.0f) * (float)t_raw;
}

float icp_raw_to_press(const icp10101_calib_t *c, uint32_t p_raw,
                                             int32_t t_raw) {
    float A, B, C;
    icp_calc_ABC(c, t_raw, &A, &B, &C);
    return A + (B / (C + (float)p_raw));
}

fsp_err_t icp_read_raw(uint32_t *p_raw, uint16_t *t_raw) {
    if ((NULL == p_raw) || (NULL == t_raw)) {
        return FSP_ERR_ASSERTION;
    }

    uint8_t cmd[2] = {
            (uint8_t)((ICP10101_CMD_MEAS_LN >> 8) & 0xFFU),
            (uint8_t)(ICP10101_CMD_MEAS_LN & 0xFFU),
    };
    uint8_t buf[9] = {0};

    fsp_err_t err = icp_i2c_write(cmd, 2U);
    if (FSP_SUCCESS != err) {
        return err;
    }

    R_BSP_SoftwareDelay(ICP10101_DELAY_MEASURE_LN_MS,
                                            BSP_DELAY_UNITS_MILLISECONDS);

    err = icp_i2c_read(buf, 9U);
    if (FSP_SUCCESS != err) {
        return err;
    }

    /* Check CRC for 3 groups (pressure1, pressure2, temperature) */
    if (icp_crc8(&buf[0], 2U) != buf[2]) {
        return FSP_ERR_CRC_ERROR;
    }
    if (icp_crc8(&buf[3], 2U) != buf[5]) {
        return FSP_ERR_CRC_ERROR;
    }
    if (icp_crc8(&buf[6], 2U) != buf[8]) {
        return FSP_ERR_CRC_ERROR;
    }

    /* Map bytes → raw same as hal_entry: Press_raw, Temp_raw */
    uint8_t pressure_mmsb = buf[0]; /* Pressure_MMSB  */
    uint8_t pressure_mlsb = buf[1]; /* Pressure_MLSB  */
    uint8_t pressure_lmsb = buf[3]; /* Pressure_LMSB  */

    uint8_t temp_msb = buf[6];
    uint8_t temp_lsb = buf[7];

    *p_raw = ((uint32_t)pressure_mmsb << 16) | ((uint32_t)pressure_mlsb << 8) |
                     (uint32_t)pressure_lmsb;

    *t_raw = (uint16_t)(((uint16_t)temp_msb << 8) | (uint16_t)temp_lsb);

    return FSP_SUCCESS;
}

fsp_err_t icp10101_init(void) {
    icp_i2c_release_bus();
    icp_i2c_hw_init();

    /* Read OTP & initialize constants */
    fsp_err_t err = icp_read_otp(&g_icp_calib);
    if (FSP_SUCCESS != err) {
        return err;
    }

    icp_init_calib_consts(&g_icp_calib);

    return FSP_SUCCESS;
}

fsp_err_t icp10101_read(float *p_pa, float *t_c) {
    uint32_t p_raw = 0;
    uint16_t t_raw = 0;
    fsp_err_t err = icp_read_raw(&p_raw, &t_raw);

    if (FSP_SUCCESS != err) {
        return err;
    }

    if (t_c) {
        *t_c = icp_raw_to_temp(t_raw);
    }

    if (p_pa) {
        *p_pa = icp_raw_to_press(&g_icp_calib, p_raw, (int32_t)t_raw);
    }

    return FSP_SUCCESS;
}
