#include "hs3001_handler_reg.h"
#include "hal_data.h"

#define HS3001_I2C_ADDR (0x44U)

volatile uint8_t g_hs_i2c_done = 0U;

void hs_comms_i2c_callback(rm_comms_callback_args_t *p_args) {
  (void)p_args;
  /* Register version does not use callback, but keeps API */
  g_hs_i2c_done = 1U;
}

/* -------------------------------------------------------------------------- */
/*  Low-level I2C for HS3001 (copied from INIT_HS3001 / VALUE_HS3001 in
 * hal_entry.c) */
/* -------------------------------------------------------------------------- */

static void hs3001_hw_init(void) {
  /* Enable IIC0 module clock */
  R_MSTP->MSTPCRB_b.MSTPB9 = 0;

  /* Unlock PFS to configure I2C pins */
  R_PMISC->PWPR_b.B0WI = 0;
  R_PMISC->PWPR_b.PFSWE = 1;

  /* P400: SCL0_A, P401: SDA0_A */
  R_PFS->PORT[4].PIN[0].PmnPFS_b.PMR = 1;
  R_PFS->PORT[4].PIN[1].PmnPFS_b.PMR = 1;

  R_PFS->PORT[4].PIN[0].PmnPFS_b.PCR = 1;
  R_PFS->PORT[4].PIN[1].PmnPFS_b.PCR = 1;

  R_PFS->PORT[4].PIN[0].PmnPFS_b.PSEL = 0b00111; /* SCL0_A */
  R_PFS->PORT[4].PIN[1].PmnPFS_b.PSEL = 0b00111; /* SDA0_A */

  /* Lock PFS */
  R_PMISC->PWPR_b.PFSWE = 0;
  R_PMISC->PWPR_b.B0WI = 1;

  /* Configure I2C (same as hal_entry INIT_HS3001) */
  R_IIC0->ICCR1_b.ICE = 0;
  R_IIC0->ICCR1_b.IICRST = 1;
  R_IIC0->ICCR1_b.ICE = 1;

  R_IIC0->ICSER_b.SAR0E = 1;
  R_IIC0->SAR[0].U_b.FS = 0; /* 7-bit address */
  R_IIC0->SAR[0].L_b.SVA = HS3001_I2C_ADDR;

  /* 100 kHz @ PCLKB = 50 MHz, config copied from hal_entry */
  R_IIC0->ICMR1_b.CKS = 0b011;
  R_IIC0->ICBRL_b.BRL = 0xFE;
  R_IIC0->ICBRH_b.BRH = 0xF8;

  R_IIC0->ICFER_b.NACKE = 1;
  R_IIC0->ICFER_b.SCLE = 1;
  R_IIC0->ICCR1_b.IICRST = 0;
}

/**
 * @brief Send Measurement Request + read 4 bytes raw data from HS3001.
 */
static fsp_err_t hs3001_measure(uint16_t *p_humidity_raw, uint16_t *p_temp_raw,
                                uint8_t *p_status) {
  int value1 = 0;
  int value2 = 0;
  int value3 = 0;
  int value4 = 0;
  int trash = 0;

  /* Ensure bus is free */
  while (R_IIC0->ICCR2_b.BBSY == 1U) {
    ;
  }

  /* -------- Measurement Request (write command 0x00) -------- */
  R_IIC0->ICCR2_b.ST = 1U;
  while (R_IIC0->ICSR2_b.TDRE == 0U) {
    ;
  }

  /* Address + write */
  R_IIC0->ICDRT = (HS3001_I2C_ADDR << 1) | 0U;
  while (R_IIC0->ICSR2_b.TDRE == 0U) {
    ;
  }

  /* Send command 0x00 */
  R_IIC0->ICDRT = 0x00U;
  while (R_IIC0->ICSR2_b.TEND == 0U) {
    ;
  }

  /* STOP */
  R_IIC0->ICSR2_b.STOP = 0U;
  R_IIC0->ICCR2_b.SP = 1U;
  while (R_IIC0->ICSR2_b.STOP == 0U) {
    ;
  }
  R_IIC0->ICSR2_b.NACKF = 0U;
  R_IIC0->ICSR2_b.STOP = 0U;

  /* Wait for ADC conversion */
  R_BSP_SoftwareDelay(HS3001_MEASUREMENT_TIME_MS, BSP_DELAY_UNITS_MILLISECONDS);

  /* ------------------------- Data Fetch (4 bytes) ------------------------- */
  while (R_IIC0->ICCR2_b.BBSY == 1U) {
    ;
  }

  R_IIC0->ICCR2_b.ST = 1U;
  while (R_IIC0->ICSR2_b.TDRE == 0U) {
    ;
  }

  R_IIC0->ICDRT = (HS3001_I2C_ADDR << 1) | 1U; /* READ */
  while (R_IIC0->ICSR2_b.RDRF == 0U) {
    ;
  }

  /* If NACK then abort */
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

  /* Read dummy byte */
  trash = R_IIC0->ICDRR;
  (void)trash;

  for (int i = 1; i < 3; i++) {
    while (R_IIC0->ICSR2_b.RDRF == 0U) {
      ;
    }

    if (i == 1) {
      value1 = R_IIC0->ICDRR;
    } else {
      R_IIC0->ICMR3_b.WAIT = 1U;
      value2 = R_IIC0->ICDRR;
    }
  }

  while (R_IIC0->ICSR2_b.RDRF == 0U) {
    ;
  }
  R_IIC0->ICMR3_b.ACKBT = 1U;
  value3 = R_IIC0->ICDRR;

  while (R_IIC0->ICSR2_b.RDRF == 0U) {
    ;
  }
  R_IIC0->ICSR2_b.STOP = 0U;
  R_IIC0->ICCR2_b.SP = 1U;
  value4 = R_IIC0->ICDRR;
  R_IIC0->ICMR3_b.WAIT = 0U;

  while (R_IIC0->ICSR2_b.STOP == 0U) {
    ;
  }
  R_IIC0->ICSR2_b.NACKF = 0U;
  R_IIC0->ICSR2_b.STOP = 0U;

  /* Decode same as READ_HS3001 in hal_entry.c */
  uint16_t humidity_raw =
      (uint16_t)(((uint16_t)(value1 & 0x3F) << 8) | (uint16_t)value2);
  uint16_t temp_raw =
      (uint16_t)((((uint16_t)value3 << 8) | (uint16_t)value4) >> 2);
  uint8_t status_bits = (uint8_t)((value1 >> 6) & 0x03U);

  if (p_humidity_raw) {
    *p_humidity_raw = humidity_raw;
  }
  if (p_temp_raw) {
    *p_temp_raw = temp_raw;
  }
  if (p_status) {
    *p_status = status_bits;
  }

  return FSP_SUCCESS;
}

/* -------------------------------------------------------------------------- */
/*  Public API                                                                */
/* -------------------------------------------------------------------------- */

fsp_err_t hs3001_init(void) {
  hs3001_hw_init();
  return FSP_SUCCESS;
}

fsp_err_t hs3001_read_raw(uint16_t *p_humidity_raw, uint16_t *p_temp_raw,
                          uint8_t *p_status) {
  if ((NULL == p_humidity_raw) || (NULL == p_temp_raw)) {
    return FSP_ERR_ASSERTION;
  }

  return hs3001_measure(p_humidity_raw, p_temp_raw, p_status);
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

  if (p_humidity_rh) {
    *p_humidity_rh = hs3001_raw_to_humidity(humidity_raw);
  }

  if (p_temperature_c) {
    *p_temperature_c = hs3001_raw_to_temperature(temp_raw);
  }

  (void)status; /* if status is not used */

  return FSP_SUCCESS;
}
