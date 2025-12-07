#ifndef OB1203_HANDLER_H
#define OB1203_HANDLER_H

#include "sensor_thread.h"
#include <stdint.h>

/* OB1203 output data structure */
typedef struct st_ob1203_data
{
    float     heart_rate_bpm;   /* Estimated heart rate (BPM)        */
    float     spo2_percent;     /* Estimated SpO2 (%)                */
    uint32_t  ir_raw;           /* Raw IR LED sample                 */
    uint32_t  red_raw;          /* Raw RED LED sample                */
    uint8_t   valid;            /* 1 if the sample is valid          */
} ob1203_data_t;

/* Minimal OB1203 PPG configuration */
typedef struct st_ob1203_config
{
    /* 9-bit LED current codes (datasheet: ILEDx[8:0]) */
    uint16_t ir_led_current;    /* IR LED current code  (0..511)     */
    uint16_t red_led_current;   /* RED LED current code (0..511)     */

    /* Number of samples to average: 1..32 (0 = use default) */
    uint8_t  averaging;
} ob1203_config_t;

/* Device bring-up:
 *  - Open I2C (if not already open)
 *  - Wait for power-up
 *  - Soft reset via register
 *  - Read part ID and status for debug
 * No hardware reset pin is used. */
fsp_err_t ob1203_init(void);

/* Configure LED currents, averaging, pulse width, sample rate, FIFO, etc. */
fsp_err_t ob1203_configure(ob1203_config_t const * p_config);

/* Start / stop PPG (SpO2 / HR) measurement using MAIN_CTRL_1 register. */
fsp_err_t ob1203_start_measurement(void);
fsp_err_t ob1203_stop_measurement(void);

/* High-level read helper:
 *  - Read one IR + RED sample from FIFO
 *  - Decode 18-bit raw values
 *  - Fill ob1203_data_t (heart_rate_bpm / spo2_percent are demo values) */
fsp_err_t ob1203_read(ob1203_data_t * p_out);

/* FIFO / status helpers */
fsp_err_t ob1203_read_fifo(ob1203_data_t * p_out, uint8_t * p_samples_available);
fsp_err_t ob1203_read_status(uint8_t * p_status0, uint8_t * p_status1);

/* Utility functions:
 *  - ob1203_soft_reset: set software reset bit in control register
 *  - ob1203_read_part_id: read part ID register
 * Only register access, no hardware reset pin. */
fsp_err_t ob1203_soft_reset(void);
fsp_err_t ob1203_read_part_id(uint8_t * p_part_id);

#endif /* OB1203_HANDLER_H */
