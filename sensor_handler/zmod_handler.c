#include "zmod_handler.h"
#include <string.h>
#include <stdio.h>

// Use P307 as sensor enable pin
#define ZMOD_EN_PIN BSP_IO_PORT_03_PIN_07

extern rm_comms_i2c_instance_ctrl_t g_zmod_comms_i2c_ctrl;
volatile uint8_t gi2c_done_zmod = 0;

// Enable/disable ZMOD4410 via control pin (EN)
void zmod4410_set_enable(uint8_t state)
{
    R_BSP_PinAccessEnable();
    R_BSP_PinWrite(ZMOD_EN_PIN, state ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW);
    R_BSP_PinAccessDisable();
}

fsp_err_t zmod4410_init(void)
{
    // Open I2C and power on
    static uint8_t inited = 0;
    fsp_err_t err = FSP_SUCCESS;
    if(!inited)
    {
        err = RM_COMMS_I2C_Open(&g_zmod_comms_i2c_ctrl, &g_zmod_comms_i2c_cfg);
        inited = (err == FSP_SUCCESS);
    }
    // Enable sensor by setting EN high
    zmod4410_set_enable(1);
    R_BSP_SoftwareDelay(20, BSP_DELAY_UNITS_MILLISECONDS); // Wait boot
    return err;
}

fsp_err_t zmod4410_read_raw(uint8_t *raw, uint8_t len)
{
    // Each vendor BSP project will have flags and callback, use one global flag here for illustration
    extern volatile uint8_t gi2c_done;
    fsp_err_t err;
    uint8_t cmd = ZMOD4410_REG_DATA; // Data register address

    // Set internal pointer to base address
    err = RM_COMMS_I2C_Write(&g_zmod_comms_i2c_ctrl, &cmd, 1);
    if (err != FSP_SUCCESS) return err;
    while (!gi2c_done_zmod) { __NOP(); }
    gi2c_done_zmod = 0;

    // Read output data bytes
    err = RM_COMMS_I2C_Read(&g_zmod_comms_i2c_ctrl, raw, len);
    while (!gi2c_done_zmod) { __NOP(); }
    gi2c_done_zmod = 0;
    return err;
}
// IAQ level based on TVOC in mg/m3 (from datasheet Figure 8/Section 4.4)
int zmod4410_calc_iaq_level(float tvoc_mg_m3) {
    if (tvoc_mg_m3 <= 0.3f) return 1;        // Very Good
    else if (tvoc_mg_m3 <= 1.0f) return 2;   // Good
    else if (tvoc_mg_m3 <= 3.0f) return 3;   // Medium
    else if (tvoc_mg_m3 <= 10.0f) return 4;  // Poor
    else return 5;                           // Bad
}

/**
 * Main ZMOD4410 reading function: fetches IAQ/TVOC/eCO2
 * Note: Conversion from raw to IAQ/TVOC/eCO2 normally requires Renesas algorithm library,
 * here is a basic estimation (for demo) – check the real algorithm for production!
 */
fsp_err_t zmod4410_read(zmod4410_data_t *out)
{
    uint8_t raw[10]={0};
    fsp_err_t err = zmod4410_read_raw(raw, 6);
    if (err != FSP_SUCCESS) return err;

    // Use raw[0/1] as RMox (big endian)
    out->rmox = (raw[0] << 8) | raw[1];

    // Estimate TVOC (ppb), then convert to mg/m3
    // Dummy: TVOC_ppb = rmox * 2; (replace factor by calibration if available!)
    float tvoc_ppb = (float)out->rmox * 2.0f;
    float tvoc_mg_m3 = (tvoc_ppb / 1000.0f) * 2.0f; // 1ppm TVOC = 2mg/m3 (datasheet)

    // Estimate eCO2 based on TVOC (datasheet Figure 8/Section 4.4)
    float eco2 = 400.0f + (tvoc_ppb/1000.0f)*1600.0f;
    if (eco2 > 5000.0f) eco2 = 5000.0f;

    int iaq_level = zmod4410_calc_iaq_level(tvoc_mg_m3);

    out->tvoc = tvoc_ppb;
    out->eco2 = eco2;
    out->iaq = iaq_level;
    // Optional: print/read code can convert out->iaq to string IAQ_UBA rating if muốn

    return FSP_SUCCESS;
}

void zmod_comms_i2c_callback(rm_comms_callback_args_t *p_args) {
  if (p_args->event == RM_COMMS_EVENT_OPERATION_COMPLETE) {
    gi2c_done_zmod = 1;
  }
}
