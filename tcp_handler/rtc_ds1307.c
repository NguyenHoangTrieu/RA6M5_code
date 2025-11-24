#include "rtc_ds1307.h"
#include "rm_comms_api.h"
#include "ether_thread.h"
#include <string.h>

/* External I2C instance - cần khai báo trong common_data.h */
extern rm_comms_i2c_instance_ctrl_t g_comms_i2c_device_rtc_ctrl;
extern const rm_comms_cfg_t g_comms_i2c_device_rtc_cfg;

/* BCD to Decimal conversion */
uint8_t bcd_to_dec(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

/* Decimal to BCD conversion */
uint8_t dec_to_bcd(uint8_t dec) {
    return ((dec / 10) << 4) | (dec % 10);
}

/* Initialize DS1307 */
fsp_err_t ds1307_init(void) {
    fsp_err_t err;
    
    /* Open I2C communication for RTC */
    err = RM_COMMS_I2C_Open(&g_comms_i2c_device_rtc_ctrl, &g_comms_i2c_device_rtc_cfg);
    if (FSP_SUCCESS != err) {
        return err;
    }
    
    /* Enable oscillator (clear CH bit in seconds register) */
    uint8_t data[2] = {DS1307_REG_SECONDS, 0x00};
    err = RM_COMMS_I2C_Write(&g_comms_i2c_device_rtc_ctrl, data, 2);
    
    return err;
}

/* Set time on DS1307 */
fsp_err_t ds1307_set_time(rtc_time_t *time) {
    fsp_err_t err;
    uint8_t data[8];
    
    if (time == NULL) {
        return FSP_ERR_INVALID_POINTER;
    }
    
    /* Prepare data buffer */
    data[0] = DS1307_REG_SECONDS;
    data[1] = dec_to_bcd(time->seconds) & 0x7F;  // Clear CH bit
    data[2] = dec_to_bcd(time->minutes);
    data[3] = dec_to_bcd(time->hours) & 0x3F;    // 24-hour mode
    data[4] = dec_to_bcd(time->day);
    data[5] = dec_to_bcd(time->date);
    data[6] = dec_to_bcd(time->month);
    data[7] = dec_to_bcd(time->year);
    
    /* Write all registers at once */
    err = RM_COMMS_I2C_Write(&g_comms_i2c_device_rtc_ctrl, data, 8);
    
    return err;
}

/* Get time from DS1307 */
fsp_err_t ds1307_get_time(rtc_time_t *time) {
    fsp_err_t err;
    uint8_t reg_addr = DS1307_REG_SECONDS;
    uint8_t data[7];
    
    if (time == NULL) {
        return FSP_ERR_INVALID_POINTER;
    }
    
    /* Write register address */
    err = RM_COMMS_I2C_Write(&g_comms_i2c_device_rtc_ctrl, &reg_addr, 1);
    if (FSP_SUCCESS != err) {
        return err;
    }
    
    /* Read 7 bytes (seconds to year) */
    err = RM_COMMS_I2C_Read(&g_comms_i2c_device_rtc_ctrl, data, 7);
    if (FSP_SUCCESS != err) {
        return err;
    }
    
    /* Convert BCD to decimal */
    time->seconds = bcd_to_dec(data[0] & 0x7F);
    time->minutes = bcd_to_dec(data[1] & 0x7F);
    time->hours   = bcd_to_dec(data[2] & 0x3F);
    time->day     = bcd_to_dec(data[3] & 0x07);
    time->date    = bcd_to_dec(data[4] & 0x3F);
    time->month   = bcd_to_dec(data[5] & 0x1F);
    time->year    = bcd_to_dec(data[6]);
    
    return FSP_SUCCESS;
}

void comms_i2c_callback(rm_comms_callback_args_t * p_args)
{
    if (NULL != p_args)
    {
        /* Handle I2C events if needed */
    }
}