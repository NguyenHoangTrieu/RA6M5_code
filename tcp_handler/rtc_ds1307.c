#include "rtc_ds1307.h"
#include "rm_comms_api.h"
#include "ether_thread.h"
#include <string.h>

#if (REAL_RTC == 0)
/* FreeRTOS software timer cho soft RTC */
#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"
#endif

/* External I2C instance - khai báo trong common_data.h (chỉ dùng khi REAL_RTC=1) */
extern rm_comms_i2c_instance_ctrl_t g_comms_i2c_device_rtc_ctrl;
extern const rm_comms_cfg_t         g_comms_i2c_device_rtc_cfg;

/* BCD to Decimal conversion */
uint8_t bcd_to_dec(uint8_t bcd)
{
    return ((bcd >> 4) * 10U) + (bcd & 0x0FU);
}

/* Decimal to BCD conversion */
uint8_t dec_to_bcd(uint8_t dec)
{
    return (uint8_t)(((dec / 10U) << 4) | (dec % 10U));
}

#if (REAL_RTC == 0)
/* =========================
 *      SOFT RTC (FAKE)
 * ========================= */

/* Biến giữ thời gian hiện tại trong RAM */
static rtc_time_t    g_soft_rtc_time = {0};
static TimerHandle_t g_rtc_timer_handle = NULL;

/* Kiểm tra năm nhuận (giả sử năm 20xx) */
static bool soft_rtc_is_leap(uint8_t year)
{
    /* 2000–2099: chia hết cho 4 là leap year */
    uint16_t y = (uint16_t)(2000U + year);
    return ((y % 4U) == 0U);
}

static uint8_t soft_rtc_days_in_month(uint8_t month, uint8_t year)
{
    switch (month)
    {
        case 1:  return 31;
        case 2:  return soft_rtc_is_leap(year) ? 29 : 28;
        case 3:  return 31;
        case 4:  return 30;
        case 5:  return 31;
        case 6:  return 30;
        case 7:  return 31;
        case 8:  return 31;
        case 9:  return 30;
        case 10: return 31;
        case 11: return 30;
        case 12: return 31;
        default: return 31;
    }
}

/* Tăng thời gian thêm 1 giây */
static void soft_rtc_increment_1s(rtc_time_t * t)
{
    if (NULL == t)
    {
        return;
    }

    t->seconds++;
    if (t->seconds >= 60U)
    {
        t->seconds = 0U;
        t->minutes++;
        if (t->minutes >= 60U)
        {
            t->minutes = 0U;
            t->hours++;
            if (t->hours >= 24U)
            {
                t->hours = 0U;

                /* Day-of-week: 1..7 */
                if (t->day < 1U || t->day > 7U)
                {
                    t->day = 1U;
                }
                else
                {
                    t->day++;
                    if (t->day > 7U)
                    {
                        t->day = 1U;
                    }
                }

                /* Ngày trong tháng */
                uint8_t dim = soft_rtc_days_in_month(t->month, t->year);
                t->date++;
                if (t->date == 0U || t->date > dim)
                {
                    t->date = 1U;
                    t->month++;
                    if (t->month == 0U || t->month > 12U)
                    {
                        t->month = 1U;
                        t->year++;
                        if (t->year > 99U)
                        {
                            t->year = 0U;
                        }
                    }
                }
            }
        }
    }
}

static void rtc_soft_timer_callback(TimerHandle_t xTimer)
{
    FSP_PARAMETER_NOT_USED(xTimer);

    taskENTER_CRITICAL();
    soft_rtc_increment_1s(&g_soft_rtc_time);
    taskEXIT_CRITICAL();
}

#endif /* REAL_RTC == 0 */


/* Initialize DS1307 (hoặc soft RTC nếu REAL_RTC==0) */
fsp_err_t ds1307_init(void)
{
#if (REAL_RTC == 1)

    fsp_err_t err;

    /* Open I2C communication for RTC */
    err = RM_COMMS_I2C_Open(&g_comms_i2c_device_rtc_ctrl, &g_comms_i2c_device_rtc_cfg);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    /* Enable oscillator (clear CH bit in seconds register) */
    uint8_t data[2] = { DS1307_REG_SECONDS, 0x00 };
    err = RM_COMMS_I2C_Write(&g_comms_i2c_device_rtc_ctrl, data, 2);

    return err;

#else /* REAL_RTC == 0 → dùng soft RTC */

    /* Reset thời gian về 0 (hoặc giữ nguyên nếu muốn) */
    memset(&g_soft_rtc_time, 0, sizeof(g_soft_rtc_time));

    if (NULL == g_rtc_timer_handle)
    {
        g_rtc_timer_handle = xTimerCreate("SoftRTC_1s",
                                          pdMS_TO_TICKS(1000),
                                          pdTRUE,     /* auto-reload */
                                          NULL,
                                          rtc_soft_timer_callback);
    }

    if (NULL == g_rtc_timer_handle)
    {
        return FSP_ERR_ASSERTION;  /* không tạo được timer */
    }

    /* Start timer (không block) */
    if (xTimerStart(g_rtc_timer_handle, 0) != pdPASS)
    {
        return FSP_ERR_ASSERTION;
    }

    return FSP_SUCCESS;

#endif
}

/* Set time on DS1307 / soft RTC */
fsp_err_t ds1307_set_time(rtc_time_t * time)
{
    if (NULL == time)
    {
        return FSP_ERR_INVALID_POINTER;
    }

#if (REAL_RTC == 1)

    fsp_err_t err;
    uint8_t   data[8];

    /* Prepare data buffer */
    data[0] = DS1307_REG_SECONDS;
    data[1] = dec_to_bcd(time->seconds) & 0x7FU;  /* Clear CH bit */
    data[2] = dec_to_bcd(time->minutes);
    data[3] = dec_to_bcd(time->hours) & 0x3FU;    /* 24h mode */
    data[4] = dec_to_bcd(time->day);
    data[5] = dec_to_bcd(time->date);
    data[6] = dec_to_bcd(time->month);
    data[7] = dec_to_bcd(time->year);

    /* Write all registers at once */
    err = RM_COMMS_I2C_Write(&g_comms_i2c_device_rtc_ctrl, data, 8);

    return err;

#else /* REAL_RTC == 0 */

    /* Chỉ ghi vào biến soft RTC, timer 1s sẽ tự tăng */
    taskENTER_CRITICAL();
    g_soft_rtc_time = *time;
    taskEXIT_CRITICAL();

    return FSP_SUCCESS;

#endif
}

/* Get time from DS1307 / soft RTC */
fsp_err_t ds1307_get_time(rtc_time_t * time)
{
    if (NULL == time)
    {
        return FSP_ERR_INVALID_POINTER;
    }

#if (REAL_RTC == 1)

    fsp_err_t err;
    uint8_t   reg_addr = DS1307_REG_SECONDS;
    uint8_t   data[7];

    /* Write register address */
    err = RM_COMMS_I2C_Write(&g_comms_i2c_device_rtc_ctrl, &reg_addr, 1);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    /* Read 7 bytes (seconds to year) */
    err = RM_COMMS_I2C_Read(&g_comms_i2c_device_rtc_ctrl, data, 7);
    if (FSP_SUCCESS != err)
    {
        return err;
    }

    /* Convert BCD to decimal */
    time->seconds = bcd_to_dec(data[0] & 0x7FU);
    time->minutes = bcd_to_dec(data[1] & 0x7FU);
    time->hours   = bcd_to_dec(data[2] & 0x3FU);
    time->day     = bcd_to_dec(data[3] & 0x07U);
    time->date    = bcd_to_dec(data[4] & 0x3FU);
    time->month   = bcd_to_dec(data[5] & 0x1FU);
    time->year    = bcd_to_dec(data[6]);

    return FSP_SUCCESS;

#else /* REAL_RTC == 0 */

    /* Đọc snapshot thời gian từ soft RTC */
    taskENTER_CRITICAL();
    *time = g_soft_rtc_time;
    taskEXIT_CRITICAL();

    return FSP_SUCCESS;

#endif
}

/* Callback I2C cho rm_comms – giữ nguyên cho bus RTC / các chỗ khác dùng */
void comms_i2c_callback(rm_comms_callback_args_t * p_args)
{
    if (NULL != p_args)
    {
        /* Handle I2C events if needed */
    }
}
