#ifndef RTC_DS1307_H_
#define RTC_DS1307_H_

#include "hal_data.h"
#include "common_data.h"
#include <stdint.h>
#include <stdbool.h>

/* DS1307 I2C Address */
#define DS1307_I2C_ADDR         0x68

/* DS1307 Register Addresses */
#define DS1307_REG_SECONDS      0x00
#define DS1307_REG_MINUTES      0x01
#define DS1307_REG_HOURS        0x02
#define DS1307_REG_DAY          0x03
#define DS1307_REG_DATE         0x04
#define DS1307_REG_MONTH        0x05
#define DS1307_REG_YEAR         0x06
#define DS1307_REG_CONTROL      0x07

/* RTC Time Structure */
typedef struct {
    uint8_t seconds;    // 0-59
    uint8_t minutes;    // 0-59
    uint8_t hours;      // 0-23
    uint8_t day;        // 1-7 (day of week)
    uint8_t date;       // 1-31
    uint8_t month;      // 1-12
    uint8_t year;       // 0-99 (last 2 digits)
} rtc_time_t;

/* Function Prototypes */
fsp_err_t ds1307_init(void);
fsp_err_t ds1307_set_time(rtc_time_t *time);
fsp_err_t ds1307_get_time(rtc_time_t *time);
uint8_t bcd_to_dec(uint8_t bcd);
uint8_t dec_to_bcd(uint8_t dec);

#endif /* RTC_DS1307_H_ */
