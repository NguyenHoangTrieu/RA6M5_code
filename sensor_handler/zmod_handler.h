#ifndef ZMOD_HANDLER_H
#define ZMOD_HANDLER_H

#include "sensor_thread.h"
#include <stdint.h>

#define ZMOD4410_I2C_ADDR        (0x32) // 7-bit I2C address (0x64/2)
#define ZMOD4410_REG_STATUS      (0x00) // Example: status register for ready
#define ZMOD4410_REG_DATA        (0x06) // Example: data result base addr

typedef struct {
    uint16_t rmox;
    float tvoc;   // Calculated TVOC (ppb)
    float eco2;   // Calculated eCO2 (ppm)
    uint8_t iaq;  // IAQ rating
} zmod4410_data_t;

void zmod4410_set_enable(uint8_t state); // Control enable pin
fsp_err_t zmod4410_init(void);
fsp_err_t zmod4410_read_raw(uint8_t *raw, uint8_t len);
fsp_err_t zmod4410_read(zmod4410_data_t *out); // Main read and convert

#endif
