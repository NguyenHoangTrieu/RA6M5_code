#include "blinky_thread.h"
#include "sensor_thread.h"
#include <stdio.h>

extern bsp_leds_t g_bsp_leds;

void sensor_thread_entry(void *pvParameters) {
  FSP_PARAMETER_NOT_USED(pvParameters);

  uint8_t icp_data[8] = {0};
  uint8_t zmod_data[8] = {0};
  char uart_msg[128];
  bsp_io_level_t pin_level = BSP_IO_LEVEL_LOW;

  // Initialize (open) all instances - call only once at task start
  fsp_err_t err_icp =
      RM_COMMS_I2C_Open(&g_icp_comms_i2c_ctrl, &g_icp_comms_i2c_cfg);
  fsp_err_t err_zmod =
      RM_COMMS_I2C_Open(&g_zmod_comms_i2c_ctrl, &g_zmod_comms_i2c_cfg);
  fsp_err_t err_uart =
      RM_COMMS_UART_Open(&g_comms_uart0_ctrl, &g_comms_uart0_cfg);

  // Report UART/I2C initialization status via UART
  snprintf(uart_msg, sizeof(uart_msg),
           "Init: ICP_i2c=%s, ZMOD_i2c=%s, UART=%s\r\n",
           (err_icp == FSP_SUCCESS ? "OK" : "ERR"),
           (err_zmod == FSP_SUCCESS ? "OK" : "ERR"),
           (err_uart == FSP_SUCCESS ? "OK" : "ERR"));
  RM_COMMS_UART_Write(&g_comms_uart0_ctrl, (uint8_t *)uart_msg,
                      strlen(uart_msg));

  while (1) {
    // --- Read ICP-10101 ---
    fsp_err_t read_icp =
        RM_COMMS_I2C_Read(&g_icp_comms_i2c_ctrl, icp_data, 6u);

    // --- Read ZMOD4410 ---
    fsp_err_t read_zmod =
        RM_COMMS_I2C_Read(&g_zmod_comms_i2c_ctrl, zmod_data, 8u);
    
    snprintf(uart_msg, sizeof(uart_msg), "Read Success!");

    // --- Process data: (quick raw parsing, actual implementation may need sensor libraries to decode IAQ, pressure) ---
    uint32_t icp_pres_raw =
        (icp_data[3] << 16) | (icp_data[4] << 8) | icp_data[5];
    float icp_pressure = icp_pres_raw / (float)(1 << 20) * 25.0f + 300.0f;
    uint16_t zmod_iaq_raw = (zmod_data[0] << 8) | zmod_data[1];

    snprintf(uart_msg, sizeof(uart_msg),
             "ICP: %.2f Pa | ZMOD_IAQ: 0x%04X [%s/%s]\r\n", icp_pressure,
             zmod_iaq_raw, (read_icp == FSP_SUCCESS ? "ICP_OK" : "ICP_ERR"),
             (read_zmod == FSP_SUCCESS ? "ZMOD_OK" : "ZMOD_ERR"));

    RM_COMMS_UART_Write(&g_comms_uart0_ctrl, (uint8_t *)uart_msg,
                        strlen(uart_msg));

    // Blink the LED to show activity
    R_BSP_PinAccessEnable();
    R_BSP_PinWrite(g_bsp_leds.p_leds[0], pin_level);
    R_BSP_PinAccessDisable();
    pin_level = (pin_level == BSP_IO_LEVEL_LOW) ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW;

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// Callback required for ZMOD4410 I2C
void zmod_comms_i2c_callback(rm_comms_callback_args_t *p_args) {
  FSP_PARAMETER_NOT_USED(p_args);
  // (Leave empty or handle events if needed)
}

// Callback required for ICP-10101 I2C
void icp_comms_i2c_callback(rm_comms_callback_args_t *p_args) {
  FSP_PARAMETER_NOT_USED(p_args);
  // (Leave empty or handle events if needed)
}
