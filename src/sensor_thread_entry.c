#include "blinky_thread.h"
#include "icp_handler.h"
#include "sensor_thread.h"
#include "zmod_handler.h"
#include <stdio.h>
#include <string.h>


extern bsp_leds_t g_bsp_leds;
extern rm_comms_i2c_instance_ctrl_t g_icp_comms_i2c_ctrl;
extern rm_comms_i2c_instance_ctrl_t g_zmod_comms_i2c_ctrl;
volatile uint8_t i2c_flag = 0;

void i2c_comms_init(rm_comms_cfg_t *p_cfg) {
  rm_comms_i2c_bus_extended_cfg_t *p_extend =
      (rm_comms_i2c_bus_extended_cfg_t *)p_cfg->p_extend;
  i2c_master_instance_t *p_driver_instance =
      (i2c_master_instance_t *)p_extend->p_driver_instance;
  p_driver_instance->p_api->open(p_driver_instance->p_ctrl,
                                 p_driver_instance->p_cfg);
#if BSP_CFG_RTOS
  /* Create a semaphore for blocking if a semaphore is not NULL */
  if (NULL != p_extend->p_blocking_semaphore) {
#if BSP_CFG_RTOS == 1 // AzureOS
    tx_semaphore_create(p_extend->p_blocking_semaphore->p_semaphore_handle,
                        p_extend->p_blocking_semaphore->p_semaphore_name,
                        (ULONG)0);
#elif BSP_CFG_RTOS == 2 // FreeRTOS
    *(p_extend->p_blocking_semaphore->p_semaphore_handle) =
        xSemaphoreCreateCountingStatic(
            (UBaseType_t)1, (UBaseType_t)0,
            p_extend->p_blocking_semaphore->p_semaphore_memory);
#endif
  }
  /* Create a recursive mutex for bus lock if a recursive mutex is not NULL */
  if (NULL != p_extend->p_bus_recursive_mutex) {
#if BSP_CFG_RTOS == 1 // AzureOS
    tx_mutex_create(p_extend->p_bus_recursive_mutex->p_mutex_handle,
                    p_extend->p_bus_recursive_mutex->p_mutex_name, TX_INHERIT);
#elif BSP_CFG_RTOS == 2 // FreeRTOS
    *(p_extend->p_bus_recursive_mutex->p_mutex_handle) =
        xSemaphoreCreateRecursiveMutexStatic(
            p_extend->p_bus_recursive_mutex->p_mutex_memory);
#endif
  }
#endif
}

void uart_print(const char *msg) {
  RM_COMMS_UART_Write(&g_comms_uart0_ctrl, (uint8_t *)msg, strlen(msg));
  R_BSP_SoftwareDelay(5, BSP_DELAY_UNITS_MILLISECONDS);
}

void sensor_thread_entry(void *pvParameters) {
  FSP_PARAMETER_NOT_USED(pvParameters);

  char uart_msg[128];
  bsp_io_level_t pin_level = BSP_IO_LEVEL_LOW;
  fsp_err_t err;

  // Open UART
  err = RM_COMMS_UART_Open(&g_comms_uart0_ctrl, &g_comms_uart0_cfg);
  uart_print("=== ICP-10101 Sensor Verification (new API) ===\r\n");

  // Open/init I2C and ICP
  i2c_comms_init(&g_icp_comms_i2c_cfg);
  err = icp10101_init();
  snprintf(uart_msg, sizeof(uart_msg), "ICP I2C Open: %s\r\n",
           (err == FSP_SUCCESS ? "OK" : "ERR"));
  uart_print(uart_msg);

  // Soft reset
  err = icp_reset();
  uart_print(err == FSP_SUCCESS ? "✓ Soft reset OK\r\n"
                                : "✗ Soft reset error\r\n");

  // Read product ID
  uint16_t product_id = 0;
  err = icp_read_id(&product_id);
  snprintf(uart_msg, sizeof(uart_msg), "Product ID: 0x%04X (%s)\r\n",
           product_id, ((product_id & 0x3F) == 0x08) ? "OK" : "BAD");
  uart_print(uart_msg);

  // Read/verify OTP calibration
  err = icp_read_otp(&g_icp_calib);
  snprintf(uart_msg, sizeof(uart_msg), "OTP Read: %s\r\n",
           err == FSP_SUCCESS ? "OK" : "ERR");
  uart_print(uart_msg);

  icp_init_calib_consts(&g_icp_calib);
  snprintf(uart_msg, sizeof(uart_msg), "OTP c1=%d c2=%d c3=%d c4=%d\r\n",
           g_icp_calib.otp[0], g_icp_calib.otp[1], g_icp_calib.otp[2],
           g_icp_calib.otp[3]);
  uart_print(uart_msg);

  // Open/init I2C and ZMOD
  i2c_comms_init(&g_zmod_comms_i2c_cfg);
//   err = zmod4410_init();
//   snprintf(uart_msg, sizeof(uart_msg), "ZMOD I2C Open: %s\r\n",
//            (err == FSP_SUCCESS ? "OK" : "ERR"));
//   uart_print(uart_msg);

  // Measurement loop (Low Noise Mode, API mới)
  uint32_t loop_count = 0;
  while (1) {
    float pressure = 0;
    float temp = 0;
    err = icp10101_read(&pressure, &temp);
    if (err == FSP_SUCCESS) {
      snprintf(uart_msg, sizeof(uart_msg),
               "Loop %lu: Pressure: %.2f Pa, Temp: %.2f C [%s]\r\n", loop_count,
               pressure, temp, err == FSP_SUCCESS ? "OK" : "ERR");
      uart_print(uart_msg);
    }
    // vTaskDelay(pdMS_TO_TICKS(100));
    // zmod4410_data_t zdata = {0};
    // err = zmod4410_read(&zdata);
    // if (err == FSP_SUCCESS) {
    //   snprintf(uart_msg + strlen(uart_msg), sizeof(uart_msg) - strlen(uart_msg),
    //            "ZMOD: TVOC=%.1fppb eCO2=%.1fppm IAQ=%d [%s]\r\n", zdata.tvoc,
    //            zdata.eco2, zdata.iaq, err == FSP_SUCCESS ? "OK" : "ERR");
    //   uart_print(uart_msg);
    // }

    // Blink LED
    R_BSP_PinAccessEnable();
    R_BSP_PinWrite(g_bsp_leds.p_leds[0], pin_level);
    R_BSP_PinAccessDisable();
    pin_level =
        (pin_level == BSP_IO_LEVEL_LOW) ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW;

    loop_count++;
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}