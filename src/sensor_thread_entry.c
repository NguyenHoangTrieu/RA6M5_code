#include "blinky_thread.h"
#include "hs3001_handler_reg.h"
#include "icp_handler_reg.h"

#include "sensor_data_queue.h"
#include "sensor_thread.h"
#include <stdio.h>
#include <string.h>

extern bsp_leds_t g_bsp_leds;

volatile uint8_t i2c_flag = 0;

// void i2c_comms_init(rm_comms_cfg_t *p_cfg) {
//   rm_comms_i2c_bus_extended_cfg_t *p_extend =
//       (rm_comms_i2c_bus_extended_cfg_t *)p_cfg->p_extend;
//   i2c_master_instance_t *p_driver_instance =
//       (i2c_master_instance_t *)p_extend->p_driver_instance;
//   p_driver_instance->p_api->open(p_driver_instance->p_ctrl,
//                                  p_driver_instance->p_cfg);
// #if BSP_CFG_RTOS
//   /* Create a semaphore for blocking if a semaphore is not NULL */
//   if (NULL != p_extend->p_blocking_semaphore) {
// #if BSP_CFG_RTOS == 1 // AzureOS
//     tx_semaphore_create(p_extend->p_blocking_semaphore->p_semaphore_handle,
//                         p_extend->p_blocking_semaphore->p_semaphore_name,
//                         (ULONG)0);
// #elif BSP_CFG_RTOS == 2 // FreeRTOS
//     *(p_extend->p_blocking_semaphore->p_semaphore_handle) =
//         xSemaphoreCreateCountingStatic(
//             (UBaseType_t)1, (UBaseType_t)0,
//             p_extend->p_blocking_semaphore->p_semaphore_memory);
// #endif
//   }
//   /* Create a recursive mutex for bus lock if a recursive mutex is not NULL */
//   if (NULL != p_extend->p_bus_recursive_mutex) {
// #if BSP_CFG_RTOS == 1 // AzureOS
//     tx_mutex_create(p_extend->p_bus_recursive_mutex->p_mutex_handle,
//                     p_extend->p_bus_recursive_mutex->p_mutex_name, TX_INHERIT);
// #elif BSP_CFG_RTOS == 2 // FreeRTOS
//     *(p_extend->p_bus_recursive_mutex->p_mutex_handle) =
//         xSemaphoreCreateRecursiveMutexStatic(
//             p_extend->p_bus_recursive_mutex->p_mutex_memory);
// #endif
//   }
// #endif
// }

void uart_print(const char *msg) {
  RM_COMMS_UART_Write(&g_comms_uart0_ctrl, (uint8_t *)msg, strlen(msg));
  R_BSP_SoftwareDelay(5, BSP_DELAY_UNITS_MILLISECONDS);
}

void sensor_thread_entry(void *pvParameters) {
  FSP_PARAMETER_NOT_USED(pvParameters);

  char uart_msg[128];
  bsp_io_level_t pin_level = BSP_IO_LEVEL_LOW;
  fsp_err_t err;
  sensor_data_t sensor_data;

  /* Create sensor data queue */
  sensor_queue_create();

  /* Open UART (giữ nguyên HAL) */
  err = RM_COMMS_UART_Open(&g_comms_uart0_ctrl, &g_comms_uart0_cfg);
  FSP_PARAMETER_NOT_USED(err);

  uart_print("=== ICP-10101 Sensor Verification (REGISTER driver) ===\r\n");

  /* Debug heap */
  void *dummy = pvPortMalloc(1);
  vPortFree(dummy);
  size_t free_heap = xPortGetFreeHeapSize();
  snprintf(uart_msg, sizeof(uart_msg), "[DEBUG] Free heap: %u bytes\r\n",
           (unsigned int)free_heap);
  uart_print(uart_msg);

  /* --- Chỉ dùng driver thanh ghi cho ICP10101 & HS3001 --- */

  /* ICP init (register driver) */
  err = icp10101_init();
  snprintf(uart_msg, sizeof(uart_msg), "ICP I2C Open: %s\r\n",
           (err == FSP_SUCCESS ? "OK" : "ERR"));
  uart_print(uart_msg);

  /* Soft reset */
  err = icp_reset();
  if (FSP_SUCCESS == err) {
    uart_print("Soft reset OK\r\n");
  } else {
    uart_print("Soft reset error\r\n");
  }

  /* Read product ID */
  uint16_t product_id = 0;
  err = icp_read_id(&product_id);
  snprintf(uart_msg, sizeof(uart_msg), "Product ID: 0x%04X (%s)\r\n",
           product_id, ((product_id & 0x3F) == 0x08) ? "OK" : "BAD");
  uart_print(uart_msg);

  /* Read/verify OTP calibration */
  err = icp_read_otp(&g_icp_calib);
  snprintf(uart_msg, sizeof(uart_msg), "OTP Read: %s\r\n",
           (err == FSP_SUCCESS ? "OK" : "ERR"));
  uart_print(uart_msg);

  icp_init_calib_consts(&g_icp_calib);
  snprintf(uart_msg, sizeof(uart_msg), "OTP c1=%d c2=%d c3=%d c4=%d\r\n",
           g_icp_calib.otp[0], g_icp_calib.otp[1], g_icp_calib.otp[2],
           g_icp_calib.otp[3]);
  uart_print(uart_msg);

  /* Initialize HS3001 humidity/temperature sensor (register driver) */
  err = hs3001_init();
  snprintf(uart_msg, sizeof(uart_msg), "HS3001 I2C Open: %s\r\n",
           (err == FSP_SUCCESS ? "OK" : "ERR"));
  uart_print(uart_msg);

  /* Measurement loop */
  uint32_t loop_count = 0;
  while (1) {
    float pressure = 0.0f;
    float temp = 0.0f;

    /* Đọc ICP10101 */
    err = icp10101_read(&pressure, &temp);
    if (FSP_SUCCESS == err) {
      /* Prepare sensor data for queue */
      memset(&sensor_data, 0, sizeof(sensor_data_t));
      snprintf(sensor_data.sensor_name, sizeof(sensor_data.sensor_name),
               "icp10101");
      sensor_data.value1 = pressure;
      sensor_data.value2 = temp;
      snprintf(sensor_data.unit1, sizeof(sensor_data.unit1), "Pa");
      snprintf(sensor_data.unit2, sizeof(sensor_data.unit2), "C");

      /* Send to queue */
      if (internet_connected) {
        if (xQueueSend(g_sensor_data_queue, &sensor_data, pdMS_TO_TICKS(100)) !=
            pdPASS) {
          uart_print("WARN: Queue full, data dropped\r\n");
        }
      }

      snprintf(uart_msg, sizeof(uart_msg),
               "Loop %lu: Pressure: %.2f Pa, Temp: %.2f C [OK]\r\n", loop_count,
               pressure, temp);
      uart_print(uart_msg);
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    /* Đọc HS3001 humidity + temperature */
    float humidity_rh = 0.0f;
    float hs_temp_c = 0.0f;

    err = hs3001_read(&humidity_rh, &hs_temp_c);

    snprintf(uart_msg, sizeof(uart_msg),
             "[HS3001] read: err=%d, RH=%.2f %%RH, T=%.2f C\r\n", (int)err,
             humidity_rh, hs_temp_c);
    uart_print(uart_msg);

    if (FSP_SUCCESS == err) {
      /* Prepare sensor data structure for queue */
      memset(&sensor_data, 0, sizeof(sensor_data_t));
      snprintf(sensor_data.sensor_name, sizeof(sensor_data.sensor_name),
               "hs3001");

      /* Map HS3001 results to generic sensor data fields */
      sensor_data.value1 = humidity_rh; /* Relative humidity */
      sensor_data.value2 = hs_temp_c;   /* Temperature */

      snprintf(sensor_data.unit1, sizeof(sensor_data.unit1), "%%RH");
      snprintf(sensor_data.unit2, sizeof(sensor_data.unit2), "C");

      /* Send to queue (non-blocking) */
      if (internet_connected) {
        if (xQueueSend(g_sensor_data_queue, &sensor_data, pdMS_TO_TICKS(100)) !=
            pdPASS) {
          uart_print("WARN: HS3001 queue full, data dropped\r\n");
        }
      }
    }

    /* Blink LED */
    // R_BSP_PinAccessEnable();
    // R_BSP_PinWrite(g_bsp_leds.p_leds[0], pin_level);
    // R_BSP_PinAccessDisable();
    pin_level =
        (BSP_IO_LEVEL_LOW == pin_level) ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW;

    loop_count++;
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
  (void)xTask;
  (void)pcTaskName;
  uart_print("ERROR: Stack overflow detected!\r\n");
  for (;;) {
    /* Loop forever */
  }
}
