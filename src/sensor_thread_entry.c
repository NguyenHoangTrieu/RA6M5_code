#include "blinky_thread.h"
#include "icp_handler.h"
#include "ob1203_handler.h"
#include "sensor_data_queue.h"
#include "sensor_thread.h"
#include <stdio.h>
#include <string.h>

extern bsp_leds_t g_bsp_leds;
extern rm_comms_i2c_instance_ctrl_t g_icp_comms_i2c_ctrl;
extern rm_comms_i2c_instance_ctrl_t g_biomet_comms_i2c_ctrl;
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

/* Callback used only for I2C scan */
extern const rm_comms_cfg_t g_biomet_comms_i2c_cfg;
extern const i2c_master_cfg_t g_biomet_comms_i2c_lower_level_cfg;

static volatile uint8_t g_scan_done = 0;
static volatile rm_comms_event_t g_scan_event = 0;
static void i2c_scan_callback(rm_comms_callback_args_t *p_args) {
  if (NULL == p_args) {
    return;
  }

  g_scan_event = p_args->event;
  g_scan_done = 1;
}

/* Scan all 7-bit I2C addresses on the same bus as OB1203 (bus0) */
void i2c_scan_bus0(void) {
  char msg[64];

  uart_print("I2C scan on bus0 start...\r\n");

  for (uint8_t addr = 1; addr < 0x7F; addr++) {
    /* Local control block for this temporary rm_comms instance */
    rm_comms_i2c_instance_ctrl_t scan_ctrl;
    memset(&scan_ctrl, 0, sizeof(scan_ctrl));

    /* Copy lower-level I2C config (channel, pins, speed...) from */
    i2c_master_cfg_t lower_cfg = g_biomet_comms_i2c_lower_level_cfg;
    lower_cfg.slave = addr; /* try this 7-bit address */

    /* Copy rm_comms config, but override lower-level cfg + callback */
    rm_comms_cfg_t scan_cfg = g_biomet_comms_i2c_cfg;
    scan_cfg.p_lower_level_cfg = &lower_cfg;
    scan_cfg.p_callback = i2c_scan_callback;

    /* Open rm_comms instance for this address */
    fsp_err_t err = RM_COMMS_I2C_Open(&scan_ctrl, &scan_cfg);
    if (FSP_SUCCESS != err) {
      /* If open failed for some reason, skip this address */
      continue;
    }

    uint8_t dummy = 0;
    g_scan_done = 0;
    g_scan_event = (rm_comms_event_t)0xFF;

    /* Simple 1-byte write – if the device NACKs, we will get ERROR */
    err = RM_COMMS_I2C_Write(&scan_ctrl, &dummy, 1);
    if (FSP_SUCCESS == err) {
      /* Wait for callback with a simple timeout */
      uint32_t timeout = 0U;
      while (!g_scan_done && timeout++ < 1000000U) {
        __NOP();
      }

      if (g_scan_done && (g_scan_event == RM_COMMS_EVENT_OPERATION_COMPLETE)) {
        /* Device ACKed this address */
        snprintf(msg, sizeof(msg), "I2C device found at 0x%02X\r\n", addr);
        uart_print(msg);
      }
    }

    (void)RM_COMMS_I2C_Close(&scan_ctrl);
  }

  uart_print("I2C scan on bus0 done.\r\n");
}

void sensor_thread_entry(void *pvParameters) {
  FSP_PARAMETER_NOT_USED(pvParameters);

  char uart_msg[128];
  bsp_io_level_t pin_level = BSP_IO_LEVEL_LOW;
  fsp_err_t err;
  sensor_data_t sensor_data;
  // Create sensor data queue
  sensor_queue_create();
  // Open UART
  err = RM_COMMS_UART_Open(&g_comms_uart0_ctrl, &g_comms_uart0_cfg);
  uart_print("=== ICP-10101 Sensor Verification (new API) ===\r\n");
  void *dummy = pvPortMalloc(1);
  vPortFree(dummy);
  size_t free_heap = xPortGetFreeHeapSize();
  snprintf(uart_msg, sizeof(uart_msg), "[DEBUG] Free heap: %u bytes\r\n",
           free_heap);
  uart_print(uart_msg);
  // Open/init I2C and ICP
  i2c_comms_init(&g_biomet_comms_i2c_cfg);
  i2c_scan_bus0();
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

  /* === Initialize OB1203 biometric sensor === */
  err = ob1203_init();
  snprintf(uart_msg, sizeof(uart_msg), "OB1203 I2C Open: %s\r\n",
           (err == FSP_SUCCESS ? "OK" : "ERR"));
  uart_print(uart_msg);
  /* Configure OB1203 PPG (example settings) */
  ob1203_config_t ob_cfg = {
      .ir_led_current = 64, /* Adjust LED currents as needed */
      .red_led_current = 64,
      .averaging = 16 /* 16x averaging */
  };

  err = ob1203_configure(&ob_cfg);
  snprintf(uart_msg, sizeof(uart_msg), "OB1203 Configure: %s\r\n",
           (err == FSP_SUCCESS ? "OK" : "ERR"));
  uart_print(uart_msg);

  if (FSP_SUCCESS == err) {
    err = ob1203_start_measurement();
    snprintf(uart_msg, sizeof(uart_msg), "OB1203 Start: %s\r\n",
             (err == FSP_SUCCESS ? "OK" : "ERR"));
    uart_print(uart_msg);
  }

  // Measurement loop (Low Noise Mode, API mới)
  uint32_t loop_count = 0;
  while (1) {
    float pressure = 0;
    float temp = 0;
    err = icp10101_read(&pressure, &temp);
    if (err == FSP_SUCCESS) {
      // Prepare sensor data for queue
      memset(&sensor_data, 0, sizeof(sensor_data_t));
      snprintf(sensor_data.sensor_name, sizeof(sensor_data.sensor_name),
               "icp10101");
      sensor_data.value1 = pressure;
      sensor_data.value2 = temp;
      snprintf(sensor_data.unit1, sizeof(sensor_data.unit1), "Pa");
      snprintf(sensor_data.unit2, sizeof(sensor_data.unit2), "C");

      // Send to queue
      if (xQueueSend(g_sensor_data_queue, &sensor_data, pdMS_TO_TICKS(100)) !=
          pdPASS) {
        uart_print("WARN: Queue full, data dropped\r\n");
      }
      snprintf(uart_msg, sizeof(uart_msg),
               "Loop %lu: Pressure: %.2f Pa, Temp: %.2f C [%s]\r\n", loop_count,
               pressure, temp, err == FSP_SUCCESS ? "OK" : "ERR");
      uart_print(uart_msg);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* Read OB1203 - process multiple samples */
    for (uint8_t i = 0; i < 5; i++) {
      ob1203_data_t ob_data;
      fsp_err_t ob_err;

      memset(&ob_data, 0, sizeof(ob_data));

      /* Read one IR + RED sample from OB1203 FIFO */
      ob_err = ob1203_read(&ob_data);

      /* Always print something so we can see what happens */
      snprintf(uart_msg, sizeof(uart_msg),
               "[OB1203] read: err=%d, valid=%u, IR=%lu, RED=%lu\r\n",
               (int)ob_err, (unsigned int)ob_data.valid,
               (unsigned long)ob_data.ir_raw, (unsigned long)ob_data.red_raw);
      uart_print(uart_msg);

      if ((ob_err == FSP_SUCCESS) && (ob_data.valid == 1U)) {
        /* Prepare sensor data structure for queue */
        memset(&sensor_data, 0, sizeof(sensor_data_t));
        snprintf(sensor_data.sensor_name, sizeof(sensor_data.sensor_name),
                 "ob1203");

        /* Map OB1203 results to generic sensor data fields */
        sensor_data.value1 = ob_data.heart_rate_bpm; /* Heart rate */
        sensor_data.value2 = ob_data.spo2_percent;   /* SpO2 */

        snprintf(sensor_data.unit1, sizeof(sensor_data.unit1), "bpm");
        snprintf(sensor_data.unit2, sizeof(sensor_data.unit2), "%%SpO2");

        /* Send to queue (non-blocking) */
        if (xQueueSend(g_sensor_data_queue, &sensor_data, pdMS_TO_TICKS(100)) !=
            pdPASS) {
          uart_print("WARN: OB1203 queue full, data dropped\r\n");
        }

        /* High-level debug log */
        snprintf(uart_msg, sizeof(uart_msg),
                 "OB1203: HR=%.1f bpm, SpO2=%.1f %%\r\n",
                 ob_data.heart_rate_bpm, ob_data.spo2_percent);
        uart_print(uart_msg);
      }

      /* 100 ms between PPG reads */
      vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Blink LED
    R_BSP_PinAccessEnable();
    R_BSP_PinWrite(g_bsp_leds.p_leds[0], pin_level);
    R_BSP_PinAccessDisable();
    pin_level =
        (pin_level == BSP_IO_LEVEL_LOW) ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW;

    loop_count++;
    vTaskDelay(pdMS_TO_TICKS(5000));
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
