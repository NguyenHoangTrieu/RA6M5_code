#include "blinky_thread.h"
#include "sensor_thread.h"
#include <stdio.h>
#include <string.h>

extern bsp_leds_t g_bsp_leds;
extern rm_comms_i2c_instance_ctrl_t g_icp_comms_i2c_ctrl;
extern rm_comms_i2c_instance_ctrl_t g_zmod_comms_i2c_ctrl;
volatile uint8_t i2c_flag = 0;

// ICP-10101 Basic Commands (from datasheet)
#define ICP_CMD_READ_ID_H 0xEF    // Read ID command high byte
#define ICP_CMD_READ_ID_L 0xC8    // Read ID command low byte
#define ICP_CMD_SOFT_RESET_H 0x80 // Soft reset command high byte
#define ICP_CMD_SOFT_RESET_L 0x5D // Soft reset command low byte
#define ICP_CMD_MEASURE_H 0x50    // Low Noise measurement high byte
#define ICP_CMD_MEASURE_L 0x59    // Low Noise measurement low byte

void i2c_comms_init(rm_comms_cfg_t *p_cfg) {
  rm_comms_i2c_bus_extended_cfg_t *p_extend = (rm_comms_i2c_bus_extended_cfg_t *)p_cfg->p_extend;
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
    RM_COMMS_UART_Write(&g_comms_uart0_ctrl, (uint8_t*)msg, strlen(msg));
    R_BSP_SoftwareDelay(5, BSP_DELAY_UNITS_MILLISECONDS);
}

void sensor_thread_entry(void *pvParameters) {
    FSP_PARAMETER_NOT_USED(pvParameters);

    i2c_comms_init(&g_icp_comms_i2c_cfg);

    char uart_msg[128];
    fsp_err_t err;

    err = RM_COMMS_UART_Open(&g_comms_uart0_ctrl, &g_comms_uart0_cfg);
    uart_print("=== ICP-10101 Sensor Task Start ===\r\n");

    err = icp10101_init();
    snprintf(uart_msg, sizeof(uart_msg), "ICP10101 init: %s\r\n", err == FSP_SUCCESS ? "OK" : "FAIL");
    uart_print(uart_msg);

    if (err == FSP_SUCCESS) {
        snprintf(uart_msg, sizeof(uart_msg), "OTP c1=%d c2=%d c3=%d c4=%d\r\n",
            g_icp_calib.otp[0], g_icp_calib.otp[1], g_icp_calib.otp[2], g_icp_calib.otp[3]);
        uart_print(uart_msg);
    }

    while (1) {
        float p_pa = 0, t_c = 0;
        err = icp10101_read(&p_pa, &t_c);
        if (err == FSP_SUCCESS)
            snprintf(uart_msg, sizeof(uart_msg),
                "Pressure: %8.1f Pa (%.2f hPa), Temp: %.2f C\r\n", p_pa, p_pa / 100.0f, t_c);
        else
            snprintf(uart_msg, sizeof(uart_msg), "ICP10101 read error: 0x%04X\r\n", err);
        uart_print(uart_msg);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}