#include "blinky_thread.h"
#include "sensor_thread.h"
#include <stdio.h>
#include <string.h>

extern bsp_leds_t g_bsp_leds;
volatile uint8_t i2c_flag = 0;

// ICP-10101 Basic Commands (from datasheet)
#define ICP_CMD_READ_ID_H 0xEF    // Read ID command high byte
#define ICP_CMD_READ_ID_L 0xC8    // Read ID command low byte
#define ICP_CMD_SOFT_RESET_H 0x80 // Soft reset command high byte
#define ICP_CMD_SOFT_RESET_L 0x5D // Soft reset command low byte
#define ICP_CMD_MEASURE_H 0x50    // Low Noise measurement high byte
#define ICP_CMD_MEASURE_L 0x59    // Low Noise measurement low byte

void sensor_thread_entry(void *pvParameters) {
  FSP_PARAMETER_NOT_USED(pvParameters);

  uint8_t icp_cmd[2];
  uint8_t icp_response[10] = {0};
  char uart_msg[128];
  bsp_io_level_t pin_level = BSP_IO_LEVEL_LOW;

  // Initialize all instances
  fsp_err_t err_icp =
      RM_COMMS_I2C_Open(&g_icp_comms_i2c_ctrl, &g_icp_comms_i2c_cfg);
  fsp_err_t err_uart =
      RM_COMMS_UART_Open(&g_comms_uart0_ctrl, &g_comms_uart0_cfg);

  // Report initialization status
  snprintf(uart_msg, sizeof(uart_msg),
           "=== ICP-10101 Simple Verification ===\r\n"
           "Init: ICP_i2c=%s, UART=%s\r\n",
           (err_icp == FSP_SUCCESS ? "OK" : "ERR"),
           (err_uart == FSP_SUCCESS ? "OK" : "ERR"));
  RM_COMMS_UART_Write(&g_comms_uart0_ctrl, (uint8_t *)uart_msg,
                      strlen(uart_msg));

  // Wait a bit for system to stabilize
  vTaskDelay(pdMS_TO_TICKS(100));

  // Step 1: Test ICP-10101 presence by reading Product ID
  snprintf(uart_msg, sizeof(uart_msg),
           "\r\nStep 1: Reading ICP-10101 Product ID...\r\n");
  RM_COMMS_UART_Write(&g_comms_uart0_ctrl, (uint8_t *)uart_msg,
                      strlen(uart_msg));

  // Send Read ID command
  icp_cmd[0] = ICP_CMD_READ_ID_H;
  icp_cmd[1] = ICP_CMD_READ_ID_L;

  fsp_err_t write_err = RM_COMMS_I2C_Write(&g_icp_comms_i2c_ctrl, icp_cmd, 2);
  snprintf(uart_msg, sizeof(uart_msg), "xxx");
  RM_COMMS_UART_Write(&g_comms_uart0_ctrl, (uint8_t *)uart_msg,
                      strlen(uart_msg));
  if (write_err == FSP_SUCCESS) {
    // Wait for I2C callback
    while (!i2c_flag) {
      __NOP();
    }
    i2c_flag = 0;

    // Read response (2 bytes ID + 1 byte CRC)
    fsp_err_t read_err =
        RM_COMMS_I2C_Read(&g_icp_comms_i2c_ctrl, icp_response, 3);
    if (read_err == FSP_SUCCESS) {
      while (!i2c_flag) {
        __NOP();
      }
      i2c_flag = 0;

      uint16_t product_id = (icp_response[0] << 8) | icp_response[1];
      uint8_t crc = icp_response[2];

      snprintf(uart_msg, sizeof(uart_msg),
               "Product ID: 0x%04X, CRC: 0x%02X\r\n", product_id, crc);
      RM_COMMS_UART_Write(&g_comms_uart0_ctrl, (uint8_t *)uart_msg,
                          strlen(uart_msg));

      // Check if bits 5:0 contain expected value 0x08 (from datasheet)
      if ((product_id & 0x3F) == 0x08) {
        snprintf(uart_msg, sizeof(uart_msg),
                 "✓ ICP-10101 detected successfully!\r\n");
      } else {
        snprintf(uart_msg, sizeof(uart_msg),
                 "✗ Unexpected product ID (expected bits 5:0 = 0x08)\r\n");
      }
    } else {
      snprintf(uart_msg, sizeof(uart_msg),
               "✗ Failed to read ID response: 0x%04X\r\n", read_err);
    }
  } else {
    snprintf(uart_msg, sizeof(uart_msg),
             "✗ Failed to send ID command: 0x%04X\r\n", write_err);
  }
  RM_COMMS_UART_Write(&g_comms_uart0_ctrl, (uint8_t *)uart_msg,
                      strlen(uart_msg));

  vTaskDelay(pdMS_TO_TICKS(1000));

  // Step 2: Test soft reset
  snprintf(uart_msg, sizeof(uart_msg), "\r\nStep 2: Testing Soft Reset...\r\n");
  RM_COMMS_UART_Write(&g_comms_uart0_ctrl, (uint8_t *)uart_msg,
                      strlen(uart_msg));

  icp_cmd[0] = ICP_CMD_SOFT_RESET_H;
  icp_cmd[1] = ICP_CMD_SOFT_RESET_L;

  write_err = RM_COMMS_I2C_Write(&g_icp_comms_i2c_ctrl, icp_cmd, 2);
  if (write_err == FSP_SUCCESS) {
    while (!i2c_flag) {
      __NOP();
    }
    i2c_flag = 0;
    snprintf(uart_msg, sizeof(uart_msg),
             "✓ Soft reset command sent successfully\r\n");
  } else {
    snprintf(uart_msg, sizeof(uart_msg), "✗ Soft reset failed: 0x%04X\r\n",
             write_err);
  }
  RM_COMMS_UART_Write(&g_comms_uart0_ctrl, (uint8_t *)uart_msg,
                      strlen(uart_msg));

  // Wait for reset to complete (datasheet: ~170μs typical)
  vTaskDelay(pdMS_TO_TICKS(5));

  // Main loop: Try basic measurement commands
  snprintf(uart_msg, sizeof(uart_msg),
           "\r\nStep 3: Starting measurement loop...\r\n");
  RM_COMMS_UART_Write(&g_comms_uart0_ctrl, (uint8_t *)uart_msg,
                      strlen(uart_msg));

  uint32_t loop_count = 0;

  while (1) {
    // Send measurement command (Low Noise mode)
    icp_cmd[0] = ICP_CMD_MEASURE_H;
    icp_cmd[1] = ICP_CMD_MEASURE_L;

    write_err = RM_COMMS_I2C_Write(&g_icp_comms_i2c_ctrl, icp_cmd, 2);

    if (write_err == FSP_SUCCESS) {
      while (!i2c_flag) {
        __NOP();
      }
      i2c_flag = 0;

      // Wait for measurement to complete (~21ms for Low Noise mode)
      vTaskDelay(pdMS_TO_TICKS(25));

      // Try to read measurement results (9 bytes total)
      fsp_err_t read_err =
          RM_COMMS_I2C_Read(&g_icp_comms_i2c_ctrl, icp_response, 9);

      if (read_err == FSP_SUCCESS) {
        while (!i2c_flag) {
          __NOP();
        }
        i2c_flag = 0;

        snprintf(uart_msg, sizeof(uart_msg),
                 "Loop %lu: Measurement data: %02X %02X %02X %02X %02X %02X "
                 "%02X %02X %02X\r\n",
                 loop_count, icp_response[0], icp_response[1], icp_response[2],
                 icp_response[3], icp_response[4], icp_response[5],
                 icp_response[6], icp_response[7], icp_response[8]);

        // Extract raw pressure (first 3 bytes of actual data)
        uint32_t p_raw = ((uint32_t)icp_response[0] << 16) |
                         ((uint32_t)icp_response[1] << 8) | icp_response[3];

        // Extract raw temperature (bytes 6-7)
        uint16_t t_raw = ((uint16_t)icp_response[6] << 8) | icp_response[7];

        snprintf(uart_msg, sizeof(uart_msg),
                 "Raw: P=0x%06lX (%lu), T=0x%04X (%u)\r\n", p_raw, p_raw, t_raw,
                 t_raw);

      } else {
        snprintf(uart_msg, sizeof(uart_msg),
                 "Loop %lu: Read measurement failed: 0x%04X\r\n", loop_count,
                 read_err);
      }
    } else {
      snprintf(uart_msg, sizeof(uart_msg),
               "Loop %lu: Send measurement cmd failed: 0x%04X\r\n", loop_count,
               write_err);
    }

    RM_COMMS_UART_Write(&g_comms_uart0_ctrl, (uint8_t *)uart_msg,
                        strlen(uart_msg));

    // Blink LED to show activity
    R_BSP_PinAccessEnable();
    R_BSP_PinWrite(g_bsp_leds.p_leds[0], pin_level);
    R_BSP_PinAccessDisable();
    pin_level =
        (pin_level == BSP_IO_LEVEL_LOW) ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW;

    loop_count++;
    vTaskDelay(pdMS_TO_TICKS(500)); // 2 second delay between measurements
  }
}

// I2C callbacks (keep your existing ones)
void icp_comms_i2c_callback(rm_comms_callback_args_t *p_args) {
  if (p_args->event == RM_COMMS_EVENT_OPERATION_COMPLETE) {
    i2c_flag = 1;
  }
}

void zmod_comms_i2c_callback(rm_comms_callback_args_t *p_args) {
  if (p_args->event == RM_COMMS_EVENT_OPERATION_COMPLETE) {
    i2c_flag = 1;
  }
}
