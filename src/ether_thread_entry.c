#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"
#include "ether_thread.h"
#include "rtc_ds1307.h"
#include "sensor_data_queue.h"
#include <stdio.h>
#include <string.h>

/* Default IP address configuration */
#ifndef configIP_ADDR0
#define configIP_ADDR0 192
#define configIP_ADDR1 168
#define configIP_ADDR2 1
#define configIP_ADDR3 150
#endif

/*Gateway Config*/
#define configGATEWAY_ADDR0 192
#define configGATEWAY_ADDR1 168
#define configGATEWAY_ADDR2 1
#define configGATEWAY_ADDR3 1

/* Server configuration */
#define SERVER_IP_ADDR0 192
#define SERVER_IP_ADDR1 168
#define SERVER_IP_ADDR2 1
#define SERVER_IP_ADDR3 9
#define SERVER_PORT 8888

/* Device name */
#define DEVICE_NAME "ck-ra6m5-01"

/* Format sensor data with RTC timestamp */
int format_sensor_message(char *buffer, size_t bufsize, rtc_time_t *rtc_time,
                          sensor_data_t *sensor) {
  const char *name1 = "pressure";
  const char *name2 = "temp";

  if (strcmp(sensor->sensor_name, "icp10101") == 0) {
    name1 = "pressure";
    name2 = "temp";
  } else if (strcmp(sensor->sensor_name, "hs3001") == 0) {
    name1 = "humidity";
    name2 = "temp";
  }

  return snprintf(buffer, bufsize,
                  "[20%02d-%02d-%02d %02d:%02d:%02d] dev=%s sensor=%s "
                  "%s=%.1f%s %s=%.0f%s\r\n",
                  rtc_time->year, rtc_time->month, rtc_time->date,
                  rtc_time->hours, rtc_time->minutes, rtc_time->seconds,
                  DEVICE_NAME, sensor->sensor_name, name1, sensor->value1,
                  sensor->unit1, name2, sensor->value2, sensor->unit2);
}

/* ===== I2C scan (bus 1 - RTC) ===== */
extern const rm_comms_cfg_t g_comms_i2c_device_rtc_cfg;
extern const i2c_master_cfg_t g_comms_i2c_device_rtc_lower_level_cfg;
extern ether_phy_instance_ctrl_t g_ether_phy0_ctrl;

static volatile uint8_t g_scan_done = 0;
static volatile rm_comms_event_t g_scan_event = 0;

static void i2c_scan_callback(rm_comms_callback_args_t *p_args) {
  if (NULL == p_args) {
    return;
  }

  g_scan_event = p_args->event;
  g_scan_done = 1;
}

/* Scan all 7-bit I2C addresses on the same bus as RTC (bus1) */
void i2c_scan_bus1(void) {
  char msg[64];

  uart_print("I2C scan on bus1 start...\r\n");

  for (uint8_t addr = 1; addr < 0x7F; addr++) {
    /* Local control block for this temporary rm_comms instance */
    rm_comms_i2c_instance_ctrl_t scan_ctrl;
    memset(&scan_ctrl, 0, sizeof(scan_ctrl));

    /* Copy lower-level I2C config (channel, pins, speed...) from */
    i2c_master_cfg_t lower_cfg = g_comms_i2c_device_rtc_lower_level_cfg;
    lower_cfg.slave = addr; /* try this 7-bit address */

    /* Copy rm_comms config, but override lower-level cfg + callback */
    rm_comms_cfg_t scan_cfg = g_comms_i2c_device_rtc_cfg;
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

  uart_print("I2C scan on bus1 done.\r\n");
}

/* TCP Client task */
void ether_thread_entry(void *pvParameters) {
  FSP_PARAMETER_NOT_USED(pvParameters);

  fsp_err_t err;
  Socket_t xSocket = NULL;
  struct freertos_sockaddr xServerAddress;
  BaseType_t xStatus;
  char msg_buffer[256];
  char uart_msg[128];
  sensor_data_t sensor_data;
  rtc_time_t current_time;
  uint32_t ulIPAddress;

  uart_print("=== Ethernet Thread Started ===\r\n");
  /* Initialize DS1307 RTC */
  // i2c_comms_init(&g_comms_i2c_device_rtc_cfg);
  // i2c_scan_bus1();
  err = ds1307_init();
  snprintf(uart_msg, sizeof(uart_msg), "[ETHER] DS1307 Init: %s\r\n",
           (err == FSP_SUCCESS ? "OK" : "ERR"));
  uart_print(uart_msg);

  // Set initial time (uncomment to set time once)

  rtc_time_t init_time = {
      .seconds = 0,
      .minutes = 30,
      .hours = 14,
      .day = 1, // Monday
      .date = 21,
      .month = 8,
      .year = 25 // 2025
  };
  ds1307_set_time(&init_time);
  uart_print("[ETHER] RTC time initialized\r\n");

  // R_BSP_PinAccessEnable();

  // R_BSP_PinWrite(BSP_IO_PORT_02_PIN_10, BSP_IO_LEVEL_LOW);
  // R_BSP_SoftwareDelay(50, BSP_DELAY_UNITS_MILLISECONDS);
  // R_BSP_PinWrite(BSP_IO_PORT_02_PIN_10, BSP_IO_LEVEL_HIGH);

  // R_BSP_PinAccessDisable();
  // R_BSP_SoftwareDelay(500, BSP_DELAY_UNITS_MILLISECONDS);
  // uart_print("[PHY] Reset done. Starting IP Stack...\r\n");
  const uint8_t ucIPAddress[4] = {configIP_ADDR0, configIP_ADDR1,
                                  configIP_ADDR2, configIP_ADDR3};
  const uint8_t ucNetMask[4] = {255, 255, 255, 0};
  const uint8_t ucGatewayAddress[4] = {configGATEWAY_ADDR0, configGATEWAY_ADDR1,
                                       configGATEWAY_ADDR2,
                                       configGATEWAY_ADDR3};
  const uint8_t ucDNSServerAddress[4] = {8, 8, 8, 8};
  const uint8_t ucMACAddress[6] = {0x74, 0x90, 0x50, 0x10, 0xC5, 0x11};
  uart_print("[ETHER] Force Initializing Static IP...\r\n");
  FreeRTOS_IPInit(ucIPAddress, ucNetMask, ucGatewayAddress, ucDNSServerAddress,
                  ucMACAddress);
  /* Wait for network to be up */
  uart_print("[ETHER] Waiting for network...\r\n");
  while (FreeRTOS_IsNetworkUp() == pdFALSE) {
    uart_print("[ETHER] Network down, retrying in 1s...\r\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  ulIPAddress = FreeRTOS_GetIPAddress();
  while (ulIPAddress == 0) {
    uart_print("[ETHER] Link Up but IP is 0.0.0.0. Waiting for DHCP...\r\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    ulIPAddress = FreeRTOS_GetIPAddress();
  }
  snprintf(uart_msg, sizeof(uart_msg),
           "[ETHER] Network UP! IP: %u.%u.%u.%u\r\n",
           (unsigned int)(ulIPAddress & 0xFF),
           (unsigned int)((ulIPAddress >> 8) & 0xFF),
           (unsigned int)((ulIPAddress >> 16) & 0xFF),
           (unsigned int)((ulIPAddress >> 24) & 0xFF));
  uart_print(uart_msg);

  /* Configure server address */
  xServerAddress.sin_port = FreeRTOS_htons(SERVER_PORT);
  xServerAddress.sin_addr = FreeRTOS_inet_addr_quick(
      SERVER_IP_ADDR0, SERVER_IP_ADDR1, SERVER_IP_ADDR2, SERVER_IP_ADDR3);

  snprintf(uart_msg, sizeof(uart_msg), "[ETHER] Server: %d.%d.%d.%d:%d\r\n",
           SERVER_IP_ADDR0, SERVER_IP_ADDR1, SERVER_IP_ADDR2, SERVER_IP_ADDR3,
           SERVER_PORT);
  uart_print(uart_msg);

  while (1) {
    /* Create TCP socket */
    xSocket = FreeRTOS_socket(FREERTOS_AF_INET, FREERTOS_SOCK_STREAM,
                              FREERTOS_IPPROTO_TCP);

    if (xSocket == FREERTOS_INVALID_SOCKET) {
      uart_print("[ETHER] ERR: Socket creation failed\r\n");
      internet_connected = false;
      vTaskDelay(pdMS_TO_TICKS(5000));
      continue;
    }

    uart_print("[ETHER] Socket created\r\n");

    /* Set socket timeout */
    TickType_t xReceiveTimeOut = pdMS_TO_TICKS(5000);
    FreeRTOS_setsockopt(xSocket, 0, FREERTOS_SO_RCVTIMEO, &xReceiveTimeOut,
                        sizeof(xReceiveTimeOut));

    /* Connect to server */
    uart_print("[ETHER] Connecting to server...\r\n");
    xStatus =
        FreeRTOS_connect(xSocket, &xServerAddress, sizeof(xServerAddress));

    if (xStatus == 0) {
      uart_print("[ETHER] Connected to server!\r\n");
      internet_connected = true;
      /* Connected - process sensor data from queue */
      while (1) {
        if (FreeRTOS_IsNetworkUp() == pdFALSE) {
          uart_print("[ETHER] Link lost! Closing socket to reconnect...\r\n");
          internet_connected = false;
          break;
        }
        /* Wait for sensor data from queue */
        if (xQueueReceive(g_sensor_data_queue, &sensor_data,
                          pdMS_TO_TICKS(5000)) == pdPASS) {

          /* Read current time from RTC */
          err = ds1307_get_time(&current_time);
          // current_time.seconds = 0;
          // current_time.minutes = 0;
          // current_time.hours = 0;
          // current_time.day = 1;
          // current_time.date = 1;
          // current_time.month = 1;
          // current_time.year = 0;
          if (err == FSP_SUCCESS) {
            /* Format message with timestamp */
            int msg_len = format_sensor_message(msg_buffer, sizeof(msg_buffer),
                                                &current_time, &sensor_data);

            /* Debug output to UART */
            uart_print("[ETHER] Sending: ");
            uart_print(msg_buffer);

            /* Send to server via TCP */
            BaseType_t sent = FreeRTOS_send(xSocket, msg_buffer, msg_len, 0);

            if (sent < 0) {
              uart_print("[ETHER] ERR: Send failed, reconnecting...\r\n");
              break; // Break to reconnect
            } else {
              snprintf(uart_msg, sizeof(uart_msg), "[ETHER] Sent %ld bytes\r\n",
                       sent);
              uart_print(uart_msg);
            }
          } else {
            uart_print("[ETHER] ERR: RTC read failed\r\n");
          }
        }

        /* Check if socket is still connected */
        if (FreeRTOS_issocketconnected(xSocket) == pdFALSE) {
          uart_print("[ETHER] Socket disconnected\r\n");
          break;
        }
      }
    } else {
      uart_print("[ETHER] ERR: Connection failed\r\n");
    }

    /* Close socket and retry */
    FreeRTOS_closesocket(xSocket);
    uart_print("[ETHER] Socket closed, retrying in 5s...\r\n");
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void vApplicationIPNetworkEventHook(eIPCallbackEvent_t eNetworkEvent) {
  uint32_t ulIPAddress, ulNetMask, ulGatewayAddress, ulDNSServerAddress;
  char cBuffer[32];
  if (eNetworkEvent == eNetworkUp) {
    FreeRTOS_GetAddressConfiguration(&ulIPAddress, &ulNetMask,
                                     &ulGatewayAddress, &ulDNSServerAddress);
    FreeRTOS_inet_ntoa(ulIPAddress, cBuffer);

    uart_print("[HOOK] Network UP! IP: ");
    uart_print(cBuffer);
    uart_print("\r\n");
  } else {
    uart_print("[HOOK] Network DOWN!\r\n");
  }
}