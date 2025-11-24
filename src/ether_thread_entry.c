#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"
#include "ether_thread.h"
#include "rtc_ds1307.h"
#include "sensor_data_queue.h"
#include <stdio.h>
#include <string.h>


/* Server configuration */
#define SERVER_IP_ADDR0 192
#define SERVER_IP_ADDR1 168
#define SERVER_IP_ADDR2 1
#define SERVER_IP_ADDR3 100
#define SERVER_PORT 8888

/* Device name */
#define DEVICE_NAME "ck-ra6m5-01"

/* Format sensor data with RTC timestamp */
int format_sensor_message(char *buffer, size_t bufsize, rtc_time_t *rtc_time,
                          sensor_data_t *sensor) {
  return snprintf(
      buffer, bufsize,
      "[20%02d-%02d-%02d %02d:%02d:%02d] dev=%s sensor=%s %s=%.1f%s "
      "%s=%.0f\r\n",
      rtc_time->year, rtc_time->month, rtc_time->date, rtc_time->hours,
      rtc_time->minutes, rtc_time->seconds, DEVICE_NAME, sensor->sensor_name,
      (strcmp(sensor->sensor_name, "zmod4410") == 0) ? "tvoc" : "pressure",
      sensor->value1, sensor->unit1,
      (strcmp(sensor->sensor_name, "zmod4410") == 0) ? "iaq" : "temp",
      sensor->value2);
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
  err = ds1307_init();
  snprintf(uart_msg, sizeof(uart_msg), "[ETHER] DS1307 Init: %s\r\n",
           (err == FSP_SUCCESS ? "OK" : "ERR"));
  uart_print(uart_msg);

// Set initial time (uncomment to set time once)

  rtc_time_t init_time = {
      .seconds = 0,
      .minutes = 30,
      .hours = 14,
      .day = 1,      // Monday
      .date = 21,
      .month = 8,
      .year = 25     // 2025
  };
  ds1307_set_time(&init_time);
  uart_print("[ETHER] RTC time initialized\r\n");

  /* Wait for network to be up */
  uart_print("[ETHER] Waiting for network...\r\n");
  while (FreeRTOS_IsNetworkUp() == pdFALSE) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  ulIPAddress = FreeRTOS_GetIPAddress();
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

      /* Connected - process sensor data from queue */
      while (1) {
        /* Wait for sensor data from queue */
        if (xQueueReceive(g_sensor_data_queue, &sensor_data,
                          pdMS_TO_TICKS(5000)) == pdPASS) {

          /* Read current time from RTC */
          err = ds1307_get_time(&current_time);

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
