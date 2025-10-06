#include "sensor_thread.h"
#include "blinky_thread.h"
#include <stdio.h>

extern bsp_leds_t g_bsp_leds; // hoặc cấu hình global

void sensor_thread_entry(void *pvParameters) {
  FSP_PARAMETER_NOT_USED(pvParameters);

  fsp_err_t err;
  const char msg[] = "UART TEST INIT\r\n";
  bsp_io_level_t pin_level = BSP_IO_LEVEL_LOW;

  /* Try to init UART */
  bsp_io_port_pin_t uart_status_led = g_bsp_leds.p_leds[0];
  R_BSP_PinAccessEnable();
  R_BSP_PinWrite(uart_status_led, pin_level);
  err = RM_COMMS_UART_Open(&g_comms_uart0_ctrl, &g_comms_uart0_cfg);

  /* Check/init LED (ví dụ LED 0 - là LED bị bỏ qua ở blinky) */

  if (err == FSP_SUCCESS) {
    /* Nếu init OK, bật LED */
    R_BSP_PinWrite(uart_status_led, BSP_IO_LEVEL_HIGH);
    /* Và gửi UART 1 lần */
    RM_COMMS_UART_Write(&g_comms_uart0_ctrl, (uint8_t *)msg, strlen(msg));
  } else {
    /* Nếu fail, tắt LED */
    R_BSP_PinWrite(uart_status_led, BSP_IO_LEVEL_LOW);
  }
  R_BSP_PinAccessDisable();
  snprintf((char *)msg, sizeof(msg), "UART TEST START\r\n");
  while (1) {
    err = RM_COMMS_UART_Write(&g_comms_uart0_ctrl, (uint8_t *)msg, strlen(msg));
    if (err == FSP_SUCCESS) {
      R_BSP_PinAccessEnable();
      R_BSP_PinWrite(uart_status_led, pin_level);
      R_BSP_PinAccessDisable();
    }
    if (pin_level == BSP_IO_LEVEL_LOW) {
      pin_level = BSP_IO_LEVEL_HIGH;
    } else {
      pin_level = BSP_IO_LEVEL_LOW;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}