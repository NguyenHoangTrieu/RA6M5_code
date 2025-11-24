/* generated thread header file - do not edit */
#ifndef ETHER_THREAD_H_
#define ETHER_THREAD_H_
#include "FreeRTOS.h"
#include "bsp_api.h"
#include "hal_data.h"
#include "semphr.h"
#include "task.h"
#ifdef __cplusplus
extern "C" void ether_thread_entry(void *pvParameters);
#else
extern void ether_thread_entry(void *pvParameters);
#endif
#include "rm_comms_api.h"
#include "rm_comms_i2c.h"

FSP_HEADER
/* I2C Communication Device */
extern const rm_comms_instance_t g_comms_i2c_device_rtc;
extern rm_comms_i2c_instance_ctrl_t g_comms_i2c_device_rtc_ctrl;
extern const rm_comms_cfg_t g_comms_i2c_device_rtc_cfg;
#ifndef comms_i2c_callback
void comms_i2c_callback(rm_comms_callback_args_t *p_args);
#endif
FSP_FOOTER
#endif /* ETHER_THREAD_H_ */
