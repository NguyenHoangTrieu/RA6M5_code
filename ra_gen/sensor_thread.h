/* generated thread header file - do not edit */
#ifndef SENSOR_THREAD_H_
#define SENSOR_THREAD_H_
#include "bsp_api.h"
                #include "FreeRTOS.h"
                #include "task.h"
                #include "semphr.h"
                #include "hal_data.h"
                #ifdef __cplusplus
                extern "C" void sensor_thread_entry(void * pvParameters);
                #else
                extern void sensor_thread_entry(void * pvParameters);
                #endif
#include "r_sci_uart.h"
            #include "r_uart_api.h"
#include "../ra/fsp/src/rm_comms_lock/rm_comms_lock.h"
#include "rm_comms_uart.h"
#include "rm_comms_api.h"
#include "rm_comms_i2c.h"
#include "rm_comms_api.h"
FSP_HEADER
/** UART on SCI Instance. */
            extern const uart_instance_t      g_uart0;

            /** Access the UART instance using these structures when calling API functions directly (::p_api is not used). */
            extern sci_uart_instance_ctrl_t     g_uart0_ctrl;
            extern const uart_cfg_t g_uart0_cfg;
            extern const sci_uart_extended_cfg_t g_uart0_cfg_extend;

            #ifndef NULL
            void NULL(uart_callback_args_t * p_args);
            #endif
/* UART Communication Device */
extern const rm_comms_instance_t g_comms_uart0;
extern rm_comms_uart_instance_ctrl_t g_comms_uart0_ctrl;
extern const rm_comms_cfg_t g_comms_uart0_cfg;

#ifndef NULL
void NULL(rm_comms_callback_args_t * p_args);
#endif
/* I2C Communication Device */
extern const rm_comms_instance_t g_icp_comms_i2c;
extern rm_comms_i2c_instance_ctrl_t g_icp_comms_i2c_ctrl;
extern const rm_comms_cfg_t g_icp_comms_i2c_cfg;
#ifndef icp_comms_i2c_callback
void icp_comms_i2c_callback(rm_comms_callback_args_t * p_args);
#endif
/* I2C Communication Device */
extern const rm_comms_instance_t g_biomet_comms_i2c;
extern rm_comms_i2c_instance_ctrl_t g_biomet_comms_i2c_ctrl;
extern const rm_comms_cfg_t g_biomet_comms_i2c_cfg;
#ifndef biomet_comms_i2c_callback
void biomet_comms_i2c_callback(rm_comms_callback_args_t * p_args);
#endif
FSP_FOOTER
#endif /* SENSOR_THREAD_H_ */
