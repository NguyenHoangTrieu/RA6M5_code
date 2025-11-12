/* generated thread header file - do not edit */
#ifndef ETHER_THREAD_H_
#define ETHER_THREAD_H_
#include "bsp_api.h"
                #include "FreeRTOS.h"
                #include "task.h"
                #include "semphr.h"
                #include "hal_data.h"
                #ifdef __cplusplus
                extern "C" void ether_thread_entry(void * pvParameters);
                #else
                extern void ether_thread_entry(void * pvParameters);
                #endif
#include "r_rtc.h"
#include "r_rtc_api.h"
FSP_HEADER
/* RTC Instance. */
extern const rtc_instance_t g_rtc0;

/** Access the RTC instance using these structures when calling API functions directly (::p_api is not used). */
extern rtc_instance_ctrl_t g_rtc0_ctrl;
extern const rtc_cfg_t g_rtc0_cfg;

#ifndef NULL
void NULL(rtc_callback_args_t * p_args);
#endif
FSP_FOOTER
#endif /* ETHER_THREAD_H_ */
