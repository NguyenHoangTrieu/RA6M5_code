/* generated thread header file - do not edit */
#ifndef BLINKY_THREAD_H_
#define BLINKY_THREAD_H_
#include "FreeRTOS.h"
#include "bsp_api.h"
#include "hal_data.h"
#include "semphr.h"
#include "task.h"
#ifdef __cplusplus
extern "C" void blinky_thread_entry(void *pvParameters);
#else
extern void blinky_thread_entry(void *pvParameters);
#endif
FSP_HEADER
FSP_FOOTER
#endif /* BLINKY_THREAD_H_ */
