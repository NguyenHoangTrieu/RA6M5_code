/* generated thread source file - do not edit */
#include "ether_thread.h"

#if 1
static StaticTask_t ether_thread_memory;
#if defined(__ARMCC_VERSION) /* AC6 compiler */
static uint8_t
    ether_thread_stack[2048] BSP_PLACE_IN_SECTION(BSP_UNINIT_SECTION_PREFIX
                                                  ".stack.thread")
        BSP_ALIGN_VARIABLE(BSP_STACK_ALIGNMENT);
#else
static uint8_t
    ether_thread_stack[2048] BSP_PLACE_IN_SECTION(BSP_UNINIT_SECTION_PREFIX
                                                  ".stack.ether_thread")
        BSP_ALIGN_VARIABLE(BSP_STACK_ALIGNMENT);
#endif
#endif
TaskHandle_t ether_thread;
void ether_thread_create(void);
static void ether_thread_func(void *pvParameters);
void rtos_startup_err_callback(void *p_instance, void *p_data);
void rtos_startup_common_init(void);
/* I2C Communication Device */
rm_comms_i2c_instance_ctrl_t g_comms_i2c_device_rtc_ctrl;

/* Lower level driver configuration */
const i2c_master_cfg_t g_comms_i2c_device_rtc_lower_level_cfg = {
    .slave = 0x00,
    .addr_mode = I2C_MASTER_ADDR_MODE_7BIT,
    .p_callback = rm_comms_i2c_callback,
};

const rm_comms_cfg_t g_comms_i2c_device_rtc_cfg = {
    .semaphore_timeout = 0xFFFFFFFF,
    .p_lower_level_cfg = (void *)&g_comms_i2c_device_rtc_lower_level_cfg,
    .p_extend = (void *)&g_comms_i2c_bus1_extended_cfg,
    .p_callback = comms_i2c_callback,
#if defined(NULL)
    .p_context = NULL,
#else
    .p_context = (void *)&NULL,
#endif
};

const rm_comms_instance_t g_comms_i2c_device_rtc = {
    .p_ctrl = &g_comms_i2c_device_rtc_ctrl,
    .p_cfg = &g_comms_i2c_device_rtc_cfg,
    .p_api = &g_comms_on_comms_i2c,
};
extern uint32_t g_fsp_common_thread_count;

const rm_freertos_port_parameters_t ether_thread_parameters = {
    .p_context = (void *)NULL,
};

void ether_thread_create(void) {
  /* Increment count so we will know the number of threads created in the RA
   * Configuration editor. */
  g_fsp_common_thread_count++;

  /* Initialize each kernel object. */

#if 1
  ether_thread = xTaskCreateStatic(
#else
  BaseType_t ether_thread_create_err = xTaskCreate(
#endif
      ether_thread_func, (const char *)"Ethernet Thread",
      2048 / 4,                         // In words, not bytes
      (void *)&ether_thread_parameters, // pvParameters
      3,
#if 1
      (StackType_t *)&ether_thread_stack, (StaticTask_t *)&ether_thread_memory
#else
      &ether_thread
#endif
  );

#if 1
  if (NULL == ether_thread) {
    rtos_startup_err_callback(ether_thread, 0);
  }
#else
  if (pdPASS != ether_thread_create_err) {
    rtos_startup_err_callback(ether_thread, 0);
  }
#endif
}
static void ether_thread_func(void *pvParameters) {
  /* Initialize common components */
  rtos_startup_common_init();

  /* Initialize each module instance. */

#if (1 == BSP_TZ_NONSECURE_BUILD) && (1 == 1)
  /* When FreeRTOS is used in a non-secure TrustZone application,
   * portALLOCATE_SECURE_CONTEXT must be called prior to calling any non-secure
   * callable function in a thread. The parameter is unused in the FSP
   * implementation. If no slots are available then configASSERT() will be
   * called from vPortSVCHandler_C(). If this occurs, the application will need
   * to either increase the value of the "Process Stack Slots" Property in the
   * rm_tz_context module in the secure project or decrease the number of
   * threads in the non-secure project that are allocating a secure context.
   * Users can control which threads allocate a secure context via the
   * Properties tab when selecting each thread. Note that the idle thread in
   * FreeRTOS requires a secure context so the application will need at least 1
   * secure context even if no user threads make secure calls. */
  portALLOCATE_SECURE_CONTEXT(0);
#endif

  /* Enter user code for this thread. Pass task handle. */
  ether_thread_entry(pvParameters);
}
