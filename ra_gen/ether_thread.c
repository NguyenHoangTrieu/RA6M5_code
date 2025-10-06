/* generated thread source file - do not edit */
#include "ether_thread.h"

#if 1
                static StaticTask_t ether_thread_memory;
                #if defined(__ARMCC_VERSION)           /* AC6 compiler */
                static uint8_t ether_thread_stack[2048] BSP_PLACE_IN_SECTION(BSP_UNINIT_SECTION_PREFIX ".stack.thread") BSP_ALIGN_VARIABLE(BSP_STACK_ALIGNMENT);
                #else
                static uint8_t ether_thread_stack[2048] BSP_PLACE_IN_SECTION(BSP_UNINIT_SECTION_PREFIX ".stack.ether_thread") BSP_ALIGN_VARIABLE(BSP_STACK_ALIGNMENT);
                #endif
                #endif
                TaskHandle_t ether_thread;
                void ether_thread_create(void);
                static void ether_thread_func(void * pvParameters);
                void rtos_startup_err_callback(void * p_instance, void * p_data);
                void rtos_startup_common_init(void);
rtc_instance_ctrl_t g_rtc0_ctrl;
const rtc_error_adjustment_cfg_t g_rtc0_err_cfg =
{
    .adjustment_mode         = RTC_ERROR_ADJUSTMENT_MODE_AUTOMATIC,
    .adjustment_period       = RTC_ERROR_ADJUSTMENT_PERIOD_10_SECOND,
    .adjustment_type         = RTC_ERROR_ADJUSTMENT_NONE,
    .adjustment_value        = 0,
};
const rtc_cfg_t g_rtc0_cfg =
{
    .clock_source            = RTC_CLOCK_SOURCE_LOCO,
    .freq_compare_value = 255,
    .p_err_cfg               = &g_rtc0_err_cfg,
    .p_callback              = NULL,
    .p_context               = NULL,
    .p_extend                = NULL,
    .alarm_ipl               = (BSP_IRQ_DISABLED),
    .periodic_ipl            = (BSP_IRQ_DISABLED),
    .carry_ipl               = (12),
#if defined(VECTOR_NUMBER_RTC_ALARM)
    .alarm_irq               = VECTOR_NUMBER_RTC_ALARM,
#else
    .alarm_irq               = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_RTC_PERIOD)
    .periodic_irq            = VECTOR_NUMBER_RTC_PERIOD,
#else
    .periodic_irq            = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_RTC_CARRY)
    .carry_irq               = VECTOR_NUMBER_RTC_CARRY,
#else
    .carry_irq               = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const rtc_instance_t g_rtc0 =
{
    .p_ctrl        = &g_rtc0_ctrl,
    .p_cfg         = &g_rtc0_cfg,
    .p_api         = &g_rtc_on_rtc
};
extern uint32_t g_fsp_common_thread_count;

                const rm_freertos_port_parameters_t ether_thread_parameters =
                {
                    .p_context = (void *) NULL,
                };

                void ether_thread_create (void)
                {
                    /* Increment count so we will know the number of threads created in the RA Configuration editor. */
                    g_fsp_common_thread_count++;

                    /* Initialize each kernel object. */
                    

                    #if 1
                    ether_thread = xTaskCreateStatic(
                    #else
                    BaseType_t ether_thread_create_err = xTaskCreate(
                    #endif
                        ether_thread_func,
                        (const char *)"Ethernet Thread",
                        2048/4, // In words, not bytes
                        (void *) &ether_thread_parameters, //pvParameters
                        3,
                        #if 1
                        (StackType_t *)&ether_thread_stack,
                        (StaticTask_t *)&ether_thread_memory
                        #else
                        & ether_thread
                        #endif
                    );

                    #if 1
                    if (NULL == ether_thread)
                    {
                        rtos_startup_err_callback(ether_thread, 0);
                    }
                    #else
                    if (pdPASS != ether_thread_create_err)
                    {
                        rtos_startup_err_callback(ether_thread, 0);
                    }
                    #endif
                }
                static void ether_thread_func (void * pvParameters)
                {
                    /* Initialize common components */
                    rtos_startup_common_init();

                    /* Initialize each module instance. */
                    

                    #if (1 == BSP_TZ_NONSECURE_BUILD) && (1 == 1)
                    /* When FreeRTOS is used in a non-secure TrustZone application, portALLOCATE_SECURE_CONTEXT must be called prior
                     * to calling any non-secure callable function in a thread. The parameter is unused in the FSP implementation.
                     * If no slots are available then configASSERT() will be called from vPortSVCHandler_C(). If this occurs, the
                     * application will need to either increase the value of the "Process Stack Slots" Property in the rm_tz_context
                     * module in the secure project or decrease the number of threads in the non-secure project that are allocating
                     * a secure context. Users can control which threads allocate a secure context via the Properties tab when
                     * selecting each thread. Note that the idle thread in FreeRTOS requires a secure context so the application
                     * will need at least 1 secure context even if no user threads make secure calls. */
                     portALLOCATE_SECURE_CONTEXT(0);
                    #endif

                    /* Enter user code for this thread. Pass task handle. */
                    ether_thread_entry(pvParameters);
                }
