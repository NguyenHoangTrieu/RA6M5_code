/* generated thread source file - do not edit */
#include "ether_thread.h"

#if 1
static StaticTask_t ether_thread_memory;
#if defined(__ARMCC_VERSION) /* AC6 compiler */
static uint8_t
    ether_thread_stack[1024] BSP_PLACE_IN_SECTION(BSP_UNINIT_SECTION_PREFIX
                                                  ".stack.thread")
        BSP_ALIGN_VARIABLE(BSP_STACK_ALIGNMENT);
#else
static uint8_t
    ether_thread_stack[1024] BSP_PLACE_IN_SECTION(BSP_UNINIT_SECTION_PREFIX
                                                  ".stack.ether_thread")
        BSP_ALIGN_VARIABLE(BSP_STACK_ALIGNMENT);
#endif
#endif
TaskHandle_t ether_thread;
void ether_thread_create(void);
static void ether_thread_func(void *pvParameters);
void rtos_startup_err_callback(void *p_instance, void *p_data);
void rtos_startup_common_init(void);
rtc_instance_ctrl_t g_rtc0_ctrl;
const rtc_error_adjustment_cfg_t g_rtc0_err_cfg = {
    .adjustment_mode = RTC_ERROR_ADJUSTMENT_MODE_AUTOMATIC,
    .adjustment_period = RTC_ERROR_ADJUSTMENT_PERIOD_10_SECOND,
    .adjustment_type = RTC_ERROR_ADJUSTMENT_NONE,
    .adjustment_value = 0,
};
const rtc_cfg_t g_rtc0_cfg = {
    .clock_source = RTC_CLOCK_SOURCE_LOCO,
    .freq_compare_value = 255,
    .p_err_cfg = &g_rtc0_err_cfg,
    .p_callback = NULL,
    .p_context = NULL,
    .p_extend = NULL,
    .alarm_ipl = (BSP_IRQ_DISABLED),
    .periodic_ipl = (BSP_IRQ_DISABLED),
    .carry_ipl = (12),
#if defined(VECTOR_NUMBER_RTC_ALARM)
    .alarm_irq = VECTOR_NUMBER_RTC_ALARM,
#else
    .alarm_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_RTC_PERIOD)
    .periodic_irq = VECTOR_NUMBER_RTC_PERIOD,
#else
    .periodic_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_RTC_CARRY)
    .carry_irq = VECTOR_NUMBER_RTC_CARRY,
#else
    .carry_irq = FSP_INVALID_VECTOR,
#endif
};
/* Instance structure to use this module. */
const rtc_instance_t g_rtc0 = {
    .p_ctrl = &g_rtc0_ctrl, .p_cfg = &g_rtc0_cfg, .p_api = &g_rtc_on_rtc};
const ether_phy_lsi_cfg_t g_ether_phy_lsi0 = {
    .address = 0,
    .type = ETHER_PHY_LSI_TYPE_KIT_COMPONENT,
};
ether_phy_instance_ctrl_t g_ether_phy0_ctrl;
#define RA_NOT_DEFINED (1)
const ether_phy_extended_cfg_t g_ether_phy0_extended_cfg = {
    .p_target_init = NULL,
    .p_target_link_partner_ability_get = NULL,
    .p_phy_lsi_cfg_list =
        {
#if (RA_NOT_DEFINED != g_ether_phy_lsi0)
            &g_ether_phy_lsi0,
#else
            NULL,
#endif
        },
};
#undef RA_NOT_DEFINED
const ether_phy_cfg_t g_ether_phy0_cfg = {

    .channel = 0,
    .phy_lsi_address = 0,
    .phy_reset_wait_time = 0x00020000,
    .mii_bit_access_wait_time = 8,
    .phy_lsi_type = ETHER_PHY_LSI_TYPE_KIT_COMPONENT,
    .flow_control = ETHER_PHY_FLOW_CONTROL_DISABLE,
    .mii_type = ETHER_PHY_MII_TYPE_RMII,
    .p_context = NULL,
    .p_extend = &g_ether_phy0_extended_cfg,

};
/* Instance structure to use this module. */
const ether_phy_instance_t g_ether_phy0 = {.p_ctrl = &g_ether_phy0_ctrl,
                                           .p_cfg = &g_ether_phy0_cfg,
                                           .p_api = &g_ether_phy_on_ether_phy};
ether_instance_ctrl_t g_ether0_ctrl;

uint8_t g_ether0_mac_address[6] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};

__attribute__((__aligned__(16))) ether_instance_descriptor_t
    g_ether0_tx_descriptors[1] ETHER_BUFFER_PLACE_IN_SECTION;
__attribute__((__aligned__(16))) ether_instance_descriptor_t
    g_ether0_rx_descriptors[1] ETHER_BUFFER_PLACE_IN_SECTION;

__attribute__((__aligned__(
    32))) uint8_t g_ether0_ether_buffer0[1536] ETHER_BUFFER_PLACE_IN_SECTION;
__attribute__((__aligned__(
    32))) uint8_t g_ether0_ether_buffer1[1536] ETHER_BUFFER_PLACE_IN_SECTION;

uint8_t *pp_g_ether0_ether_buffers[2] = {
    (uint8_t *)&g_ether0_ether_buffer0[0],
    (uint8_t *)&g_ether0_ether_buffer1[0],
};

const ether_extended_cfg_t g_ether0_extended_cfg_t = {
    .p_rx_descriptors = g_ether0_rx_descriptors,
    .p_tx_descriptors = g_ether0_tx_descriptors,
    .eesr_event_filter =
        (ETHER_EESR_EVENT_MASK_RFOF | ETHER_EESR_EVENT_MASK_RDE |
         ETHER_EESR_EVENT_MASK_FR | ETHER_EESR_EVENT_MASK_TFUF |
         ETHER_EESR_EVENT_MASK_TDE | ETHER_EESR_EVENT_MASK_TC | 0U),
    .ecsr_event_filter = (0U),
};

const ether_cfg_t g_ether0_cfg = {
    .channel = 0,
    .zerocopy = ETHER_ZEROCOPY_DISABLE,
    .multicast = ETHER_MULTICAST_ENABLE,
    .promiscuous = ETHER_PROMISCUOUS_DISABLE,
    .flow_control = ETHER_FLOW_CONTROL_DISABLE,
    .padding = ETHER_PADDING_DISABLE,
    .padding_offset = 0,
    .broadcast_filter = 0,
    .p_mac_address = g_ether0_mac_address,

    .num_tx_descriptors = 1,
    .num_rx_descriptors = 1,

    .pp_ether_buffers = pp_g_ether0_ether_buffers,

    .ether_buffer_size = 1536,

#if defined(VECTOR_NUMBER_EDMAC0_EINT)
    .irq = VECTOR_NUMBER_EDMAC0_EINT,
#else
    .irq = FSP_INVALID_VECTOR,
#endif

    .interrupt_priority = (12),

    .p_callback = NULL,
    .p_ether_phy_instance = &g_ether_phy0,
    .p_context = NULL,
    .p_extend = &g_ether0_extended_cfg_t,
};

/* Instance structure to use this module. */
const ether_instance_t g_ether0 = {.p_ctrl = &g_ether0_ctrl,
                                   .p_cfg = &g_ether0_cfg,
                                   .p_api = &g_ether_on_ether};
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
      1024 / 4,                         // In words, not bytes
      (void *)&ether_thread_parameters, // pvParameters
      1,
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
