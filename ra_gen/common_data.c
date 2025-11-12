/* generated common source file - do not edit */
#include "common_data.h"

const ether_phy_lsi_cfg_t g_ether_phy_lsi0 =
{
    .address           = 0,
    .type              = ETHER_PHY_LSI_TYPE_KIT_COMPONENT,
};
ether_phy_instance_ctrl_t g_ether_phy0_ctrl;
#define RA_NOT_DEFINED (1)
const ether_phy_extended_cfg_t g_ether_phy0_extended_cfg =
{
    .p_target_init                     = NULL,
    .p_target_link_partner_ability_get = NULL,
    .p_phy_lsi_cfg_list = {
#if (RA_NOT_DEFINED != g_ether_phy_lsi0)
    	&g_ether_phy_lsi0,
#else
    	NULL,
#endif
    },
};
#undef RA_NOT_DEFINED
const ether_phy_cfg_t g_ether_phy0_cfg =
{

    .channel                   = 0,
    .phy_lsi_address           = 0,
    .phy_reset_wait_time       = 0x00020000,
    .mii_bit_access_wait_time  = 8,
    .phy_lsi_type              = ETHER_PHY_LSI_TYPE_KIT_COMPONENT,
    .flow_control              = ETHER_PHY_FLOW_CONTROL_DISABLE,
    .mii_type                  = ETHER_PHY_MII_TYPE_RMII,
    .p_context                 = NULL,
    .p_extend                  = &g_ether_phy0_extended_cfg,

};
/* Instance structure to use this module. */
const ether_phy_instance_t g_ether_phy0 =
{
    .p_ctrl        = &g_ether_phy0_ctrl,
    .p_cfg         = &g_ether_phy0_cfg,
    .p_api         = &g_ether_phy_on_ether_phy
};
ether_instance_ctrl_t g_ether0_ctrl;

            uint8_t g_ether0_mac_address[6] = { 0x00,0x11,0x22,0x33,0x44,0x55 };

            __attribute__((__aligned__(16))) ether_instance_descriptor_t g_ether0_tx_descriptors[1] ETHER_BUFFER_PLACE_IN_SECTION;
            __attribute__((__aligned__(16))) ether_instance_descriptor_t g_ether0_rx_descriptors[1] ETHER_BUFFER_PLACE_IN_SECTION;

            __attribute__((__aligned__(32)))uint8_t g_ether0_ether_buffer0[1536]ETHER_BUFFER_PLACE_IN_SECTION;
__attribute__((__aligned__(32)))uint8_t g_ether0_ether_buffer1[1536]ETHER_BUFFER_PLACE_IN_SECTION;


            uint8_t *pp_g_ether0_ether_buffers[2] = {
(uint8_t *) &g_ether0_ether_buffer0[0],
(uint8_t *) &g_ether0_ether_buffer1[0],
};

            const ether_extended_cfg_t g_ether0_extended_cfg_t =
            {
                .p_rx_descriptors   = g_ether0_rx_descriptors,
                .p_tx_descriptors   = g_ether0_tx_descriptors,
                .eesr_event_filter     = (ETHER_EESR_EVENT_MASK_RFOF | ETHER_EESR_EVENT_MASK_RDE | ETHER_EESR_EVENT_MASK_FR | ETHER_EESR_EVENT_MASK_TFUF | ETHER_EESR_EVENT_MASK_TDE | ETHER_EESR_EVENT_MASK_TC |  0U),
                .ecsr_event_filter     = ( 0U),
            };

            const ether_cfg_t g_ether0_cfg =
            {
                .channel            = 0,
                .zerocopy           = ETHER_ZEROCOPY_DISABLE,
                .multicast          = ETHER_MULTICAST_ENABLE,
                .promiscuous        = ETHER_PROMISCUOUS_DISABLE,
                .flow_control       = ETHER_FLOW_CONTROL_DISABLE,
                .padding            = ETHER_PADDING_DISABLE,
                .padding_offset     = 0,
                .broadcast_filter   = 0,
                .p_mac_address      = g_ether0_mac_address,

                .num_tx_descriptors = 1,
                .num_rx_descriptors = 1,

                .pp_ether_buffers   = pp_g_ether0_ether_buffers,

                .ether_buffer_size  = 1536,

#if defined(VECTOR_NUMBER_EDMAC0_EINT)
                .irq                = VECTOR_NUMBER_EDMAC0_EINT,
#else
                .irq                = FSP_INVALID_VECTOR,
#endif

                .interrupt_priority = (12),

                .p_callback         = vEtherISRCallback,
                .p_ether_phy_instance = &g_ether_phy0,
                .p_context          = &g_freertos_plus_tcp0,
                .p_extend           = &g_ether0_extended_cfg_t,
            };

/* Instance structure to use this module. */
const ether_instance_t g_ether0 =
{
    .p_ctrl        = &g_ether0_ctrl,
    .p_cfg         = &g_ether0_cfg,
    .p_api         = &g_ether_on_ether
};
#if (ipconfigIPv4_BACKWARD_COMPATIBLE == 0)
 NetworkInterface_t g_freertos_plus_tcp0_xInterface = {.pvArgument = (void *) &g_freertos_plus_tcp0};
#else
 rm_freertos_plus_tcp_instance_t * gp_freertos_plus_tcp_instance = &g_freertos_plus_tcp0;
#endif

static rm_freertos_plus_tcp_ctrl_t g_freertos_plus_tcp0_ctrl;

static rm_freertos_plus_tcp_cfg_t  g_freertos_plus_tcp0_cfg =
{
    .p_ether_instance = (ether_instance_t *)(&g_ether0),
    .rx_handler_task_stacksize = configMINIMAL_STACK_SIZE,
    .rx_handler_task_priority = configMAX_PRIORITIES - 1,
    .check_link_status_task_stacksize = configMINIMAL_STACK_SIZE,
    .check_link_status_task_priority = configMAX_PRIORITIES - 1,
    .link_check_interval = 1000,
};

rm_freertos_plus_tcp_instance_t g_freertos_plus_tcp0 =
{
    .p_ctrl = &g_freertos_plus_tcp0_ctrl,
    .p_cfg  = &g_freertos_plus_tcp0_cfg
};
iic_master_instance_ctrl_t g_i2c_master0_ctrl;
const iic_master_extended_cfg_t g_i2c_master0_extend =
{
    .timeout_mode             = IIC_MASTER_TIMEOUT_MODE_SHORT,
    .timeout_scl_low          = IIC_MASTER_TIMEOUT_SCL_LOW_ENABLED,
    .smbus_operation         = 0,
    /* Actual calculated bitrate: 98425. Actual calculated duty cycle: 50%. */ .clock_settings.brl_value = 28, .clock_settings.brh_value = 28, .clock_settings.cks_value = 3, .clock_settings.sddl_value = 0, .clock_settings.dlcs_value = 0,
};
const i2c_master_cfg_t g_i2c_master0_cfg =
{
    .channel             = 0,
    .rate                = I2C_MASTER_RATE_STANDARD,
    .slave               = 0x00,
    .addr_mode           = I2C_MASTER_ADDR_MODE_7BIT,
#define RA_NOT_DEFINED (1)
#if (RA_NOT_DEFINED == RA_NOT_DEFINED)
                .p_transfer_tx       = NULL,
#else
                .p_transfer_tx       = &RA_NOT_DEFINED,
#endif
#if (RA_NOT_DEFINED == RA_NOT_DEFINED)
                .p_transfer_rx       = NULL,
#else
                .p_transfer_rx       = &RA_NOT_DEFINED,
#endif
#undef RA_NOT_DEFINED
    .p_callback          = rm_comms_i2c_callback,
    .p_context           = NULL,
#if defined(VECTOR_NUMBER_IIC0_RXI)
    .rxi_irq             = VECTOR_NUMBER_IIC0_RXI,
#else
    .rxi_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_IIC0_TXI)
    .txi_irq             = VECTOR_NUMBER_IIC0_TXI,
#else
    .txi_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_IIC0_TEI)
    .tei_irq             = VECTOR_NUMBER_IIC0_TEI,
#else
    .tei_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_IIC0_ERI)
    .eri_irq             = VECTOR_NUMBER_IIC0_ERI,
#else
    .eri_irq             = FSP_INVALID_VECTOR,
#endif
    .ipl                 = (12),
    .p_extend            = &g_i2c_master0_extend,
};
/* Instance structure to use this module. */
const i2c_master_instance_t g_i2c_master0 =
{
    .p_ctrl        = &g_i2c_master0_ctrl,
    .p_cfg         = &g_i2c_master0_cfg,
    .p_api         = &g_i2c_master_on_iic
};
#if BSP_CFG_RTOS
#if BSP_CFG_RTOS == 1
#if !defined(g_comms_i2c_bus0_recursive_mutex)
TX_MUTEX g_comms_i2c_bus0_recursive_mutex_handle;
CHAR g_comms_i2c_bus0_recursive_mutex_name[] = "g_comms_i2c_bus0 recursive mutex";
#endif
#if !defined(g_comms_i2c_bus0_blocking_semaphore)
TX_SEMAPHORE g_comms_i2c_bus0_blocking_semaphore_handle;
CHAR g_comms_i2c_bus0_blocking_semaphore_name[] = "g_comms_i2c_bus0 blocking semaphore";
#endif
#elif BSP_CFG_RTOS == 2
#if !defined(g_comms_i2c_bus0_recursive_mutex)
SemaphoreHandle_t g_comms_i2c_bus0_recursive_mutex_handle;
StaticSemaphore_t g_comms_i2c_bus0_recursive_mutex_memory;
#endif
#if !defined(g_comms_i2c_bus0_blocking_semaphore)
SemaphoreHandle_t g_comms_i2c_bus0_blocking_semaphore_handle;
StaticSemaphore_t g_comms_i2c_bus0_blocking_semaphore_memory;
#endif
#endif

#if !defined(g_comms_i2c_bus0_recursive_mutex)
/* Recursive Mutex for I2C bus */
rm_comms_i2c_mutex_t g_comms_i2c_bus0_recursive_mutex =
{
    .p_mutex_handle = &g_comms_i2c_bus0_recursive_mutex_handle,
#if BSP_CFG_RTOS == 1 // ThradX
    .p_mutex_name = &g_comms_i2c_bus0_recursive_mutex_name[0],
#elif BSP_CFG_RTOS == 2 // FreeRTOS
    .p_mutex_memory = &g_comms_i2c_bus0_recursive_mutex_memory,
#endif
};
#endif

#if !defined(g_comms_i2c_bus0_blocking_semaphore)
/* Semaphore for blocking */
rm_comms_i2c_semaphore_t g_comms_i2c_bus0_blocking_semaphore =
{
    .p_semaphore_handle = &g_comms_i2c_bus0_blocking_semaphore_handle,
#if BSP_CFG_RTOS == 1 // ThreadX
    .p_semaphore_name = &g_comms_i2c_bus0_blocking_semaphore_name[0],
#elif BSP_CFG_RTOS == 2 // FreeRTOS
    .p_semaphore_memory = &g_comms_i2c_bus0_blocking_semaphore_memory,
#endif
};
#endif
#endif

/* Shared I2C Bus */
#define RA_NOT_DEFINED (1)
rm_comms_i2c_bus_extended_cfg_t g_comms_i2c_bus0_extended_cfg =
{
#if !defined(g_i2c_master0)
    .p_driver_instance      = (void*)&g_i2c_master0,
#elif !defined(RA_NOT_DEFINED)
    .p_driver_instance      = (void*)&RA_NOT_DEFINED,
#elif !defined(RA_NOT_DEFINED)
    .p_driver_instance      = (void*)&RA_NOT_DEFINED,
#endif
    .p_current_ctrl = NULL,
    .bus_timeout = 500,
#if BSP_CFG_RTOS
#if !defined(g_comms_i2c_bus0_blocking_semaphore)
    .p_blocking_semaphore = &g_comms_i2c_bus0_blocking_semaphore,
#if !defined(g_comms_i2c_bus0_recursive_mutex)
    .p_bus_recursive_mutex = &g_comms_i2c_bus0_recursive_mutex,
#else
    .p_bus_recursive_mutex = NULL,
#endif
#else
    .p_bus_recursive_mutex = NULL,
    .p_blocking_semaphore = NULL,
#endif
#else
#endif

#if (0)
    .p_elc = (void*)&g_elc,
    .p_timer = (void*)&g_timer,
#else
    .p_elc = NULL,
    .p_timer = NULL,
#endif
};
#undef RA_NOT_DEFINED
ioport_instance_ctrl_t g_ioport_ctrl;
const ioport_instance_t g_ioport =
        {
            .p_api = &g_ioport_on_ioport,
            .p_ctrl = &g_ioport_ctrl,
            .p_cfg = &g_bsp_pin_cfg,
        };
void g_common_init(void) {
}
