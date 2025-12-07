/* generated thread source file - do not edit */
#include "sensor_thread.h"

#if 1
                static StaticTask_t sensor_thread_memory;
                #if defined(__ARMCC_VERSION)           /* AC6 compiler */
                static uint8_t sensor_thread_stack[8192] BSP_PLACE_IN_SECTION(BSP_UNINIT_SECTION_PREFIX ".stack.thread") BSP_ALIGN_VARIABLE(BSP_STACK_ALIGNMENT);
                #else
                static uint8_t sensor_thread_stack[8192] BSP_PLACE_IN_SECTION(BSP_UNINIT_SECTION_PREFIX ".stack.sensor_thread") BSP_ALIGN_VARIABLE(BSP_STACK_ALIGNMENT);
                #endif
                #endif
                TaskHandle_t sensor_thread;
                void sensor_thread_create(void);
                static void sensor_thread_func(void * pvParameters);
                void rtos_startup_err_callback(void * p_instance, void * p_data);
                void rtos_startup_common_init(void);
sci_uart_instance_ctrl_t     g_uart0_ctrl;

            baud_setting_t               g_uart0_baud_setting =
            {
                /* Baud rate calculated with 0.469% error. */ .semr_baudrate_bits_b.abcse = 0, .semr_baudrate_bits_b.abcs = 0, .semr_baudrate_bits_b.bgdm = 1, .cks = 0, .brr = 53, .mddr = (uint8_t) 256, .semr_baudrate_bits_b.brme = false
            };

            /** UART extended configuration for UARTonSCI HAL driver */
            const sci_uart_extended_cfg_t g_uart0_cfg_extend =
            {
                .clock                = SCI_UART_CLOCK_INT,
                .rx_edge_start          = SCI_UART_START_BIT_FALLING_EDGE,
                .noise_cancel         = SCI_UART_NOISE_CANCELLATION_DISABLE,
                .rx_fifo_trigger        = SCI_UART_RX_FIFO_TRIGGER_MAX,
                .p_baud_setting         = &g_uart0_baud_setting,
                .flow_control           = SCI_UART_FLOW_CONTROL_RTS,
                #if 0xFF != 0xFF
                .flow_control_pin       = BSP_IO_PORT_FF_PIN_0xFF,
                #else
                .flow_control_pin       = (bsp_io_port_pin_t) UINT16_MAX,
                #endif
                .rs485_setting = {
                    .enable = SCI_UART_RS485_DISABLE,
                    .polarity = SCI_UART_RS485_DE_POLARITY_HIGH,
                #if 0xFF != 0xFF
                    .de_control_pin = BSP_IO_PORT_FF_PIN_0xFF,
                #else
                    .de_control_pin       = (bsp_io_port_pin_t) UINT16_MAX,
                #endif
                },
                .irda_setting = {
                    .ircr_bits_b.ire = 0,
                    .ircr_bits_b.irrxinv = 0,
                    .ircr_bits_b.irtxinv = 0,
                },
            };

            /** UART interface configuration */
            const uart_cfg_t g_uart0_cfg =
            {
                .channel             = 3,
                .data_bits           = UART_DATA_BITS_8,
                .parity              = UART_PARITY_OFF,
                .stop_bits           = UART_STOP_BITS_1,
                .p_callback          = NULL,
                .p_context           = NULL,
                .p_extend            = &g_uart0_cfg_extend,
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
                .rxi_ipl             = (12),
                .txi_ipl             = (12),
                .tei_ipl             = (12),
                .eri_ipl             = (12),
#if defined(VECTOR_NUMBER_SCI3_RXI)
                .rxi_irq             = VECTOR_NUMBER_SCI3_RXI,
#else
                .rxi_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SCI3_TXI)
                .txi_irq             = VECTOR_NUMBER_SCI3_TXI,
#else
                .txi_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SCI3_TEI)
                .tei_irq             = VECTOR_NUMBER_SCI3_TEI,
#else
                .tei_irq             = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_SCI3_ERI)
                .eri_irq             = VECTOR_NUMBER_SCI3_ERI,
#else
                .eri_irq             = FSP_INVALID_VECTOR,
#endif
            };

/* Instance structure to use this module. */
const uart_instance_t g_uart0 =
{
    .p_ctrl        = &g_uart0_ctrl,
    .p_cfg         = &g_uart0_cfg,
    .p_api         = &g_uart_on_sci
};
/* UART Communication Device */

rm_comms_uart_instance_ctrl_t g_comms_uart0_ctrl;

#if BSP_CFG_RTOS == 1 // ThreadX

 #if !defined(g_comms_uart0_tx_mutex)
 rm_comms_mutex_t g_comms_uart0_tx_mutex =
 {
     .p_name = "g_comms_uart0 tx mutex",
 };
 #endif

 #if !defined(g_comms_uart0_rx_mutex)
 rm_comms_mutex_t g_comms_uart0_rx_mutex =
 {
     .p_name = "g_comms_uart0 rx mutex",
 };
 #endif

 #if !defined(g_comms_uart0_tx_semaphore)
 rm_comms_semaphore_t g_comms_uart0_tx_semaphore =
 {
     .p_name = "g_comms_uart0 tx semaphore",
 };
 #endif

  #if !defined(g_comms_uart0_rx_semaphore)
 rm_comms_semaphore_t g_comms_uart0_rx_semaphore =
 {
     .p_name = "g_comms_uart0 rx semaphore",
 };
 #endif

#elif BSP_CFG_RTOS == 2 // FreeRTOS

#if !defined(g_comms_uart0_tx_mutex)
rm_comms_mutex_t g_comms_uart0_tx_mutex;
#endif

#if !defined(g_comms_uart0_rx_mutex)
rm_comms_mutex_t g_comms_uart0_rx_mutex;
#endif
#if !defined(g_comms_uart0_tx_semaphore)
rm_comms_semaphore_t g_comms_uart0_tx_semaphore;
#endif

#if !defined(g_comms_uart0_rx_semaphore)
rm_comms_semaphore_t g_comms_uart0_rx_semaphore;
#endif

#else

#endif

rm_comms_uart_extended_cfg_t g_comms_uart0_extended_cfg =
{
#if BSP_CFG_RTOS

#if !defined(g_comms_uart0_tx_mutex)
    .p_tx_mutex = &g_comms_uart0_tx_mutex,
#else
    .p_tx_mutex = NULL,
#endif

#if !defined(g_comms_uart0_rx_mutex)
    .p_rx_mutex = &g_comms_uart0_rx_mutex,
#else
    .p_rx_mutex = NULL,
#endif

#if !defined(g_comms_uart0_tx_semaphore)
    .p_tx_semaphore = &g_comms_uart0_tx_semaphore,
#else
    .p_tx_semaphore = NULL,
#endif

#if !defined(g_comms_uart0_rx_semaphore)
    .p_rx_semaphore = &g_comms_uart0_rx_semaphore,
#else
    .p_rx_semaphore = NULL,
#endif
    .mutex_timeout  = 0xFFFFFFFF,
#endif
    .p_uart = &g_uart0,
};

const rm_comms_cfg_t g_comms_uart0_cfg =
{
    .semaphore_timeout  = 0xFFFFFFFF,
    .p_lower_level_cfg  = NULL,
    .p_extend           = (void*)&g_comms_uart0_extended_cfg,
    .p_callback         = NULL,
};

const rm_comms_instance_t g_comms_uart0 =
{
    .p_ctrl = &g_comms_uart0_ctrl,
    .p_cfg  = &g_comms_uart0_cfg,
    .p_api  = &g_comms_on_comms_uart,
};
/* I2C Communication Device */
rm_comms_i2c_instance_ctrl_t g_icp_comms_i2c_ctrl;

/* Lower level driver configuration */
const i2c_master_cfg_t g_icp_comms_i2c_lower_level_cfg =
{
    .slave = 0x63,
    .addr_mode = I2C_MASTER_ADDR_MODE_7BIT,
    .p_callback = rm_comms_i2c_callback,
};

const rm_comms_cfg_t g_icp_comms_i2c_cfg =
{
    .semaphore_timeout  = 500,
    .p_lower_level_cfg  = (void*)&g_icp_comms_i2c_lower_level_cfg,
    .p_extend           = (void*)&g_comms_i2c_bus0_extended_cfg,
    .p_callback         = icp_comms_i2c_callback,
#if defined(NULL)
    .p_context          = NULL,
#else
    .p_context          = (void*)&NULL,
#endif
};

const rm_comms_instance_t g_icp_comms_i2c =
{
    .p_ctrl = &g_icp_comms_i2c_ctrl,
    .p_cfg  = &g_icp_comms_i2c_cfg,
    .p_api  = &g_comms_on_comms_i2c,
};
/* I2C Communication Device */
rm_comms_i2c_instance_ctrl_t g_hs_comms_i2c_ctrl;

/* Lower level driver configuration */
const i2c_master_cfg_t g_hs_comms_i2c_lower_level_cfg =
{
    .slave = 0x44,
    .addr_mode = I2C_MASTER_ADDR_MODE_7BIT,
    .p_callback = rm_comms_i2c_callback,
};

const rm_comms_cfg_t g_hs_comms_i2c_cfg =
{
    .semaphore_timeout  = 500,
    .p_lower_level_cfg  = (void*)&g_hs_comms_i2c_lower_level_cfg,
    .p_extend           = (void*)&g_comms_i2c_bus0_extended_cfg,
    .p_callback         = hs_comms_i2c_callback,
#if defined(NULL)
    .p_context          = NULL,
#else
    .p_context          = (void*)&NULL,
#endif
};

const rm_comms_instance_t g_hs_comms_i2c =
{
    .p_ctrl = &g_hs_comms_i2c_ctrl,
    .p_cfg  = &g_hs_comms_i2c_cfg,
    .p_api  = &g_comms_on_comms_i2c,
};
extern uint32_t g_fsp_common_thread_count;

                const rm_freertos_port_parameters_t sensor_thread_parameters =
                {
                    .p_context = (void *) NULL,
                };

                void sensor_thread_create (void)
                {
                    /* Increment count so we will know the number of threads created in the RA Configuration editor. */
                    g_fsp_common_thread_count++;

                    /* Initialize each kernel object. */
                    

                    #if 1
                    sensor_thread = xTaskCreateStatic(
                    #else
                    BaseType_t sensor_thread_create_err = xTaskCreate(
                    #endif
                        sensor_thread_func,
                        (const char *)"Sensor Thread",
                        8192/4, // In words, not bytes
                        (void *) &sensor_thread_parameters, //pvParameters
                        4,
                        #if 1
                        (StackType_t *)&sensor_thread_stack,
                        (StaticTask_t *)&sensor_thread_memory
                        #else
                        & sensor_thread
                        #endif
                    );

                    #if 1
                    if (NULL == sensor_thread)
                    {
                        rtos_startup_err_callback(sensor_thread, 0);
                    }
                    #else
                    if (pdPASS != sensor_thread_create_err)
                    {
                        rtos_startup_err_callback(sensor_thread, 0);
                    }
                    #endif
                }
                static void sensor_thread_func (void * pvParameters)
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
                    sensor_thread_entry(pvParameters);
                }
