/* generated vector source file - do not edit */
        #include "bsp_api.h"
        /* Do not build these data structures if no interrupts are currently allocated because IAR will have build errors. */
        #if VECTOR_DATA_IRQ_COUNT > 0
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_NUM_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
                        [0] = iic_master_rxi_isr, /* IIC0 RXI (Receive data full) */
            [1] = iic_master_txi_isr, /* IIC0 TXI (Transmit data empty) */
            [2] = iic_master_tei_isr, /* IIC0 TEI (Transmit end) */
            [3] = iic_master_eri_isr, /* IIC0 ERI (Transfer error) */
            [4] = sci_uart_rxi_isr, /* SCI3 RXI (Receive data full) */
            [5] = sci_uart_txi_isr, /* SCI3 TXI (Transmit data empty) */
            [6] = sci_uart_tei_isr, /* SCI3 TEI (Transmit end) */
            [7] = sci_uart_eri_isr, /* SCI3 ERI (Receive error) */
            [8] = ether_eint_isr, /* EDMAC0 EINT (EDMAC 0 interrupt) */
            [9] = iic_master_rxi_isr, /* IIC1 RXI (Receive data full) */
            [10] = iic_master_txi_isr, /* IIC1 TXI (Transmit data empty) */
            [11] = iic_master_tei_isr, /* IIC1 TEI (Transmit end) */
            [12] = iic_master_eri_isr, /* IIC1 ERI (Transfer error) */
        };
        #if BSP_FEATURE_ICU_HAS_IELSR
        const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_NUM_ENTRIES] =
        {
            [0] = BSP_PRV_VECT_ENUM(EVENT_IIC0_RXI,GROUP0), /* IIC0 RXI (Receive data full) */
            [1] = BSP_PRV_VECT_ENUM(EVENT_IIC0_TXI,GROUP1), /* IIC0 TXI (Transmit data empty) */
            [2] = BSP_PRV_VECT_ENUM(EVENT_IIC0_TEI,GROUP2), /* IIC0 TEI (Transmit end) */
            [3] = BSP_PRV_VECT_ENUM(EVENT_IIC0_ERI,GROUP3), /* IIC0 ERI (Transfer error) */
            [4] = BSP_PRV_VECT_ENUM(EVENT_SCI3_RXI,GROUP4), /* SCI3 RXI (Receive data full) */
            [5] = BSP_PRV_VECT_ENUM(EVENT_SCI3_TXI,GROUP5), /* SCI3 TXI (Transmit data empty) */
            [6] = BSP_PRV_VECT_ENUM(EVENT_SCI3_TEI,GROUP6), /* SCI3 TEI (Transmit end) */
            [7] = BSP_PRV_VECT_ENUM(EVENT_SCI3_ERI,GROUP7), /* SCI3 ERI (Receive error) */
            [8] = BSP_PRV_VECT_ENUM(EVENT_EDMAC0_EINT,GROUP0), /* EDMAC0 EINT (EDMAC 0 interrupt) */
            [9] = BSP_PRV_VECT_ENUM(EVENT_IIC1_RXI,GROUP1), /* IIC1 RXI (Receive data full) */
            [10] = BSP_PRV_VECT_ENUM(EVENT_IIC1_TXI,GROUP2), /* IIC1 TXI (Transmit data empty) */
            [11] = BSP_PRV_VECT_ENUM(EVENT_IIC1_TEI,GROUP3), /* IIC1 TEI (Transmit end) */
            [12] = BSP_PRV_VECT_ENUM(EVENT_IIC1_ERI,GROUP4), /* IIC1 ERI (Transfer error) */
        };
        #endif
        #endif