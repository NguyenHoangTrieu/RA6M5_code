/* generated vector header file - do not edit */
        #ifndef VECTOR_DATA_H
        #define VECTOR_DATA_H
        #ifdef __cplusplus
        extern "C" {
        #endif
                /* Number of interrupts allocated */
        #ifndef VECTOR_DATA_IRQ_COUNT
        #define VECTOR_DATA_IRQ_COUNT    (13)
        #endif
        /* ISR prototypes */
        void iic_master_rxi_isr(void);
        void iic_master_txi_isr(void);
        void iic_master_tei_isr(void);
        void iic_master_eri_isr(void);
        void sci_uart_rxi_isr(void);
        void sci_uart_txi_isr(void);
        void sci_uart_tei_isr(void);
        void sci_uart_eri_isr(void);
        void ether_eint_isr(void);

        /* Vector table allocations */
        #define VECTOR_NUMBER_IIC0_RXI ((IRQn_Type) 0) /* IIC0 RXI (Receive data full) */
        #define IIC0_RXI_IRQn          ((IRQn_Type) 0) /* IIC0 RXI (Receive data full) */
        #define VECTOR_NUMBER_IIC0_TXI ((IRQn_Type) 1) /* IIC0 TXI (Transmit data empty) */
        #define IIC0_TXI_IRQn          ((IRQn_Type) 1) /* IIC0 TXI (Transmit data empty) */
        #define VECTOR_NUMBER_IIC0_TEI ((IRQn_Type) 2) /* IIC0 TEI (Transmit end) */
        #define IIC0_TEI_IRQn          ((IRQn_Type) 2) /* IIC0 TEI (Transmit end) */
        #define VECTOR_NUMBER_IIC0_ERI ((IRQn_Type) 3) /* IIC0 ERI (Transfer error) */
        #define IIC0_ERI_IRQn          ((IRQn_Type) 3) /* IIC0 ERI (Transfer error) */
        #define VECTOR_NUMBER_SCI3_RXI ((IRQn_Type) 4) /* SCI3 RXI (Receive data full) */
        #define SCI3_RXI_IRQn          ((IRQn_Type) 4) /* SCI3 RXI (Receive data full) */
        #define VECTOR_NUMBER_SCI3_TXI ((IRQn_Type) 5) /* SCI3 TXI (Transmit data empty) */
        #define SCI3_TXI_IRQn          ((IRQn_Type) 5) /* SCI3 TXI (Transmit data empty) */
        #define VECTOR_NUMBER_SCI3_TEI ((IRQn_Type) 6) /* SCI3 TEI (Transmit end) */
        #define SCI3_TEI_IRQn          ((IRQn_Type) 6) /* SCI3 TEI (Transmit end) */
        #define VECTOR_NUMBER_SCI3_ERI ((IRQn_Type) 7) /* SCI3 ERI (Receive error) */
        #define SCI3_ERI_IRQn          ((IRQn_Type) 7) /* SCI3 ERI (Receive error) */
        #define VECTOR_NUMBER_EDMAC0_EINT ((IRQn_Type) 8) /* EDMAC0 EINT (EDMAC 0 interrupt) */
        #define EDMAC0_EINT_IRQn          ((IRQn_Type) 8) /* EDMAC0 EINT (EDMAC 0 interrupt) */
        #define VECTOR_NUMBER_IIC1_RXI ((IRQn_Type) 9) /* IIC1 RXI (Receive data full) */
        #define IIC1_RXI_IRQn          ((IRQn_Type) 9) /* IIC1 RXI (Receive data full) */
        #define VECTOR_NUMBER_IIC1_TXI ((IRQn_Type) 10) /* IIC1 TXI (Transmit data empty) */
        #define IIC1_TXI_IRQn          ((IRQn_Type) 10) /* IIC1 TXI (Transmit data empty) */
        #define VECTOR_NUMBER_IIC1_TEI ((IRQn_Type) 11) /* IIC1 TEI (Transmit end) */
        #define IIC1_TEI_IRQn          ((IRQn_Type) 11) /* IIC1 TEI (Transmit end) */
        #define VECTOR_NUMBER_IIC1_ERI ((IRQn_Type) 12) /* IIC1 ERI (Transfer error) */
        #define IIC1_ERI_IRQn          ((IRQn_Type) 12) /* IIC1 ERI (Transfer error) */
        /* The number of entries required for the ICU vector table. */
        #define BSP_ICU_VECTOR_NUM_ENTRIES (13)

        #ifdef __cplusplus
        }
        #endif
        #endif /* VECTOR_DATA_H */