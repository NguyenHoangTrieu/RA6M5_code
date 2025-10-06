/* generated vector header file - do not edit */
#ifndef VECTOR_DATA_H
#define VECTOR_DATA_H
#ifdef __cplusplus
extern "C" {
#endif
/* Number of interrupts allocated */
#ifndef VECTOR_DATA_IRQ_COUNT
#define VECTOR_DATA_IRQ_COUNT (10)
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
void rtc_carry_isr(void);
void ether_eint_isr(void);

/* Vector table allocations */
#define VECTOR_NUMBER_IIC0_RXI ((IRQn_Type)0) /* IIC0 RXI (Receive data full)  \
                                               */
#define IIC0_RXI_IRQn ((IRQn_Type)0)          /* IIC0 RXI (Receive data full) */
#define VECTOR_NUMBER_IIC0_TXI                                                 \
  ((IRQn_Type)1)                     /* IIC0 TXI (Transmit data empty) */
#define IIC0_TXI_IRQn ((IRQn_Type)1) /* IIC0 TXI (Transmit data empty) */
#define VECTOR_NUMBER_IIC0_TEI ((IRQn_Type)2) /* IIC0 TEI (Transmit end) */
#define IIC0_TEI_IRQn ((IRQn_Type)2)          /* IIC0 TEI (Transmit end) */
#define VECTOR_NUMBER_IIC0_ERI ((IRQn_Type)3) /* IIC0 ERI (Transfer error) */
#define IIC0_ERI_IRQn ((IRQn_Type)3)          /* IIC0 ERI (Transfer error) */
#define VECTOR_NUMBER_SCI5_RXI ((IRQn_Type)4) /* SCI5 RXI (Receive data full)  \
                                               */
#define SCI5_RXI_IRQn ((IRQn_Type)4)          /* SCI5 RXI (Receive data full) */
#define VECTOR_NUMBER_SCI5_TXI                                                 \
  ((IRQn_Type)5)                     /* SCI5 TXI (Transmit data empty) */
#define SCI5_TXI_IRQn ((IRQn_Type)5) /* SCI5 TXI (Transmit data empty) */
#define VECTOR_NUMBER_SCI5_TEI ((IRQn_Type)6)  /* SCI5 TEI (Transmit end) */
#define SCI5_TEI_IRQn ((IRQn_Type)6)           /* SCI5 TEI (Transmit end) */
#define VECTOR_NUMBER_SCI5_ERI ((IRQn_Type)7)  /* SCI5 ERI (Receive error) */
#define SCI5_ERI_IRQn ((IRQn_Type)7)           /* SCI5 ERI (Receive error) */
#define VECTOR_NUMBER_RTC_CARRY ((IRQn_Type)8) /* RTC CARRY (Carry interrupt)  \
                                                */
#define RTC_CARRY_IRQn ((IRQn_Type)8)          /* RTC CARRY (Carry interrupt) */
#define VECTOR_NUMBER_EDMAC0_EINT                                              \
  ((IRQn_Type)9)                        /* EDMAC0 EINT (EDMAC 0 interrupt) */
#define EDMAC0_EINT_IRQn ((IRQn_Type)9) /* EDMAC0 EINT (EDMAC 0 interrupt) */
/* The number of entries required for the ICU vector table. */
#define BSP_ICU_VECTOR_NUM_ENTRIES (10)

#ifdef __cplusplus
}
#endif
#endif /* VECTOR_DATA_H */