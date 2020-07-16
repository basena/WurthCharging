/*
*********************************************************************************************************
*                                       uC/Probe Communication
*
*                         (c) Copyright 2007-2016; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                        COMMUNICATION: RS-232
*
*                                            PORT TEMPLATE
*
* Filename      : probe_rs232c.c
* Version       : V2.30
* Programmer(s) : BAN
*                 JFT
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#include  "probe_com.h"
#include  "probe_rs232.h"
#include  "ucprobe_conf.h"

#include  <xmc_gpio.h>
#include  <xmc_uart.h>

/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                               ENABLE
*
* Note(s) : (1) See 'probe_rs232.h  ENABLE'.
*********************************************************************************************************
*/

#if (PROBE_COM_CFG_RS232_EN == DEF_ENABLED)                     /* See Note #1.                                         */

/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                     LOCAL CONFIGURATION ERRORS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                       ProbeRS232_InitTarget()
*
* Description : Initialize the UART for Probe communication.
*
* Argument(s) : baud_rate   Intended baud rate of the RS-232.
*
* Return(s)   : none.
*
* Caller(s)   : ProbeRS232_Init().
*
* Note(s)     : (1) Though the baud rate used may vary from application to application or target to
*                   target, other communication settings are constamt.  The hardware must always be
*                   configured for the following :
*
*                   (a) No parity
*                   (b) One stop bit
*                   (c) Eight data bits.
*
*               (2) Neither receive nor transmit interrupts should be enabled by this function.
*********************************************************************************************************
*/

void  ProbeRS232_InitTarget (CPU_INT32U  baud_rate)
{
    XMC_UART_CH_CONFIG_t uart_config;
    XMC_GPIO_CONFIG_t    gpio_config;

    XMC_STRUCT_INIT(uart_config);

    uart_config.baudrate = baud_rate;
    uart_config.data_bits = UCPROBE_UART_DATA_BITS;
    uart_config.parity_mode = UCPROBE_UART_PARITY_MODE;
    uart_config.stop_bits = UCPROBE_UART_STOP_BITS;

    XMC_UART_CH_Init(UCPROBE_UART_CH, &uart_config);
    
	gpio_config.mode = XMC_GPIO_MODE_INPUT_TRISTATE;
    XMC_GPIO_Init(UCPROBE_UART_RX_PIN, &gpio_config);
	XMC_GPIO_Init(UCPROBE_UART_TX_PIN, &gpio_config);
	
	XMC_UART_CH_SetInputSource(UCPROBE_UART_CH, XMC_UART_CH_INPUT_RXD, UCPROBE_UART_RX_MUX);
    
    XMC_USIC_CH_SetInterruptNodePointer(UCPROBE_UART_CH, XMC_USIC_CH_INTERRUPT_NODE_POINTER_TRANSMIT_BUFFER, UCPROBE_UART_TX_SR);
    XMC_USIC_CH_SetInterruptNodePointer(UCPROBE_UART_CH, XMC_USIC_CH_INTERRUPT_NODE_POINTER_RECEIVE, UCPROBE_UART_RX_SR);
	XMC_USIC_CH_SetInterruptNodePointer(UCPROBE_UART_CH, XMC_USIC_CH_INTERRUPT_NODE_POINTER_ALTERNATE_RECEIVE, UCPROBE_UART_RX_SR);
	
#if (__CORTEX_M >= 0x03U) 
    NVIC_SetPriority(UCPROBE_UART_TX_IRQN, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), UCPROBE_UART_TX_PRIO, UCPROBE_UART_TX_SUBPRIO));
    NVIC_SetPriority(UCPROBE_UART_RX_IRQN, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), UCPROBE_UART_RX_PRIO, UCPROBE_UART_RX_SUBPRIO));
#else 
    NVIC_SetPriority(UCPROBE_UART_TX_IRQN, UCPROBE_UART_TX_PRIO);
    NVIC_SetPriority(UCPROBE_UART_RX_IRQN, UCPROBE_UART_RX_PRIO);
#endif

    NVIC_EnableIRQ(UCPROBE_UART_TX_IRQN);
    NVIC_EnableIRQ(UCPROBE_UART_RX_IRQN);

	XMC_UART_CH_Start(UCPROBE_UART_CH);

    gpio_config.mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL | UCPROBE_UART_TX_MUX;
    gpio_config.output_level = XMC_GPIO_OUTPUT_LEVEL_HIGH;
#if (__CORTEX_M >= 0x03U)
    gpio_config.output_strength = XMC_GPIO_OUTPUT_STRENGTH_STRONG_MEDIUM_EDGE;
#endif
    XMC_GPIO_Init(UCPROBE_UART_TX_PIN, &gpio_config);
}


/*
*********************************************************************************************************
*                                     ProbeRS232_RxTxISRHandler()
*
* Description : Handle receive and transmit interrupts.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : This is an ISR.
*
* Note(s)     : (1) This function may be empty if receive and transmit interrupts are handled in the
*                   separate interrupt handlers, ProbeRS232_RxISRHandler() and ProbeRS232_TxISRHandler().
*********************************************************************************************************
*/

void  ProbeRS232_RxTxISRHandler (void)
{

}


/*
*********************************************************************************************************
*                                      ProbeRS232_RxISRHandler()
*
* Description : Handle receive interrupts.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : This is an ISR.
*
* Note(s)     : (1) This function may be empty if receive interrupts are handled by the combined receive/
*                   transmit interrupt handler, 'ProbeRS232_RxTxISRHandler()'.
*********************************************************************************************************
*/

void  ProbeRS232_RxISRHandler (void)
{
    CPU_INT08U rx_data;


    rx_data = XMC_UART_CH_GetReceivedData(UCPROBE_UART_CH)  & 0xFF; /* Rd rx’d byte. */
    ProbeRS232_RxHandler(rx_data);
}


/*
*********************************************************************************************************
*                                        ProbeRS232_RxIntDis()
*
* Description : Disable receive interrupts.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : (1) The application should call this function to stop communication.
*********************************************************************************************************
*/

void  ProbeRS232_RxIntDis (void)
{
    XMC_UART_CH_DisableEvent(UCPROBE_UART_CH, XMC_UART_CH_EVENT_STANDARD_RECEIVE | XMC_UART_CH_EVENT_ALTERNATIVE_RECEIVE);
}


/*
*********************************************************************************************************
*                                        ProbeRS232_RxIntEn()
*
* Description : Enable receive interrupts.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : (1) The application should call this function to start communication.
*********************************************************************************************************
*/

void  ProbeRS232_RxIntEn (void)
{
    XMC_UART_CH_EnableEvent(UCPROBE_UART_CH, XMC_UART_CH_EVENT_STANDARD_RECEIVE | XMC_UART_CH_EVENT_ALTERNATIVE_RECEIVE);
}


/*
*********************************************************************************************************
*                                      ProbeRS232_TxISRHandler()
*
* Description : Handle transmit interrupts.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : This is an ISR.
*
* Note(s)     : (1) This function may be empty if transmit interrupts are handled by the combined receive/
*                   transmit interrupt handler, 'ProbeRS232_RxTxISRHandler()'.
*********************************************************************************************************
*/

void  ProbeRS232_TxISRHandler (void)
{
    ProbeRS232_TxHandler();
}


/*
*********************************************************************************************************
*                                          ProbeRS232_Tx1()
*
* Description : Transmit one byte.
*
* Argument(s) : c           The byte to transmit.
*
* Return(s)   : none.
*
* Caller(s)   : ProbeRS232_TxHandler().
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  ProbeRS232_Tx1 (CPU_INT08U c)
{
    XMC_UART_CH_Transmit(UCPROBE_UART_CH, c);
}


/*
*********************************************************************************************************
*                                        ProbeRS232_TxIntDis()
*
* Description : Disable transmit interrupts.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : ProbeRS232_TxHandler().
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  ProbeRS232_TxIntDis (void)
{
    XMC_UART_CH_DisableEvent(UCPROBE_UART_CH, XMC_UART_CH_EVENT_TRANSMIT_BUFFER);
}


/*
*********************************************************************************************************
*                                        ProbeRS232_TxIntEn()
*
* Description : Enable transmit interrupts.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : ProbeRS232_TxStart().
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  ProbeRS232_TxIntEn (void)
{
    XMC_UART_CH_EnableEvent(UCPROBE_UART_CH, XMC_UART_CH_EVENT_TRANSMIT_BUFFER);
}


/*
*********************************************************************************************************
*                                              ENABLE END
*
* Note(s) : See 'ENABLE  Note #1'.
*********************************************************************************************************
*/

#endif
