#if defined(CH32X035)

#include "wch_cdc_uart.h"

volatile UART_CTL Uart;
__attribute__((aligned(4))) uint8_t UART2_Tx_Buf[DEF_UARTx_TX_BUF_LEN];

void UART2_ParaInit(uint8_t mode)
{
    memset((void *)&Uart, 0, sizeof(Uart));
    if (mode) {
        Uart.Com_Cfg[0] = 0x00;
        Uart.Com_Cfg[1] = 0xc2;
        Uart.Com_Cfg[2] = 0x01;
        Uart.Com_Cfg[6] = 8;
    }
}

void UART2_USB_Init(void)
{
}

#endif
