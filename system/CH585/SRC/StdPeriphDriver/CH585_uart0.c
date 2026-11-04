/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH585_uart0.c
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : source file(ch585/ch584)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH585_common.h"

/*********************************************************************
 * @fn      UART0_DefInit
 *
 * @brief   ����Ĭ�ϳ�ʼ������
 *
 * @param   none
 *
 * @return  none
 */
void UART0_DefInit(void)
{
    UART0_BaudRateCfg(115200);
    R8_UART0_FCR = (2 << 6) | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN; // FIFO�򿪣�������4�ֽ�
    R8_UART0_LCR = RB_LCR_WORD_SZ;
    R8_UART0_IER = RB_IER_TXD_EN;
    R8_UART0_DIV = 1;
}

/*********************************************************************
 * @fn      UART0_BaudRateCfg
 *
 * @brief   ���ڲ���������
 *
 * @param   baudrate    - ������
 *
 * @return  none
 */
void UART0_BaudRateCfg(uint32_t baudrate)
{
    uint32_t x;

    x = 10 * GetSysClock() / 8 / baudrate;
    x = (x + 5) / 10;
    R16_UART0_DL = (uint16_t)x;
}

/*********************************************************************
 * @fn      UART0_ByteTrigCfg
 *
 * @brief   �����ֽڴ����ж�����
 *
 * @param   b       - �����ֽ��� refer to UARTByteTRIGTypeDef
 *
 * @return  none
 */
void UART0_ByteTrigCfg(UARTByteTRIGTypeDef b)
{
    R8_UART0_FCR = (R8_UART0_FCR & ~RB_FCR_FIFO_TRIG) | (b << 6);
}

/*********************************************************************
 * @fn      UART0_INTCfg
 *
 * @brief   �����ж�����
 *
 * @param   s       - �жϿ���״̬���Ƿ�ʹ����Ӧ�ж�
 * @param   i       - �ж�����
 *                    RB_IER_MODEM_CHG  - ���ƽ��������״̬�仯�ж�ʹ��λ���� UART0 ֧�֣�
 *                    RB_IER_LINE_STAT  - ������·״̬�ж�
 *                    RB_IER_THR_EMPTY  - ���ͱ��ּĴ������ж�
 *                    RB_IER_RECV_RDY   - ���������ж�
 *
 * @return  none
 */
void UART0_INTCfg(FunctionalState s, uint8_t i)
{
    if(s)
    {
        R8_UART0_IER |= i;
        R8_UART0_MCR |= RB_MCR_INT_OE;
    }
    else
    {
        R8_UART0_IER &= ~i;
    }
}

/*********************************************************************
 * @fn      UART0_Reset
 *
 * @brief   ����������λ
 *
 * @param   none
 *
 * @return  none
 */
void UART0_Reset(void)
{
    R8_UART0_IER = RB_IER_RESET;
}

/*********************************************************************
 * @fn      UART0_SendString
 *
 * @brief   ���ڶ��ֽڷ���
 *
 * @param   buf     - �����͵����������׵�ַ
 * @param   l       - �����͵����ݳ���
 *
 * @return  none
 */
void UART0_SendString(uint8_t *buf, uint16_t l)
{
    uint16_t len = l;

    while(len)
    {
        if(R8_UART0_TFC != UART_FIFO_SIZE)
        {
            R8_UART0_THR = *buf++;
            len--;
        }
    }
}

/*********************************************************************
 * @fn      UART0_RecvString
 *
 * @brief   ���ڶ�ȡ���ֽ�
 *
 * @param   buf     - ��ȡ���ݴ�Ż������׵�ַ
 *
 * @return  ��ȡ���ݳ���
 */
uint16_t UART0_RecvString(uint8_t *buf)
{
    uint16_t len = 0;

    while(R8_UART0_RFC)
    {
        *buf++ = R8_UART0_RBR;
        len++;
    }

    return (len);
}

