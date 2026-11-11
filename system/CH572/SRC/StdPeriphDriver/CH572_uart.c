/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH572_UART.c
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : source file(ch572/ch570)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH572_common.h"

/*********************************************************************
 * @fn      UART_DefInit
 *
 * @brief   ����Ĭ�ϳ�ʼ������
 *
 * @param   none
 *
 * @return  none
 */
void UART_DefInit(void)
{
    UART_BaudRateCfg(115200);
    R8_UART_FCR = (2 << 6) | RB_FCR_FIFO_EN; // FIFO�򿪣�������4�ֽ�
    R8_UART_LCR = RB_LCR_WORD_SZ;
    R8_UART_IER = RB_IER_TXD_EN;
    R8_UART_DIV = 1;
}

/*********************************************************************
 * @fn      UART_BaudRateCfg
 *
 * @brief   ���ڲ���������
 *
 * @param   baudrate    - ������
 *
 * @return  none
 */
void UART_BaudRateCfg(uint32_t baudrate)
{
    uint32_t x;

    x = 10 * GetSysClock() / 8 / baudrate;
    x = (x + 5) / 10;
    R16_UART_DL = (uint16_t)x;
}

/*********************************************************************
 * @fn      UART_ByteTrigCfg
 *
 * @brief   �����ֽڴ����ж�����
 *
 * @param   b       - �����ֽ��� refer to UARTByteTRIGTypeDef
 *
 * @return  none
 */
void UART_ByteTrigCfg(UARTByteTRIGTypeDef b)
{
    R8_UART_FCR = (R8_UART_FCR & ~RB_FCR_FIFO_TRIG) | (b << 6);
}

/*********************************************************************
 * @fn      UART_INTCfg
 *
 * @brief   �����ж�����
 *
 * @param   s       - �жϿ���״̬���Ƿ�ʹ����Ӧ�ж�
 * @param   i       - �ж�����
 *                    RB_IER_MODEM_CHG  - ���ƽ��������״̬�仯�ж�ʹ��λ���� UART ֧�֣�
 *                    RB_IER_LINE_STAT  - ������·״̬�ж�
 *                    RB_IER_THR_EMPTY  - ���ͱ��ּĴ������ж�
 *                    RB_IER_RECV_RDY   - ���������ж�
 *
 * @return  none
 */
void UART_INTCfg(FunctionalState s, uint8_t i)
{
    if(s)
    {
        R8_UART_IER |= i;
        R8_UART_MCR |= RB_MCR_INT_OE;
    }
    else
    {
        R8_UART_IER &= ~i;
    }
}

/*********************************************************************
 * @fn      UART_SendString
 *
 * @brief   ���ڶ��ֽڷ���
 *
 * @param   buf     - �����͵����������׵�ַ
 * @param   l       - �����͵����ݳ���
 *
 * @return  none
 */
void UART_SendString(uint8_t *buf, uint16_t l)
{
    uint16_t len = l;

    while(len)
    {
        if(R8_UART_TFC != UART_FIFO_SIZE)
        {
            R8_UART_THR = *buf++;
            len--;
        }
    }
}

/*********************************************************************
 * @fn      UART_RecvString
 *
 * @brief   ���ڶ�ȡ���ֽ�
 *
 * @param   buf     - ��ȡ���ݴ�Ż������׵�ַ
 *
 * @return  ��ȡ���ݳ���
 */
uint16_t UART_RecvString(uint8_t *buf)
{
    uint16_t len = 0;

    while(R8_UART_RFC)
    {
        *buf++ = R8_UART_RBR;
        len++;
    }

    return (len);
}

/*********************************************************************
 * @fn      UART_Remap
 *
 * @brief   ����ӳ��
 *
 * @param   s       - �Ƿ�ʹ��ӳ��
 * @param   perph   - дTx��Rx��ӳ���ϵ
 *
 *
 * @return  none
 */
void UART_Remap(FunctionalState s, UARTTxPinRemapDef u_tx, UARTRxPinRemapDef u_rx)
{
    if(s)
    {
        R16_PIN_ALTERNATE_H &= ~(RB_UART_TXD | RB_UART_RXD); // �üĴ��������ϵ縴λ��ShutDown˯��ʱ��0
        R16_PIN_ALTERNATE_H |= (u_tx << 3) | u_rx;
    }
    else
    {
        R16_PIN_ALTERNATE_H &= ~(RB_UART_TXD | RB_UART_RXD);
    }
}
