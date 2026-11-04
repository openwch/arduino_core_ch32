/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH572_uart.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : head file(ch572/ch570)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __CH572_UART_H__
#define __CH572_UART_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	LINE error and status define
 */
#define STA_ERR_BREAK     RB_LSR_BREAK_ERR    // ���ݼ������
#define STA_ERR_FRAME     RB_LSR_FRAME_ERR    // ����֡����
#define STA_ERR_PAR       RB_LSR_PAR_ERR      // ��żУ��λ����
#define STA_ERR_FIFOOV    RB_LSR_OVER_ERR     // �����������

#define STA_TXFIFO_EMP    RB_LSR_TX_FIFO_EMP  // ��ǰ����FIFO�գ����Լ�����䷢������
#define STA_TXALL_EMP     RB_LSR_TX_ALL_EMP   // ��ǰ���з������ݶ��������
#define STA_RECV_DATA     RB_LSR_DATA_RDY     // ��ǰ�н��յ�����

/**
 * @brief  Configuration UART TrigByte num
 */
typedef enum
{
    UART_1BYTE_TRIG = 0, // 1�ֽڴ���
    UART_2BYTE_TRIG,     // 2�ֽڴ���
    UART_4BYTE_TRIG,     // 4�ֽڴ���
    UART_7BYTE_TRIG,     // 7�ֽڴ���

} UARTByteTRIGTypeDef;

/**
 * @brief  UART Tx Pin Remap Def
 */
typedef enum
{
    UART_TX_REMAP_PA3 = 0, /*!<Ĭ��ӳ�䣨TXD/PA3�� */
    UART_TX_REMAP_PA2,     /*!<��ӳ�䣨TXD/PA2�� */
    UART_TX_REMAP_PA1,     /*!<��ӳ�䣨TXD/PA1�� */
    UART_TX_REMAP_PA0,     /*!<��ӳ�䣨TXD/PA0�� */
    UART_TX_REMAP_PA7,     /*!<��ӳ�䣨TXD/PA7�� */
    UART_TX_REMAP_PA8,     /*!<��ӳ�䣨TXD/PA8�� */
    UART_TX_REMAP_PA11,    /*!<��ӳ�䣨TXD/PA11�� */
    UART_TX_REMAP_PA10,    /*!<��ӳ�䣨TXD/PA10�� */
} UARTTxPinRemapDef;

/**
 * @brief  UART Rx Pin Remap Def
 */
typedef enum
{
    UART_RX_REMAP_PA2 = 0, /*!<Ĭ��ӳ�䣨RXD/PA2�� */
    UART_RX_REMAP_PA3,     /*!<��ӳ�䣨RXD/PA3�� */
    UART_RX_REMAP_PA0,     /*!<��ӳ�䣨RXD/PA0�� */
    UART_RX_REMAP_PA1,     /*!<��ӳ�䣨RXD/PA1�� */
    UART_RX_REMAP_PA6,     /*!<��ӳ�䣨RXD/PA6�� */
    UART_RX_REMAP_PA9,     /*!<��ӳ�䣨RXD/PA9�� */
    UART_RX_REMAP_PA10,    /*!<��ӳ�䣨RXD/PA10�� */
    UART_RX_REMAP_PA11,    /*!<��ӳ�䣨RXD/PA11�� */
} UARTRxPinRemapDef;

/**
 * @brief   ����Ĭ�ϳ�ʼ������
 */
void UART_DefInit(void);

/**
 * @brief   ���ڲ���������
 *
 * @param   baudrate    - ������
 */
void UART_BaudRateCfg(uint32_t baudrate);

/**
 * @brief   �����ֽڴ����ж�����
 *
 * @param   b       - �����ֽ��� refer to UARTByteTRIGTypeDef
 */
void UART_ByteTrigCfg(UARTByteTRIGTypeDef b);

/**
 * @brief   �����ж�����
 *
 * @param   s       - �жϿ���״̬���Ƿ�ʹ����Ӧ�ж�
 * @param   i       - �ж�����
 *                    RB_IER_MODEM_CHG  - ���ƽ��������״̬�仯�ж�ʹ��λ���� UART0 ֧�֣�
 *                    RB_IER_LINE_STAT  - ������·״̬�ж�
 *                    RB_IER_THR_EMPTY  - ���ͱ��ּĴ������ж�
 *                    RB_IER_RECV_RDY   - ���������ж�
 */
void UART_INTCfg(FunctionalState s, uint8_t i);

/**
 * @brief   �����ǰ����FIFO
 */
#define UART_CLR_RXFIFO()    (R8_UART_FCR |= RB_FCR_RX_FIFO_CLR)

/**
 * @brief   �����ǰ����FIFO
 */
#define UART_CLR_TXFIFO()    (R8_UART_FCR |= RB_FCR_TX_FIFO_CLR)

/**
 * @brief   ��ȡ��ǰ�жϱ�־
 *
 * @return  ��ǰ�жϱ�־
 */
#define UART_GetITFlag()     (R8_UART_IIR & RB_IIR_INT_MASK)

/**
 * @brief   ��ȡ��ǰͨѶ״̬
 *
 * @return  refer to LINE error and status define
 */
#define UART_GetLinSTA()     (R8_UART_LSR)

/**
 * @brief   ���ڵ��ֽڷ���
 *
 * @param   b       �����͵��ֽ�
 */
#define UART_SendByte(b)     (R8_UART_THR = b)

/**
 * @brief   ���ڶ��ֽڷ���
 *
 * @param   buf     - �����͵����������׵�ַ
 * @param   l       - �����͵����ݳ���
 */
void UART_SendString(uint8_t *buf, uint16_t l);

/**
 * @brief   ���ڶ�ȡ���ֽ�
 *
 * @return  ��ȡ���ĵ��ֽ�
 */
#define UART_RecvByte()    (R8_UART_RBR)

/**
 * @brief   ���ڶ�ȡ���ֽ�
 *
 * @param   buf     - ��ȡ���ݴ�Ż������׵�ַ
 *
 * @return  ��ȡ���ݳ���
 */
uint16_t UART_RecvString(uint8_t *buf);

/**
 * @brief   ��������ӳ��
 *
 * @param   s       - �Ƿ�ʹ��ӳ��
 * @param   perph   - дTx��Rx��ӳ���ϵ
 */
void UART_Remap(FunctionalState s, UARTTxPinRemapDef u_tx, UARTRxPinRemapDef u_rx);

#ifdef __cplusplus
}
#endif

#endif // __CH572_UART_H__
