/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH585_uart.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : head file(ch585/ch584)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __CH585_UART_H__
#define __CH585_UART_H__

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
 * @brief   ����Ĭ�ϳ�ʼ������
 */
void UART0_DefInit(void);

/**
 * @brief   ���ڲ���������
 *
 * @param   baudrate    - ������
 */
void UART0_BaudRateCfg(uint32_t baudrate);

/**
 * @brief   �����ֽڴ����ж�����
 *
 * @param   b       - �����ֽ��� refer to UARTByteTRIGTypeDef
 */
void UART0_ByteTrigCfg(UARTByteTRIGTypeDef b);

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
void UART0_INTCfg(FunctionalState s, uint8_t i);

/**
 * @brief   ����������λ
 */
void UART0_Reset(void);

/**
 * @brief   �����ǰ����FIFO
 */
#define UART0_CLR_RXFIFO()    (R8_UART0_FCR |= RB_FCR_RX_FIFO_CLR)

/**
 * @brief   �����ǰ����FIFO
 */
#define UART0_CLR_TXFIFO()    (R8_UART0_FCR |= RB_FCR_TX_FIFO_CLR)

/**
 * @brief   ��ȡ��ǰ�жϱ�־
 *
 * @return  ��ǰ�жϱ�־
 */
#define UART0_GetITFlag()     (R8_UART0_IIR & RB_IIR_INT_MASK)

/**
 * @brief   ��ȡ��ǰͨѶ״̬
 *
 * @return  refer to LINE error and status define
 */
#define UART0_GetLinSTA()     (R8_UART0_LSR)

/**
 * @brief   ���ڵ��ֽڷ���
 *
 * @param   b       �����͵��ֽ�
 */
#define UART0_SendByte(b)     (R8_UART0_THR = b)

/**
 * @brief   ���ڶ��ֽڷ���
 *
 * @param   buf     - �����͵����������׵�ַ
 * @param   l       - �����͵����ݳ���
 */
void UART0_SendString(uint8_t *buf, uint16_t l);

/**
 * @brief   ���ڶ�ȡ���ֽ�
 *
 * @return  ��ȡ���ĵ��ֽ�
 */
#define UART0_RecvByte()    (R8_UART0_RBR)

/**
 * @brief   ���ڶ�ȡ���ֽ�
 *
 * @param   buf     - ��ȡ���ݴ�Ż������׵�ַ
 *
 * @return  ��ȡ���ݳ���
 */
uint16_t UART0_RecvString(uint8_t *buf);

/**
 * @brief   ����Ĭ�ϳ�ʼ������
 */
void UART1_DefInit(void);

/**
 * @brief   ���ڲ���������
 *
 * @param   baudrate    - ������
 */
void UART1_BaudRateCfg(uint32_t baudrate);

/**
 * @brief   �����ֽڴ����ж�����
 *
 * @param   b       - �����ֽ��� refer to UARTByteTRIGTypeDef
 */
void UART1_ByteTrigCfg(UARTByteTRIGTypeDef b);

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
void UART1_INTCfg(FunctionalState s, uint8_t i);

/**
 * @brief   ����������λ
 */
void UART1_Reset(void);

/**
 * @brief   �����ǰ����FIFO
 */
#define UART1_CLR_RXFIFO()    (R8_UART1_FCR |= RB_FCR_RX_FIFO_CLR)

/**
 * @brief   �����ǰ����FIFO
 */
#define UART1_CLR_TXFIFO()    (R8_UART1_FCR |= RB_FCR_TX_FIFO_CLR)

/**
 * @brief   ��ȡ��ǰ�жϱ�־
 *
 * @return  ��ǰ�жϱ�־
 */
#define UART1_GetITFlag()     (R8_UART1_IIR & RB_IIR_INT_MASK)

/**
 * @brief   ��ȡ��ǰͨѶ״̬
 *
 * @return  refer to LINE error and status define
 */
#define UART1_GetLinSTA()     (R8_UART1_LSR)

/**
 * @brief   ���ڵ��ֽڷ���
 *
 * @param   b       �����͵��ֽ�
 */
#define UART1_SendByte(b)     (R8_UART1_THR = b)

/**
 * @brief   ���ڶ��ֽڷ���
 *
 * @param   buf     - �����͵����������׵�ַ
 * @param   l       - �����͵����ݳ���
 */
void UART1_SendString(uint8_t *buf, uint16_t l);

/**
 * @brief   ���ڶ�ȡ���ֽ�
 *
 * @return  ��ȡ���ĵ��ֽ�
 */
#define UART1_RecvByte()    (R8_UART1_RBR)

/**
 * @brief   ���ڶ�ȡ���ֽ�
 *
 * @param   buf     - ��ȡ���ݴ�Ż������׵�ַ
 *
 * @return  ��ȡ���ݳ���
 */
uint16_t UART1_RecvString(uint8_t *buf);

/**
 * @brief   ����Ĭ�ϳ�ʼ������
 */
void UART2_DefInit(void);

/**
 * @brief   ���ڲ���������
 *
 * @param   baudrate    - ������
 */
void UART2_BaudRateCfg(uint32_t baudrate);

/**
 * @brief   �����ֽڴ����ж�����
 *
 * @param   b       - �����ֽ��� refer to UARTByteTRIGTypeDef
 */
void UART2_ByteTrigCfg(UARTByteTRIGTypeDef b);

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
void UART2_INTCfg(FunctionalState s, uint8_t i);

/**
 * @brief   ����������λ
 */
void UART2_Reset(void);

/**
 * @brief   �����ǰ����FIFO
 */
#define UART2_CLR_RXFIFO()    (R8_UART2_FCR |= RB_FCR_RX_FIFO_CLR)

/**
 * @brief   �����ǰ����FIFO
 */
#define UART2_CLR_TXFIFO()    (R8_UART2_FCR |= RB_FCR_TX_FIFO_CLR)

/**
 * @brief   ��ȡ��ǰ�жϱ�־
 *
 * @return  ��ǰ�жϱ�־
 */
#define UART2_GetITFlag()     (R8_UART2_IIR & RB_IIR_INT_MASK)

/**
 * @brief   ��ȡ��ǰͨѶ״̬
 *
 * @return  refer to LINE error and status define
 */
#define UART2_GetLinSTA()     (R8_UART2_LSR)

/**
 * @brief   ���ڵ��ֽڷ���
 *
 * @param   b       �����͵��ֽ�
 */
#define UART2_SendByte(b)     (R8_UART2_THR = b)

/**
 * @brief   ���ڶ��ֽڷ���
 *
 * @param   buf     - �����͵����������׵�ַ
 * @param   l       - �����͵����ݳ���
 */
void UART2_SendString(uint8_t *buf, uint16_t l);

/**
 * @brief   ���ڶ�ȡ���ֽ�
 *
 * @return  ��ȡ���ĵ��ֽ�
 */
#define UART2_RecvByte()    (R8_UART2_RBR)

/**
 * @brief   ���ڶ�ȡ���ֽ�
 *
 * @param   buf     - ��ȡ���ݴ�Ż������׵�ַ
 *
 * @return  ��ȡ���ݳ���
 */
uint16_t UART2_RecvString(uint8_t *buf);

/**
 * @brief   ����Ĭ�ϳ�ʼ������
 */
void UART3_DefInit(void);

/**
 * @brief   ���ڲ���������
 *
 * @param   baudrate    - ������
 */
void UART3_BaudRateCfg(uint32_t baudrate);

/**
 * @brief   �����ֽڴ����ж�����
 *
 * @param   b       - �����ֽ��� refer to UARTByteTRIGTypeDef
 */
void UART3_ByteTrigCfg(UARTByteTRIGTypeDef b);

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
void UART3_INTCfg(FunctionalState s, uint8_t i);

/**
 * @brief   ����������λ
 */
void UART3_Reset(void);

/**
 * @brief   �����ǰ����FIFO
 */
#define UART3_CLR_RXFIFO()    (R8_UART3_FCR |= RB_FCR_RX_FIFO_CLR)

/**
 * @brief   �����ǰ����FIFO
 */
#define UART3_CLR_TXFIFO()    (R8_UART3_FCR |= RB_FCR_TX_FIFO_CLR)

/**
 * @brief   ��ȡ��ǰ�жϱ�־
 *
 * @return  ��ǰ�жϱ�־
 */
#define UART3_GetITFlag()     (R8_UART3_IIR & RB_IIR_INT_MASK)

/**
 * @brief   ��ȡ��ǰͨѶ״̬
 *
 * @return  refer to LINE error and status define
 */
#define UART3_GetLinSTA()     (R8_UART3_LSR)

/**
 * @brief   ���ڵ��ֽڷ���
 *
 * @param   b       �����͵��ֽ�
 */
#define UART3_SendByte(b)     (R8_UART3_THR = b)

/**
 * @brief   ���ڶ��ֽڷ���
 *
 * @param   buf     - �����͵����������׵�ַ
 * @param   l       - �����͵����ݳ���
 */
void UART3_SendString(uint8_t *buf, uint16_t l);

/**
 * @brief   ���ڶ�ȡ���ֽ�
 *
 * @return  ��ȡ���ĵ��ֽ�
 */
#define UART3_RecvByte()    (R8_UART3_RBR)

/**
 * @brief   ���ڶ�ȡ���ֽ�
 *
 * @param   buf     - ��ȡ���ݴ�Ż������׵�ַ
 *
 * @return  ��ȡ���ݳ���
 */
uint16_t UART3_RecvString(uint8_t *buf);

#ifdef __cplusplus
}
#endif

#endif // __CH585_UART_H__
