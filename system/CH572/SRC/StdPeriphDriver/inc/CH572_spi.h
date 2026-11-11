/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH572_SPI.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : head file(ch572/ch570)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __CH572_SPI_H__
#define __CH572_SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  SPI interrupt bit define
 */
#define SPI_IT_FST_BYTE    RB_SPI_IE_FST_BYTE  // �ӻ�ģʽ�����ֽ�����ģʽ�£����յ����ֽ��ж�
#define SPI_IT_FIFO_OV     RB_SPI_IE_FIFO_OV   // FIFO ���
#define SPI_IT_DMA_END     RB_SPI_IE_DMA_END   // DMA �������
#define SPI_IT_FIFO_HF     RB_SPI_IE_FIFO_HF   // FIFO ʹ�ù���
#define SPI_IT_BYTE_END    RB_SPI_IE_BYTE_END  // ���ֽڴ������
#define SPI_IT_CNT_END     RB_SPI_IE_CNT_END   // ȫ���ֽڴ������

/**
 * @brief  Configuration data mode
 */
typedef enum
{
    Mode0_LowBitINFront = 0, // ģʽ0����λ��ǰ
    Mode0_HighBitINFront,    // ģʽ0����λ��ǰ
    Mode3_LowBitINFront,     // ģʽ3����λ��ǰ
    Mode3_HighBitINFront,    // ģʽ3����λ��ǰ
} ModeBitOrderTypeDef;

/**
 * @brief  Configuration SPI slave mode
 */
typedef enum
{
    Mode_DataStream = 0, // ������ģʽ
    Mose_FirstCmd,       // ���ֽ�����ģʽ
} Slave_ModeTypeDef;

/**
 * @brief   ����ģʽĬ�ϳ�ʼ����ģʽ0+3��ȫ˫��+8MHz
 */
void SPI_MasterDefInit(void);

/**
 * @brief   ����2�߷���ģʽ��ʼ����ģʽ1+2�߰�˫��+8MHz
 */
void SPI_2WIRE_MasterOutputInit(void);

/**
 * @brief   ����2�߽���ģʽ��ʼ����ģʽ1+2�߰�˫��+8MHz
 */
void SPI_2WIRE_MasterReceiveInit(void);

/**
 * @brief   �ӻ�2�߽���ģʽ��ʼ��
 */
void SPI_2WIRE_SlaveInputInit(void);

/**
 * @brief   �ӻ�2�߷���ģʽ��ʼ��
 */
void SPI_2WIRE_SlaveOutputInit(void);

/**
 * @brief   SPI ��׼ʱ�����ã�= d*Tsys
 *
 * @param   c       - ʱ�ӷ�Ƶϵ��
 */
void SPI_CLKCfg(uint8_t c);

/**
 * @brief   ����������ģʽ
 *
 * @param   m       - ������ģʽ refer to ModeBitOrderTypeDef
 */
void SPI_DataMode(ModeBitOrderTypeDef m);

/**
 * @brief   ���͵��ֽ� (buffer)
 *
 * @param   d       - �����ֽ�
 */
void SPI_MasterSendByte(uint8_t d);

/**
 * @brief   ���յ��ֽ� (buffer)
 *
 * @param   none
 */
uint8_t SPI_MasterRecvByte(void);

/**
 * @brief   ʹ��FIFO�������Ͷ��ֽ�
 *
 * @param   pbuf    - �����͵����������׵�ַ
 * @param   len     - �����͵����ݳ��ȣ����4095
 */
void SPI_MasterTrans(uint8_t *pbuf, uint16_t len);

/**
 * @brief   ʹ��FIFO�������ն��ֽ�
 *
 * @param   pbuf    - �����յ������׵�ַ
 * @param   len     - �����յ����ݳ��ȣ����4095
 */
void SPI_MasterRecv(uint8_t *pbuf, uint16_t len);

/**
 * @brief   DMA��ʽ������������
 *
 * @param   pbuf    - ������������ʼ��ַ,��Ҫ���ֽڶ���
 * @param   len     - ���������ݳ���
 */
void SPI_MasterDMATrans(uint8_t *pbuf, uint16_t len);

/**
 * @brief   DMA��ʽ������������
 *
 * @param   pbuf    - ���������ݴ����ʼ��ַ,��Ҫ���ֽڶ���
 * @param   len     - ���������ݳ���
 */
void SPI_MasterDMARecv(uint8_t *pbuf, uint16_t len);

/**
 * @brief   �������ֽ���������
 *
 * @param   d       - ���ֽ���������
 */
#define SetFirstData(d)    (R8_SPI_SLAVE_PRE = d)

/**
 * @brief   �ӻ�ģʽ��ʼ��
 */
void SPI_SlaveInit(void);

/**
 * @brief   �ӻ�2��ģʽ��ʼ��
 */
void SPI_2WIRE_SlaveInit(void);

/**
 * @brief   �ӻ�ģʽ������һ�ֽ�����
 *
 * @param   d       - ����������
 */
void SPI_SlaveSendByte(uint8_t d);

/**
 * @brief   �ӻ�ģʽ������һ�ֽ�����
 *
 * @return  ���յ�����
 */
uint8_t SPI_SlaveRecvByte(void);

/**
 * @brief   �ӻ�ģʽ�����Ͷ��ֽ�����
 *
 * @param   pbuf    - �����͵����������׵�ַ
 * @param   len     - �����͵����ݳ��ȣ����4095
 */
void SPI_SlaveTrans(uint8_t *pbuf, uint16_t len);

/**
 * @brief   �ӻ�ģʽ�����ն��ֽ�����
 *
 * @param   pbuf    - ���������ݴ����ʼ��ַ
 * @param   len     - ����������ݳ���
 */
void SPI_SlaveRecv(uint8_t *pbuf, uint16_t len);

/**
 * @brief   DMA��ʽ������������
 *
 * @param   pbuf    - ������������ʼ��ַ,��Ҫ���ֽڶ���
 * @param   len     - ���������ݳ���
 */
void SPI_SlaveDMATrans(uint8_t *pbuf, uint16_t len);

/**
 * @brief   DMA��ʽ������������
 *
 * @param   pbuf    - ���������ݴ����ʼ��ַ,��Ҫ���ֽڶ���
 * @param   len     - ���������ݳ���
 */
void SPI_SlaveDMARecv(uint8_t *pbuf, uint16_t len);

/**
 * @brief   ����SPI�ж�
 *
 * @param   s       - ʹ��/�ر�
 * @param   f       - refer to SPI interrupt bit define
 */
#define SPI_ITCfg(s, f)       ((s) ? (R8_SPI_INTER_EN |= f) : (R8_SPI_INTER_EN &= ~f))

/**
 * @brief   ��ȡ�жϱ�־״̬��0-δ��λ��(!0)-����
 *
 * @param   f       - refer to SPI interrupt bit define
 */
#define SPI_GetITFlag(f)      (R8_SPI_INT_FLAG & f)

/**
 * @brief   �����ǰ�жϱ�־
 *
 * @param   f       - refer to SPI interrupt bit define
 */
#define SPI_ClearITFlag(f)    (R8_SPI_INT_FLAG = f)

/**
 * @brief   �ر�SPI
 */
#define SPI_Disable()         (R8_SPI_CTRL_MOD &= ~(RB_SPI_MOSI_OE | RB_SPI_SCK_OE | RB_SPI_MISO_OE))

#ifdef __cplusplus
}
#endif

#endif // __CH572_SPI_H__
