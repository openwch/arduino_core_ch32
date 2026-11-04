/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH585_SPI.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : head file(ch585/ch584)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __CH585_SPI_H__
#define __CH585_SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  SPI0 interrupt bit define
 */
#define SPI0_IT_FST_BYTE    RB_SPI_IE_FST_BYTE  // �ӻ�ģʽ�����ֽ�����ģʽ�£����յ����ֽ��ж�
#define SPI0_IT_FIFO_OV     RB_SPI_IE_FIFO_OV   // FIFO ���
#define SPI0_IT_DMA_END     RB_SPI_IE_DMA_END   // DMA �������
#define SPI0_IT_FIFO_HF     RB_SPI_IE_FIFO_HF   // FIFO ʹ�ù���
#define SPI0_IT_BYTE_END    RB_SPI_IE_BYTE_END  // ���ֽڴ������
#define SPI0_IT_CNT_END     RB_SPI_IE_CNT_END   // ȫ���ֽڴ������

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
 * @brief  Configuration SPI0 slave mode
 */
typedef enum
{
    Mode_DataStream = 0, // ������ģʽ
    Mose_FirstCmd,       // ���ֽ�����ģʽ
} Slave_ModeTypeDef;

/**
 * @brief   ����ģʽĬ�ϳ�ʼ����ģʽ0+3��ȫ˫��+8MHz
 */
void SPI0_MasterDefInit(void);

/**
 * @brief   SPI0 ��׼ʱ�����ã�= d*Tsys
 *
 * @param   c       - ʱ�ӷ�Ƶϵ��
 */
void SPI0_CLKCfg(uint8_t c);

/**
 * @brief   ����������ģʽ
 *
 * @param   m       - ������ģʽ refer to ModeBitOrderTypeDef
 */
void SPI0_DataMode(ModeBitOrderTypeDef m);

/**
 * @brief   ���͵��ֽ� (buffer)
 *
 * @param   d       - �����ֽ�
 */
void SPI0_MasterSendByte(uint8_t d);

/**
 * @brief   ���յ��ֽ� (buffer)
 *
 * @param   none
 */
uint8_t SPI0_MasterRecvByte(void);

/**
 * @brief   ʹ��FIFO�������Ͷ��ֽ�
 *
 * @param   pbuf    - �����͵����������׵�ַ
 * @param   len     - �����͵����ݳ��ȣ����4095
 */
void SPI0_MasterTrans(uint8_t *pbuf, uint16_t len);

/**
 * @brief   ʹ��FIFO�������ն��ֽ�
 *
 * @param   pbuf    - �����յ������׵�ַ
 * @param   len     - �����յ����ݳ��ȣ����4095
 */
void SPI0_MasterRecv(uint8_t *pbuf, uint16_t len);

/**
 * @brief   DMA��ʽ������������
 *
 * @param   pbuf    - ������������ʼ��ַ,��Ҫ���ֽڶ���
 * @param   len     - ���������ݳ���
 */
void SPI0_MasterDMATrans(uint8_t *pbuf, uint16_t len);

/**
 * @brief   DMA��ʽ������������
 *
 * @param   pbuf    - ���������ݴ����ʼ��ַ,��Ҫ���ֽڶ���
 * @param   len     - ���������ݳ���
 */
void SPI0_MasterDMARecv(uint8_t *pbuf, uint16_t len);

/**
 * @brief   ����ģʽĬ�ϳ�ʼ����ģʽ0+3��ȫ˫��+8MHz
 */
void SPI1_MasterDefInit(void);

/**
 * @brief   SPI1 ��׼ʱ�����ã�= d*Tsys
 *
 * @param   c       - ʱ�ӷ�Ƶϵ��
 */
void SPI1_CLKCfg(uint8_t c);

/**
 * @brief   ����������ģʽ
 *
 * @param   m       - ������ģʽ refer to ModeBitOrderTypeDef
 */
void SPI1_DataMode(ModeBitOrderTypeDef m);

/**
 * @brief   ���͵��ֽ� (buffer)
 *
 * @param   d       - �����ֽ�
 */
void  SPI1_MasterSendByte(uint8_t d);

/**
 * @brief   ���յ��ֽ� (buffer)
 *
 * @param   none
 */
uint8_t SPI1_MasterRecvByte(void);

/**
 * @brief   ʹ��FIFO�������Ͷ��ֽ�
 *
 * @param   pbuf    - �����͵����������׵�ַ
 * @param   len     - �����͵����ݳ��ȣ����4095
 */
void SPI1_MasterTrans(uint8_t *pbuf, uint16_t len);

/**
 * @brief   ʹ��FIFO�������ն��ֽ�
 *
 * @param   pbuf    - �����յ������׵�ַ
 * @param   len     - �����յ����ݳ��ȣ����4095
 */
void SPI1_MasterRecv(uint8_t *pbuf, uint16_t len);

/**
 * @brief   �豸ģʽĬ�ϳ�ʼ������������MISO��GPIO��ӦΪ����ģʽ
 */
void SPI0_SlaveInit(void);

/**
 * @brief   �������ֽ���������
 *
 * @param   d       - ���ֽ���������
 */
#define SetFirstData(d)    (R8_SPI0_SLAVE_PRE = d)

/**
 * @brief   �ӻ�ģʽ������һ�ֽ�����
 *
 * @param   d       - ����������
 */
void SPI0_SlaveSendByte(uint8_t d);

/**
 * @brief   �ӻ�ģʽ������һ�ֽ�����
 *
 * @return  ���յ�����
 */
uint8_t SPI0_SlaveRecvByte(void);

/**
 * @brief   �ӻ�ģʽ�����Ͷ��ֽ�����
 *
 * @param   pbuf    - �����͵����������׵�ַ
 * @param   len     - �����͵����ݳ��ȣ����4095
 */
void SPI0_SlaveTrans(uint8_t *pbuf, uint16_t len);

/**
 * @brief   �ӻ�ģʽ�����ն��ֽ�����
 *
 * @param   pbuf    - ���������ݴ����ʼ��ַ
 * @param   len     - ����������ݳ���
 */
void SPI0_SlaveRecv(uint8_t *pbuf, uint16_t len);

/**
 * @brief   DMA��ʽ������������
 *
 * @param   pbuf    - ������������ʼ��ַ,��Ҫ���ֽڶ���
 * @param   len     - ���������ݳ���
 */
void SPI0_SlaveDMATrans(uint8_t *pbuf, uint16_t len);

/**
 * @brief   DMA��ʽ������������
 *
 * @param   pbuf    - ���������ݴ����ʼ��ַ,��Ҫ���ֽڶ���
 * @param   len     - ���������ݳ���
 */
void SPI0_SlaveDMARecv(uint8_t *pbuf, uint16_t len);

/**
 * @brief   ����SPI0�ж�
 *
 * @param   s       - ʹ��/�ر�
 * @param   f       - refer to SPI0 interrupt bit define
 */
#define SPI0_ITCfg(s, f)       ((s) ? (R8_SPI0_INTER_EN |= f) : (R8_SPI0_INTER_EN &= ~f))

/**
 * @brief   ��ȡ�жϱ�־״̬��0-δ��λ��(!0)-����
 *
 * @param   f       - refer to SPI0 interrupt bit define
 */
#define SPI0_GetITFlag(f)      (R8_SPI0_INT_FLAG & f)

/**
 * @brief   �����ǰ�жϱ�־
 *
 * @param   f       - refer to SPI0 interrupt bit define
 */
#define SPI0_ClearITFlag(f)    (R8_SPI0_INT_FLAG = f)

/**
 * @brief   �ر�SPI0
 */
#define SPI0_Disable()         (R8_SPI0_CTRL_MOD &= ~(RB_SPI_MOSI_OE | RB_SPI_SCK_OE | RB_SPI_MISO_OE))

#ifdef __cplusplus
}
#endif

#endif // __CH585_SPI_H__
