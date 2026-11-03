/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH572_SPI.c
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
 * @fn      SPI_MasterDefInit
 *
 * @brief   ����ģʽĬ�ϳ�ʼ����ģʽ0+3��ȫ˫��+8MHz
 *
 * @param   none
 *
 * @return  none
 */
void SPI_MasterDefInit(void)
{
    R8_SPI_CLOCK_DIV = 4; // ��Ƶʱ��4��Ƶ
    R8_SPI_CTRL_MOD = RB_SPI_ALL_CLEAR;
    R8_SPI_CTRL_MOD = RB_SPI_MOSI_OE | RB_SPI_SCK_OE;
    R8_SPI_CTRL_CFG |= RB_SPI_AUTO_IF;     // ����BUFFER/FIFO�Զ����IF_BYTE_END��־
    R8_SPI_CTRL_CFG &= ~RB_SPI_DMA_ENABLE; // ������DMA��ʽ
}

/*********************************************************************
 * @fn      SPI_2WIRE_MasterOutputInit
 *
 * @brief   ����2�߷���ģʽ��ʼ����ģʽ1+2�߰�˫��+8MHz
 *
 * @param   none
 *
 * @return  none
 */
void SPI_2WIRE_MasterOutputInit(void)
{
    R8_SPI_CLOCK_DIV = 4; // ��Ƶʱ��4��Ƶ
    R8_SPI_CTRL_MOD = RB_SPI_ALL_CLEAR;
    R8_SPI_CTRL_MOD =  RB_SPI_MOSI_OE | RB_SPI_SCK_OE | RB_SPI_2WIRE_MOD;  // ʹ������ģʽ
    R8_SPI_CTRL_CFG |= RB_SPI_AUTO_IF;     // ����BUFFER/FIFO�Զ����IF_BYTE_END��־
    R8_SPI_CTRL_CFG &= ~RB_SPI_DMA_ENABLE; // ������DMA��ʽ
}

/*********************************************************************
 * @fn      SPI_2WIRE_MasterReceiveInit
 *
 * @brief   ����2�߽���ģʽ��ʼ����ģʽ1+2�߰�˫��+8MHz
 *
 * @param   none
 *
 * @return  none
 */
void SPI_2WIRE_MasterReceiveInit(void)
{
    R8_SPI_CLOCK_DIV = 4; // ��Ƶʱ��4��Ƶ
    R8_SPI_CTRL_MOD = RB_SPI_ALL_CLEAR;
    R8_SPI_CTRL_MOD = RB_SPI_SCK_OE | RB_SPI_2WIRE_MOD;  // ʹ������ģʽ
    R8_SPI_CTRL_CFG |= RB_SPI_AUTO_IF;     // ����BUFFER/FIFO�Զ����IF_BYTE_END��־
    R8_SPI_CTRL_CFG &= ~RB_SPI_DMA_ENABLE; // ������DMA��ʽ
}

/*********************************************************************
 * @fn      SPI_2WIRE_SlaveInputInit
 *
 * @brief   �ӻ�2�߽���ģʽ��ʼ����ģʽ1+2�߰�˫��+8MHz
 *
 * @param   none
 *
 * @return  none
 */
void SPI_2WIRE_SlaveInputInit(void)
{
    R8_SPI_CTRL_MOD =  RB_SPI_ALL_CLEAR;
    R8_SPI_CTRL_MOD =  RB_SPI_2WIRE_MOD | RB_SPI_MODE_SLAVE;
    R8_SPI_CTRL_CFG |= RB_SPI_AUTO_IF | RB_MST_CLK_SEL;
}

/*********************************************************************
 * @fn      SPI_2WIRE_SlaveOutputInit
 *
 * @brief   �ӻ�2�߷���ģʽ��ʼ����ģʽ1+2�߰�˫��+8MHz
 *
 * @param   none
 *
 * @return  none
 */
void SPI_2WIRE_SlaveOutputInit(void)
{
    R8_SPI_CTRL_MOD =  RB_SPI_ALL_CLEAR;
    R8_SPI_CTRL_MOD =  RB_SPI_MISO_OE | RB_SPI_2WIRE_MOD | RB_SPI_MODE_SLAVE;
    R8_SPI_CTRL_CFG |= RB_SPI_AUTO_IF;
}

/*********************************************************************
 * @fn      SPI_CLKCfg
 *
 * @brief   SPI ��׼ʱ�����ã�= d*Tsys
 *
 * @param   c       - ʱ�ӷ�Ƶϵ��
 *
 * @return  none
 */
void SPI_CLKCfg(uint8_t c)
{
    if(c == 2)
    {
        R8_SPI_CTRL_CFG |= RB_SPI_MST_DLY_EN;
    }
    else
    {
        R8_SPI_CTRL_CFG &= ~RB_SPI_MST_DLY_EN;
    }
    R8_SPI_CLOCK_DIV = c;
}

/*********************************************************************
 * @fn      SPI_DataMode
 *
 * @brief   ����������ģʽ
 *
 * @param   m       - ������ģʽ refer to ModeBitOrderTypeDef
 *
 * @return  none
 */
void SPI_DataMode(ModeBitOrderTypeDef m)
{
    switch(m)
    {
        case Mode0_LowBitINFront:
            R8_SPI_CTRL_MOD &= ~RB_SPI_MST_SCK_MOD;
            R8_SPI_CTRL_CFG |= RB_SPI_BIT_ORDER;
            break;
        case Mode0_HighBitINFront:
            R8_SPI_CTRL_MOD &= ~RB_SPI_MST_SCK_MOD;
            R8_SPI_CTRL_CFG &= ~RB_SPI_BIT_ORDER;
            break;
        case Mode3_LowBitINFront:
            R8_SPI_CTRL_MOD |= RB_SPI_MST_SCK_MOD;
            R8_SPI_CTRL_CFG |= RB_SPI_BIT_ORDER;
            break;
        case Mode3_HighBitINFront:
            R8_SPI_CTRL_MOD |= RB_SPI_MST_SCK_MOD;
            R8_SPI_CTRL_CFG &= ~RB_SPI_BIT_ORDER;
            break;
        default:
            break;
    }
}

/*********************************************************************
 * @fn      SPI_MasterSendByte
 *
 * @brief   ���͵��ֽ� (buffer)
 *
 * @param   d       - �����ֽ�
 *
 * @return  none
 */
void SPI_MasterSendByte(uint8_t d)
{
    R8_SPI_CTRL_MOD &= ~RB_SPI_FIFO_DIR;
    R16_SPI_TOTAL_CNT = 1;
    R8_SPI_FIFO = d;
    while(!(R8_SPI_INT_FLAG & RB_SPI_FREE));
}

/*********************************************************************
 * @fn      SPI_MasterRecvByte
 *
 * @brief   ���յ��ֽ� (buffer)
 *
 * @param   none
 *
 * @return  ���յ����ֽ�
 */
uint8_t SPI_MasterRecvByte(void)
{
    R8_SPI_CTRL_MOD &= ~RB_SPI_FIFO_DIR;
    R8_SPI_BUFFER = 0xFF; // ��������
    while(!(R8_SPI_INT_FLAG & RB_SPI_FREE));
    return (R8_SPI_BUFFER);
}

/*********************************************************************
 * @fn      SPI_MasterTrans
 *
 * @brief   ʹ��FIFO�������Ͷ��ֽ�
 *
 * @param   pbuf    - �����͵����������׵�ַ
 * @param   len     - �����͵����ݳ��ȣ����4095
 *
 * @return  none
 */
void SPI_MasterTrans(uint8_t *pbuf, uint16_t len)
{
    uint16_t sendlen;

    sendlen = len;
    R8_SPI_CTRL_MOD &= ~RB_SPI_FIFO_DIR; // �������ݷ���Ϊ���
    R16_SPI_TOTAL_CNT = sendlen;         // ����Ҫ���͵����ݳ���
    R8_SPI_INT_FLAG = RB_SPI_IF_CNT_END;
    while(sendlen)
    {
        if(R8_SPI_FIFO_COUNT < SPI_FIFO_SIZE)
        {
            R8_SPI_FIFO = *pbuf;
            pbuf++;
            sendlen--;
        }
    }
    while(R8_SPI_FIFO_COUNT != 0); // �ȴ�FIFO�е�����ȫ���������
}

/*********************************************************************
 * @fn      SPI_MasterRecv
 *
 * @brief   ʹ��FIFO�������ն��ֽ�
 *
 * @param   pbuf    - �����յ������׵�ַ
 * @param   len     - �����յ����ݳ��ȣ����4095
 *
 * @return  none
 */
void SPI_MasterRecv(uint8_t *pbuf, uint16_t len)
{
    uint16_t readlen;

    readlen = len;
    R8_SPI_CTRL_MOD |= RB_SPI_FIFO_DIR; // �������ݷ���Ϊ����
    R16_SPI_TOTAL_CNT = len;            // ������Ҫ���յ����ݳ��ȣ�FIFO����Ϊ���볤�Ȳ�Ϊ0����������� */
    R8_SPI_INT_FLAG = RB_SPI_IF_CNT_END;
    while(readlen)
    {
        if(R8_SPI_FIFO_COUNT)
        {
            *pbuf = R8_SPI_FIFO;
            pbuf++;
            readlen--;
        }
    }
}

/*********************************************************************
 * @fn      SPI_MasterDMATrans
 *
 * @brief   DMA��ʽ������������
 *
 * @param   pbuf    - ������������ʼ��ַ,��Ҫ���ֽڶ���
 * @param   len     - ���������ݳ���
 *
 * @return  none
 */
void SPI_MasterDMATrans(uint8_t *pbuf, uint16_t len)
{
    R8_SPI_CTRL_MOD &= ~RB_SPI_FIFO_DIR;
    R16_SPI_DMA_BEG = (uint32_t)pbuf;
    R16_SPI_DMA_END = (uint32_t)(pbuf + len);
    R16_SPI_TOTAL_CNT = len;
    R8_SPI_INT_FLAG = RB_SPI_IF_CNT_END | RB_SPI_IF_DMA_END;
    R8_SPI_CTRL_CFG |= RB_SPI_DMA_ENABLE;
    while(!(R8_SPI_INT_FLAG & RB_SPI_IF_CNT_END));
    R8_SPI_CTRL_CFG &= ~RB_SPI_DMA_ENABLE;
}

/*********************************************************************
 * @fn      SPI_MasterDMARecv
 *
 * @brief   DMA��ʽ������������
 *
 * @param   pbuf    - ���������ݴ����ʼ��ַ,��Ҫ���ֽڶ���
 * @param   len     - ���������ݳ���
 *
 * @return  none
 */
void SPI_MasterDMARecv(uint8_t *pbuf, uint16_t len)
{
    R8_SPI_CTRL_MOD |= RB_SPI_FIFO_DIR;
    R16_SPI_DMA_BEG = (uint32_t)pbuf;
    R16_SPI_DMA_END = (uint32_t)(pbuf + len);
    R16_SPI_TOTAL_CNT = len;
    R8_SPI_INT_FLAG = RB_SPI_IF_CNT_END | RB_SPI_IF_DMA_END;
    R8_SPI_CTRL_CFG |= RB_SPI_DMA_ENABLE;
    while(!(R8_SPI_INT_FLAG & RB_SPI_IF_CNT_END));
    R8_SPI_CTRL_CFG &= ~RB_SPI_DMA_ENABLE;
}

/*********************************************************************
 * @fn      SPI_SlaveInit
 *
 * @brief   �豸ģʽĬ�ϳ�ʼ������������MISO��GPIO��ӦΪ����ģʽ
 *
 * @return  none
 */
void SPI_SlaveInit(void)
{
    R8_SPI_CTRL_MOD = RB_SPI_ALL_CLEAR;
    R8_SPI_CTRL_MOD = RB_SPI_MISO_OE | RB_SPI_MODE_SLAVE;
    R8_SPI_CTRL_CFG |= RB_SPI_AUTO_IF;
}

/*********************************************************************
 * @fn      SPI_2WIRE_SlaveInit
 *
 * @brief   �豸2��ģʽ��ʼ��
 *
 * @return  none
 */
void SPI_2WIRE_SlaveInit(void)
{
    R8_SPI_CTRL_MOD = RB_SPI_ALL_CLEAR;
    R8_SPI_CTRL_MOD = RB_SPI_MISO_OE | RB_SPI_2WIRE_MOD | RB_SPI_MODE_SLAVE;
    R8_SPI_CTRL_CFG |= RB_SPI_AUTO_IF;
}

/*********************************************************************
 * @fn      SPI_SlaveRecvByte
 *
 * @brief   �ӻ�ģʽ������һ�ֽ�����
 *
 * @return  ���յ�����
 */
uint8_t SPI_SlaveRecvByte(void)
{
    R8_SPI_CTRL_MOD |= RB_SPI_FIFO_DIR;
    while(R8_SPI_FIFO_COUNT == 0);
    return R8_SPI_FIFO;
}

/*********************************************************************
 * @fn      SPI_SlaveSendByte
 *
 * @brief   �ӻ�ģʽ������һ�ֽ�����
 *
 * @param   d       - ����������
 *
 * @return  none
 */
void SPI_SlaveSendByte(uint8_t d)
{
    R8_SPI_CTRL_MOD &= ~RB_SPI_FIFO_DIR;
    R16_SPI_TOTAL_CNT = 1;
    R8_SPI_FIFO = d;
    while(R8_SPI_FIFO_COUNT != 0); // �ȴ��������
}

/*********************************************************************
 * @fn      SPI_SlaveRecv
 *
 * @brief   �ӻ�ģʽ�����ն��ֽ�����
 *
 * @param   pbuf    - ���������ݴ����ʼ��ַ
 * @param   len     - ����������ݳ���
 *
 * @return  none
 */
__HIGH_CODE
void SPI_SlaveRecv(uint8_t *pbuf, uint16_t len)
{
    uint16_t revlen;

    revlen = len;
    R8_SPI_CTRL_MOD |= RB_SPI_FIFO_DIR;
    R8_SPI_INT_FLAG = RB_SPI_IF_CNT_END;
    while(revlen)
    {
        if(R8_SPI_FIFO_COUNT)
        {
            *pbuf = R8_SPI_FIFO;
            pbuf++;
            revlen--;
        }
    }
}

/*********************************************************************
 * @fn      SPI_SlaveTrans
 *
 * @brief   �ӻ�ģʽ�����Ͷ��ֽ�����
 *
 * @param   pbuf    - �����͵����������׵�ַ
 * @param   len     - �����͵����ݳ��ȣ����4095
 *
 * @return  none
 */
__HIGH_CODE
void SPI_SlaveTrans(uint8_t *pbuf, uint16_t len)
{
    uint16_t sendlen;

    sendlen = len;
    R8_SPI_CTRL_MOD &= ~RB_SPI_FIFO_DIR; // �������ݷ���Ϊ���
    R8_SPI_INT_FLAG = RB_SPI_IF_CNT_END;
    while(sendlen)
    {
        if(R8_SPI_FIFO_COUNT < SPI_FIFO_SIZE)
        {
            R8_SPI_FIFO = *pbuf;
            pbuf++;
            sendlen--;
        }
    }
    while(R8_SPI_FIFO_COUNT != 0); // �ȴ�FIFO�е�����ȫ���������
}

/*********************************************************************
 * @fn      SPI_SlaveDMARecv
 *
 * @brief   DMA��ʽ������������
 *
 * @param   pbuf    - ���������ݴ����ʼ��ַ,��Ҫ���ֽڶ���
 * @param   len     - ���������ݳ���
 *
 * @return  none
 */
void SPI_SlaveDMARecv(uint8_t *pbuf, uint16_t len)
{
    R8_SPI_CTRL_MOD |= RB_SPI_FIFO_DIR;
    R16_SPI_DMA_BEG = (uint32_t)pbuf;
    R16_SPI_DMA_END = (uint32_t)(pbuf + len);
    R16_SPI_TOTAL_CNT = len;
    R8_SPI_INT_FLAG = RB_SPI_IF_CNT_END | RB_SPI_IF_DMA_END;
    R8_SPI_CTRL_CFG |= RB_SPI_DMA_ENABLE;
    while(!(R8_SPI_INT_FLAG & RB_SPI_IF_CNT_END));
    R8_SPI_CTRL_CFG &= ~RB_SPI_DMA_ENABLE;
}

/*********************************************************************
 * @fn      SPI_SlaveDMATrans
 *
 * @brief   DMA��ʽ������������
 *
 * @param   pbuf    - ������������ʼ��ַ,��Ҫ���ֽڶ���
 * @param   len     - ���������ݳ���
 *
 * @return  none
 */
void SPI_SlaveDMATrans(uint8_t *pbuf, uint16_t len)
{
    R8_SPI_CTRL_MOD &= ~RB_SPI_FIFO_DIR;
    R16_SPI_DMA_BEG = (uint32_t)pbuf;
    R16_SPI_DMA_END = (uint32_t)(pbuf + len);
    R16_SPI_TOTAL_CNT = len;
    R8_SPI_INT_FLAG = RB_SPI_IF_CNT_END | RB_SPI_IF_DMA_END;
    R8_SPI_CTRL_CFG |= RB_SPI_DMA_ENABLE;
    while(!(R8_SPI_INT_FLAG & RB_SPI_IF_CNT_END));
    R8_SPI_CTRL_CFG &= ~RB_SPI_DMA_ENABLE;
}
