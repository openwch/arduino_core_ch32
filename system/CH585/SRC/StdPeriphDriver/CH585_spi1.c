/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH585_SPI1.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2018/12/15
 * Description        : source file(ch585/ch584)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH585_common.h"

/*********************************************************************
 * @fn      SPI1_MasterDefInit
 *
 * @brief   ����ģʽĬ�ϳ�ʼ����ģʽ0+3��ȫ˫��+8MHz
 *
 * @param   none
 *
 * @return  none
 */
void SPI1_MasterDefInit(void)
{
    R8_SPI1_CLOCK_DIV = 4; // ��Ƶʱ��4��Ƶ
    R8_SPI1_CTRL_MOD = RB_SPI_ALL_CLEAR;
    R8_SPI1_CTRL_MOD = RB_SPI_MOSI_OE | RB_SPI_SCK_OE;
    R8_SPI1_CTRL_CFG |= RB_SPI_AUTO_IF; // ����BUFFER/FIFO�Զ����IF_BYTE_END��־
}

/*********************************************************************
 * @fn      SPI1_CLKCfg
 *
 * @brief   SPI1 ��׼ʱ�����ã�= d*Tsys
 *
 * @param   c       - ʱ�ӷ�Ƶϵ��
 *
 * @return  none
 */
void SPI1_CLKCfg(uint8_t c)
{
    if(c == 2)
    {
        R8_SPI1_CTRL_CFG |= RB_SPI_MST_DLY_EN;
    }
    else
    {
        R8_SPI1_CTRL_CFG &= ~RB_SPI_MST_DLY_EN;
    }
    R8_SPI1_CLOCK_DIV = c;
}

/*********************************************************************
 * @fn      SPI1_DataMode
 *
 * @brief   ����������ģʽ
 *
 * @param   m       - ������ģʽ refer to ModeBitOrderTypeDef
 *
 * @return  none
 */
void SPI1_DataMode(ModeBitOrderTypeDef m)
{
    switch(m)
    {
        case Mode0_LowBitINFront:
            R8_SPI1_CTRL_MOD &= ~RB_SPI_MST_SCK_MOD;
            R8_SPI1_CTRL_CFG |= RB_SPI_BIT_ORDER;
            break;
        case Mode0_HighBitINFront:
            R8_SPI1_CTRL_MOD &= ~RB_SPI_MST_SCK_MOD;
            R8_SPI1_CTRL_CFG &= ~RB_SPI_BIT_ORDER;
            break;
        case Mode3_LowBitINFront:
            R8_SPI1_CTRL_MOD |= RB_SPI_MST_SCK_MOD;
            R8_SPI1_CTRL_CFG |= RB_SPI_BIT_ORDER;
            break;
        case Mode3_HighBitINFront:
            R8_SPI1_CTRL_MOD |= RB_SPI_MST_SCK_MOD;
            R8_SPI1_CTRL_CFG &= ~RB_SPI_BIT_ORDER;
            break;
        default:
            break;
    }
}

/*********************************************************************
 * @fn      SPI1_MasterSendByte
 *
 * @brief   ���͵��ֽ� (buffer)
 *
 * @param   d       - �����ֽ�
 *
 * @return  none
 */
void SPI1_MasterSendByte(uint8_t d)
{
    R8_SPI1_CTRL_MOD &= ~RB_SPI_FIFO_DIR;
    R16_SPI1_TOTAL_CNT = 1;         // ����Ҫ���͵����ݳ���
    R8_SPI1_FIFO = d;
    while(!(R8_SPI1_INT_FLAG & RB_SPI_FREE));
}

/*********************************************************************
 * @fn      SPI1_MasterRecvByte
 *
 * @brief   ���յ��ֽ� (buffer)
 *
 * @param   none
 *
 * @return  ���յ����ֽ�
 */
uint8_t SPI1_MasterRecvByte(void)
{
    R8_SPI1_CTRL_MOD |= RB_SPI_FIFO_DIR; // �������ݷ���Ϊ����
    R8_SPI1_BUFFER = 0xFF; // ��������
    while(!(R8_SPI1_INT_FLAG & RB_SPI_FREE));
    return (R8_SPI1_BUFFER);
}

/*********************************************************************
 * @fn      SPI1_MasterTrans
 *
 * @brief   ʹ��FIFO�������Ͷ��ֽ�
 *
 * @param   pbuf    - �����͵����������׵�ַ
 * @param   len     - �����͵����ݳ��ȣ����4095
 *
 * @return  none
 */
void SPI1_MasterTrans(uint8_t *pbuf, uint16_t len)
{
    uint16_t sendlen;

    sendlen = len;
    R8_SPI1_CTRL_MOD &= ~RB_SPI_FIFO_DIR; // �������ݷ���Ϊ���
    R16_SPI1_TOTAL_CNT = sendlen;         // ����Ҫ���͵����ݳ���
    R8_SPI1_INT_FLAG = RB_SPI_IF_CNT_END;
    while(sendlen)
    {
        if(R8_SPI1_FIFO_COUNT < SPI_FIFO_SIZE)
        {
            R8_SPI1_FIFO = *pbuf;
            pbuf++;
            sendlen--;
        }
    }
    while(R8_SPI1_FIFO_COUNT != 0); // �ȴ�FIFO�е�����ȫ���������
}

/*********************************************************************
 * @fn      SPI1_MasterRecv
 *
 * @brief   ʹ��FIFO�������ն��ֽ�
 *
 * @param   pbuf    - �����յ������׵�ַ
 * @param   len     - �����յ����ݳ��ȣ����4095
 *
 * @return  none
 */
void SPI1_MasterRecv(uint8_t *pbuf, uint16_t len)
{
    uint16_t readlen;

    readlen = len;
    R8_SPI1_CTRL_MOD |= RB_SPI_FIFO_DIR; // �������ݷ���Ϊ����
    R16_SPI1_TOTAL_CNT = len;            // ������Ҫ���յ����ݳ��ȣ�FIFO����Ϊ���볤�Ȳ�Ϊ0����������� */
    R8_SPI1_INT_FLAG = RB_SPI_IF_CNT_END;
    while(readlen)
    {
        if(R8_SPI1_FIFO_COUNT)
        {
            *pbuf = R8_SPI1_FIFO;
            pbuf++;
            readlen--;
        }
    }
}
