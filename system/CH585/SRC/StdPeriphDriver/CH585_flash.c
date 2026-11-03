/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH585_flash.c
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

/* RESET_EN */
#define RESET_Enable           0x00000008
#define RESET_Disable          0xFFFFFFF7

/* LOCKUP_RST_EN */
#define UART_NO_KEY_Enable     0x00000100
#define UART_NO_KEY_Disable    0xFFFFFEFF

/* BOOT_PIN */
#define BOOT_PIN_PB22          0x00000200
#define BOOT_PIN_PB11          0xFFFFFDFF

/* FLASH_WRProt */
#define FLASH_WRProt           0xFFF003FF

/*********************************************************************
 * @fn      FLASH_ROM_READ
 *
 * @brief   Read Flash
 *
 * @param   StartAddr   - read address
 * @param   Buffer      - read buffer
 * @param   len         - read len
 *
 * @return  none
 */
void FLASH_ROM_READ(uint32_t StartAddr, void *Buffer, uint32_t len)
{
    uint32_t  i, Length = (len + 3) >> 2;
    uint32_t *pCode = (uint32_t *)StartAddr;
    uint32_t *pBuf = (uint32_t *)Buffer;

    for(i = 0; i < Length; i++)
    {
        *pBuf++ = *pCode++;
    }
}

/*********************************************************************
 * @fn      UserOptionByteConfig
 *
 * @brief   Configure User Option Byte.���ڵ����û���������Ч��������Ч,��ÿ����¼��ֻ���޸�һ��
 *          (ʹ�øú���������ʹ�ùٷ��ṩ��.S�ļ���ͬʱ���øú����������ϵ�����ߵ��Խӿ�Ĭ�Ϲر�)
 *
 * @param   RESET_EN        - �ⲿ��λ����ʹ��
 * @param   BOOT_PIN        - ENABLE-ʹ��Ĭ��boot��-PB22,DISABLE-ʹ��boot��-PB11
 * @param   UART_NO_KEY_EN  - �����ⰴ������ʹ��
 * @param   FLASHProt_Size  - д������С(��λ4K)
 *
 * @return  0-Success, 1-Err
 */
uint8_t UserOptionByteConfig(FunctionalState RESET_EN, FunctionalState BOOT_PIN, FunctionalState UART_NO_KEY_EN,
                           uint32_t FLASHProt_Size)
{
    uint32_t s, t;

    FLASH_ROM_READ(0x14, &s, 4);

    if(s == 0xF5F9BDA9)
    {
        s = 0;
        FLASH_EEPROM_CMD(CMD_GET_ROM_INFO, 0x7EFFC, &s, 4);
        s &= 0xFF;

        if(RESET_EN == ENABLE)
            s |= RESET_Enable;
        else
            s &= RESET_Disable;

        /* bit[7:0]-bit[31-24] */
        s |= ((~(s << 24)) & 0xFF000000); //��8λ ������Ϣȡ����

        if(BOOT_PIN == ENABLE)
            s |= BOOT_PIN_PB22;
        if(UART_NO_KEY_EN == ENABLE)
            s |= UART_NO_KEY_Enable;

        /* bit[23-10] */
        s &= 0xFF0003FF;
        s |= ((FLASHProt_Size << 10) | (5 << 20)) & 0x00FFFC00;

        /*Write user option byte*/
        FLASH_ROM_WRITE(0x14, &s, 4);

        /* Verify user option byte */
        FLASH_ROM_READ(0x14, &t, 4);

        if(s == t)
            return 0;
        else
            return 1;
    }

    return 1;
}

/*********************************************************************
 * @fn      UserOptionByteClose_SWD
 *
 * @brief   �����ߵ��Խӿڣ���������ֵ���ֲ���.���ڵ����û���������Ч��������Ч,��ÿ����¼��ֻ���޸�һ��
 *          (ʹ�øú���������ʹ�ùٷ��ṩ��.S�ļ���ͬʱ���øú����������ϵ�����ߵ��Խӿ�Ĭ�Ϲر�)
 *
 * @return  0-Success, 1-Err
 */
uint8_t UserOptionByteClose_SWD(void)
{
    uint32_t s, t;

    FLASH_ROM_READ(0x14, &s, 4);

    if(s == 0xF3F9BDA9)
    {
        FLASH_EEPROM_CMD(CMD_GET_ROM_INFO, 0x7EFFC, &s, 4);

        s &= ~((1 << 4) | (1 << 7)); //���õ��Թ��ܣ� ����SPI��дFLASH

        /* bit[7:0]-bit[31-24] */
        s &= 0x00FFFFFF;
        s |= ((~(s << 24)) & 0xFF000000); //��8λ ������Ϣȡ����

        /*Write user option byte*/
        FLASH_ROM_WRITE(0x14, &s, 4);

        /* Verify user option byte */
        FLASH_ROM_READ(0x14, &t, 4);

        if(s == t)
            return 0;
        else
            return 1;
    }

    return 1;
}

/*********************************************************************
 * @fn      UserOptionByte_Active
 *
 * @brief   �û���������Ч������ִ�к��Զ���λ
 *
 * @return  0-Success, 1-Err
 */
void UserOptionByte_Active(void)
{
    FLASH_ROM_SW_RESET();
    sys_safe_access_enable();
    R16_INT32K_TUNE = 0xFFFF;
    sys_safe_access_disable();
    sys_safe_access_enable();
    R8_RST_WDOG_CTRL |= RB_SOFTWARE_RESET;
    sys_safe_access_disable();
    while(1);
}

/*********************************************************************
 * @fn      GET_UNIQUE_ID
 *
 * @brief   get 64 bit unique ID
 *
 * @param   Buffer      - Pointer to the buffer where data should be stored, Must be aligned to 4 bytes.
 *
 * @return  0-SUCCESS  (!0)-FAILURE
 */
void GET_UNIQUE_ID(uint8_t *Buffer)
{
    uint16_t temp;
    FLASH_EEPROM_CMD( CMD_GET_ROM_INFO, ROM_CFG_MAC_ADDR, Buffer, 0 );
    temp = (Buffer[0]|(Buffer[1]<<8)) + (Buffer[2]|(Buffer[3]<<8)) + (Buffer[4]|(Buffer[5]<<8));
    Buffer[6] = temp&0xFF;
    Buffer[7] = (temp>>8)&0xFF;
}
