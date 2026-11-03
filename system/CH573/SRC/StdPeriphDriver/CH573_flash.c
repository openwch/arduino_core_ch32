/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH573_flash.c
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH573_common.h"

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
    FLASH_EEPROM_CMD( CMD_GET_ROM_INFO, ROM_CFG_MAC_ADDR, Buffer, 0 );
    Buffer[6] = 0;
    Buffer[7] = 0;
}

