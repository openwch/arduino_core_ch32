/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH572_flash.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : head file(ch572/ch570)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __CH572_FLASH_H__
#define __CH572_FLASH_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   ��ȡFlash-ROM
 *
 * @param   StartAddr   - read address
 * @param   Buffer      - read buffer
 * @param   len         - read len
 */
void FLASH_ROM_READ(uint32_t StartAddr, void *Buffer, uint32_t len);

uint8_t UserOptionByteConfig(FunctionalState RESET_EN, FunctionalState UART_NO_KEY_EN,
        uint32_t FLASHProt_Size);

uint8_t UserOptionByteClose_SWD(void);

void UserOptionByte_Active(void);

void GET_UNIQUE_ID(uint8_t *Buffer);
#ifdef __cplusplus
}
#endif

#endif // __CH572_FLASH_H__
