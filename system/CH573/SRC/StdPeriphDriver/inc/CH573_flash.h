/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH573_flash.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __CH573_FLASH_H__
#define __CH573_FLASH_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   读取Flash-ROM
 *
 * @param   StartAddr   - read address
 * @param   Buffer      - read buffer
 * @param   len         - read len
 */
void FLASH_ROM_READ(uint32_t StartAddr, void *Buffer, uint32_t len);

void GET_UNIQUE_ID(uint8_t *Buffer);

#ifdef __cplusplus
}
#endif

#endif // __CH573_FLASH_H__
