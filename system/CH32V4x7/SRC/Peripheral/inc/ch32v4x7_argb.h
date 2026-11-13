/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32v4x7_argb.h
* Author             : WCH
* Version            : V1.0.1
* Date               : 2026/04/08
* Description        : This file contains all the functions prototypes for the 
*                      ARGB firmware library.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32V4x7_ARGB_H
#define __CH32V4x7_ARGB_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "ch32v4x7.h"

/* ARGB mode enumeration */
typedef enum
{
    ARGB_Mode_SendRSTFirst = 0,
    ARGB_Mode_SendDATAFirst = 0x4
} ARGB_Mode_Typedef;

/* ARGB data endianness enumeration */
typedef enum
{
    ARGB_Endian_MSB = 0,
    ARGB_Endian_LSB = 0x8
} ARGB_Endian_Typedef;

/* ARGB interrupt register enumeration */
typedef enum
{
    ARGB_IT_FRH = 0x00000100,
    ARGB_IT_RST = 0x00000200,
    ARGB_IT_TC = 0x00000400,
    ARGB_IT_BYTE = 0x00000800,
    ARGB_IT_HALF = 0x00001000,
    ARGB_IT_TXE = 0x00002000
} ARGB_IT_Typedef;

/* ARGB status enumeration */
typedef enum
{
    ARGB_Idle = 0,
    ARGB_SendingData,
    ARGB_SendingRST
} ARGB_STATypeDef;

/* ARGB initialization structure definition */
typedef struct
{
    uint16_t ARGB_Length;                       /* Frame data length count in bytes
                                                    This parameter must be a number between 0 and 0xFFFF */
    uint16_t ARGB_T1H;                          /* The high-level duration of Logic 1, the time is based on the system clock.
                                                    This parameter must be a number between 0 and 0xFFF */
    uint16_t ARGB_T0H;                          /* The high-level duration of Logic 0, the time is based on the system clock.
                                                    This parameter must be a number between 0 and 0xFFF */
    uint16_t ARGB_DataPeriod;                   /* Data period
                                                    This parameter must be a number between 0 and 0xFFF */
    uint32_t ARGB_RSTPeriod;                    /* Reset period
                                                    This parameter must be a number between 0 and 0x1FFFF */
    ARGB_Mode_Typedef ARGB_Mode;                /* ARGB mode: send RST first or send data first
                                                    This parameter must be one of the @ref ARGB_Mode_Typedef enumeration */
    ARGB_Endian_Typedef ARGB_Endian;            /* Data endianness
                                                    This parameter must be one of the @ref ARGB_Endian_Typedef enumeration */
} ARGB_InitTypeDef;

void ARGB_DeInit(void);
void ARGB_Init(ARGB_InitTypeDef *ARGB_InitStruct);
void ARGB_StructInit(ARGB_InitTypeDef *ARGB_InitStruct);
void ARGB_DMACmd(FunctionalState NewState);
void ARGB_Cmd(FunctionalState NewState);
void ARGB_ITConfig(ARGB_IT_Typedef ARGB_IT, FunctionalState NewState);
void ARGB_ClearFlag(ARGB_IT_Typedef ARGB_IT);
FlagStatus ARGB_GetFlagStatus(ARGB_IT_Typedef ARGB_IT);
void ARGB_ClearITPendingBit(ARGB_IT_Typedef ARGB_IT);
FlagStatus ARGB_GetITStatus(ARGB_IT_Typedef ARGB_IT);
uint32_t ARGB_GetCurrBaseCNT(void);
uint32_t ARGB_GetBitCNT(void);
uint32_t ARGB_GetByteCNT(void);
void ARGB_SendData(uint8_t Data);
ARGB_STATypeDef ARGB_GetCurrState(void);

#ifdef __cplusplus
}
#endif

#endif
