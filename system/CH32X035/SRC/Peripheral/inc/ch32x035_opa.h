/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32x035_opa.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/04/06
 * Description        : This file contains all the functions prototypes for the
 *                      OPA firmware library.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32X035_OPA_H
#define __CH32X035_OPA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ch32x035.h"

/* OPA_member_enumeration */
typedef enum
{
    OPA1 = 0,
    OPA2
} OPA_Num_TypeDef;

/* OPA_out_channel_enumeration */
typedef enum
{
    OUT_IO_OUT0 = 0,
    OUT_IO_OUT1
} OPA_Mode_TypeDef;

/* OPA_PSEL_enumeration */
typedef enum
{
    CHP0 = 0,
    CHP1,
    CHP2,
    CHP_OFF
} OPA_PSEL_TypeDef;

/* OPA_FB_enumeration */
typedef enum
{
    FB_OFF = 0,
    FB_ON
} OPA_FB_TypeDef;

/* OPA_NSEL_enumeration */
typedef enum
{
    CHN0 = 0,
    CHN1,
    CHN2_PGA_16xIN,
    CHN_PGA_4xIN,
    CHN_PGA_8xIN,
    CHN_PGA_16xIN,
    CHN_PGA_32xIN,
    CHN_OFF
} OPA_NSEL_TypeDef;

/* OPA_PSEL_POLL_enumeration */
typedef enum
{
    CHP_OPA1_OFF_OPA2_OFF = 0,
    CHP_OPA1_ON_OPA2_OFF,
    CHP_OPA1_OFF_OPA2_ON,
    CHP_OPA1_ON_OPA2_ON
} OPA_PSEL_POLL_TypeDef;

/* OPA_BKIN_EN_enumeration */
typedef enum
{
    BKIN_OPA1_OFF_OPA2_OFF = 0,
    BKIN_OPA1_ON_OPA2_OFF,
    BKIN_OPA1_OFF_OPA2_ON,
    BKIN_OPA1_ON_OPA2_ON
} OPA_BKIN_EN_TypeDef;

/* OPA_RST_EN_enumeration */
typedef enum
{
    RST_OPA1_OFF_OPA2_OFF = 0,
    RST_OPA1_ON_OPA2_OFF,
    RST_OPA1_OFF_OPA2_ON,
    RST_OPA1_ON_OPA2_ON
} OPA_RST_EN_TypeDef;

/* OPA_BKIN_SEL_enumeration */
typedef enum
{
    BKIN_OPA1_TIM1_OPA2_TIM2 = 0,
    BKIN_OPA1_TIM2_OPA2_TIM1
} OPA_BKIN_SEL_TypeDef;

/* OPA_OUT_IE_enumeration */
typedef enum
{
    OUT_IE_OPA1_OFF_OPA2_OFF = 0,
    OUT_IE_OPA1_ON_OPA2_OFF,
    OUT_IE_OPA1_OFF_OPA2_ON,
    OUT_IE_OPA1_ON_OPA2_ON
} OPA_OUT_IE_TypeDef;

/* OPA_CNT_IE_enumeration */
typedef enum
{
    CNT_IE_OFF = 0,
    CNT_IE_ON,
} OPA_CNT_IE_TypeDef;

/* OPA_NMI_IE_enumeration */
typedef enum
{
    NMI_IE_OFF = 0,
    NMI_IE_ON,
} OPA_NMI_IE_TypeDef;

/* OPA_PSEL_POLL_NUM_enumeration */
typedef enum
{
    CHP_POLL_NUM_1 = 0,
    CHP_POLL_NUM_2,
    CHP_POLL_NUM_3
} OPA_PSEL_POLL_NUM_TypeDef;


/* OPA Init Structure definition */
typedef struct
{
    uint16_t                  OPA_POLL_Interval; /* OPA polling interval = (OPA_POLL_Interval+1)*1us
                                                      This parameter must range from 0 to 0x1FF.*/
    OPA_Num_TypeDef           OPA_NUM;  /* Specifies the members of OPA */
    OPA_Mode_TypeDef          Mode;     /* Specifies the mode of OPA */
    OPA_PSEL_TypeDef          PSEL;     /* Specifies the positive channel of OPA */
    OPA_FB_TypeDef            FB;       /* Specifies the internal feedback resistor of OPA */
    OPA_NSEL_TypeDef          NSEL;     /* Specifies the negative channel of OPA */
    OPA_PSEL_POLL_TypeDef     PSEL_POLL; /* Specifies the positive channel poll of OPA */
    OPA_BKIN_EN_TypeDef       BKIN_EN;  /* Specifies the brake input source of OPA */
    OPA_RST_EN_TypeDef        RST_EN;   /* Specifies the reset source of OPA */
    OPA_BKIN_SEL_TypeDef      BKIN_SEL; /* Specifies the brake input source selection of OPA */
    OPA_OUT_IE_TypeDef        OUT_IE;   /* Specifies the out interrupt of OPA */
    OPA_CNT_IE_TypeDef        CNT_IE;   /* Specifies the out interrupt rising edge of sampling data */
    OPA_NMI_IE_TypeDef        NMI_IE;   /* Specifies the out NIM interrupt of OPA */
    OPA_PSEL_POLL_NUM_TypeDef POLL_NUM; /* Specifies the number of forward inputs*/
} OPA_InitTypeDef;

/* CMP_member_enumeration */
typedef enum
{
    CMP1 = 0,
    CMP2,
    CMP3
} CMP_Num_TypeDef;

/* CMP_out_channel_enumeration */
typedef enum
{
    OUT_IO_TIM2 = 0,
    OUT_IO0
} CMP_Mode_TypeDef;

/* CMP_NSEL_enumeration */
typedef enum
{
    CMP_CHN0 = 0,
    CMP_CHN1,
} CMP_NSEL_TypeDef;

/* CMP_PSEL_enumeration */
typedef enum
{
    CMP_CHP1 = 0,
    CMP_CHP2,
} CMP_PSEL_TypeDef;

/* CMP_HYEN_enumeration */
typedef enum
{
    CMP_HYEN1 = 0,
    CMP_HYEN2,
} CMP_HYEN_TypeDef;

/* CMP Init Structure definition */
typedef struct
{
    CMP_Num_TypeDef    CMP_NUM;  /* Specifies the members of CMP */
    CMP_Mode_TypeDef   Mode;     /* Specifies the mode of CMP */
    CMP_NSEL_TypeDef   NSEL;     /* Specifies the negative channel of CMP */
    CMP_PSEL_TypeDef   PSEL;     /* Specifies the positive channel of CMP */
    CMP_HYEN_TypeDef   HYEN;     /* Specifies the hysteresis comparator of CMP */
} CMP_InitTypeDef;

/* OPA_flags_definition */
#define OPA_FLAG_OUT_OPA1                  ((uint16_t)0x1000)
#define OPA_FLAG_OUT_OPA2                  ((uint16_t)0x2000)
#define OPA_FLAG_OUT_CNT                   ((uint16_t)0x4000)

void       OPA_Unlock(void);
void       OPA_Lock(void);
void       OPA_POLL_Unlock(void);
void       OPA_POLL_Lock(void);
void       OPA_CMP_Unlock(void);
void       OPA_CMP_Lock(void);
void       OPA_Init(OPA_InitTypeDef *OPA_InitStruct);
void       OPA_StructInit(OPA_InitTypeDef *OPA_InitStruct);
void       OPA_Cmd(OPA_Num_TypeDef OPA_NUM, FunctionalState NewState);
void       OPA_CMP_Init(CMP_InitTypeDef *CMP_InitStruct);
void       OPA_CMP_StructInit(CMP_InitTypeDef *CMP_InitStruct);
void       OPA_CMP_Cmd(CMP_Num_TypeDef CMP_NUM, FunctionalState NewState);
FlagStatus OPA_GetFlagStatus( uint16_t OPA_FLAG);
void       OPA_ClearFlag(uint16_t OPA_FLAG);

#ifdef __cplusplus
}
#endif

#endif
