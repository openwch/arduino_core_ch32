/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32l103_opa.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/11/05
 * Description        : This file contains all the functions prototypes for the
 *                      OPA firmware library.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#ifndef __CH32L103_OPA_H
#define __CH32L103_OPA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ch32l103.h"

/* OPA_member_enumeration */
typedef enum
{
    OPA1 = 0,
} OPA_Num_TypeDef;

/* OPA_out_channel_enumeration */
typedef enum
{
    OUT_IO_OUT0 = 0,       /* PA3 */
    OUT_IO_OUT1,           /* PB1 */
    OUT_IO_OUT2,           /* PA2 */
    OUT_IO_OUT3,           /* PA4 */
    OUT_IO_OUT4,           /* PB0 */
    OUT_IO_OFF
} OPA_Mode_TypeDef;

/* OPA_PSEL_enumeration */
typedef enum
{
    CHP0 = 0,              /* PB15 */
    CHP1,                  /* PB0 */
    CHP2,                  /* PB14 */
    CHP3,                  /* PA7 */
    CHP4,                  /* PA0 */
    CHP5,                  /* PA6 */
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
    CHN0 = 0,             /* PB11 */
    CHN1,                 /* PA6 */
    CHN2,                 /* PB10 */
    CHN3,                 /* PA5 */
    CHN4,                 /* PA1 */
    CHN5,                 /* PA7 */
    CHN2_PGA_32xIN,       /* PB10 */
    CHN_PGA_8xIN,
    CHN_PGA_16xIN,
    CHN_PGA_32xIN,
    CHN_PGA_64xIN,
    CHN_OFF = 0xF
} OPA_NSEL_TypeDef;

/* OPA_PSEL_POLL_enumeration */
typedef enum
{
    CHP_OPA1_OFF = 0,
    CHP_OPA1_ON,
} OPA_PSEL_POLL_TypeDef;

/* OPA_BKIN_EN_enumeration */
typedef enum
{
    BKIN_OPA1_OFF = 0,  /* TIM1 braking signal source form IO input */
    BKIN_OPA1_ON,       /* TIM1 braking signal source form OPA output */
} OPA_BKIN_EN_TypeDef;

/* OPA_RST_EN_enumeration */
typedef enum
{
    RST_OPA1_OFF = 0,
    RST_OPA1_ON,
} OPA_RST_EN_TypeDef;

/* OPA_OUT_IE_enumeration */
typedef enum
{
    OUT_IE_OPA1_OFF = 0,
    OUT_IE_OPA1_ON,
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
    CHP_POLL_NUM_3,
	CHP_POLL_NUM_4,
    CHP_POLL_NUM_5,
    CHP_POLL_NUM_6
} OPA_PSEL_POLL_NUM_TypeDef;

/* Offset_voltage_adjustment_value_polarity */
typedef enum
{
    OPA_Vos_Ads_N = 0,
    OPA_Vos_Ads_P
} OPA_Vos_ADS_POLARITY_TypeDef;

/* OPA Init Structure definition */
typedef struct
{
    uint16_t                       OPA_POLL_Interval; /* OPA polling interval = (OPA_POLL_Interval+1)*1us
                                                      This parameter must range from 0 to 0x1FF.*/
    OPA_Num_TypeDef                OPA_NUM;  /* Specifies the members of OPA */
    OPA_Mode_TypeDef               Mode;     /* Specifies the mode of OPA */
    OPA_PSEL_TypeDef               PSEL;     /* Specifies the positive channel of OPA */
    OPA_FB_TypeDef                 FB;       /* Specifies the internal feedback resistor of OPA */
    OPA_NSEL_TypeDef               NSEL;     /* Specifies the negative channel of OPA */
    OPA_PSEL_POLL_TypeDef          PSEL_POLL; /* Specifies the positive channel poll of OPA */
    OPA_BKIN_EN_TypeDef            BKIN_EN;  /* Specifies the brake input source of OPA */
    OPA_RST_EN_TypeDef             RST_EN;   /* Specifies the reset source of OPA */
    OPA_OUT_IE_TypeDef             OUT_IE;   /* Specifies the out interrupt of OPA */
    OPA_CNT_IE_TypeDef             CNT_IE;   /* Specifies the out interrupt rising edge of sampling data */
    OPA_NMI_IE_TypeDef             NMI_IE;   /* Specifies the out NIM interrupt of OPA */
    OPA_PSEL_POLL_NUM_TypeDef      POLL_NUM; /* Specifies the number of forward inputs*/
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
    OUT_IO0 = 0,
    OUT_IO1,
    OUT_IO_TIM2
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
    CMP_CHP_0 = 0,
    CMP_CHP_1,
} CMP_PSEL_TypeDef;

#define CMP_CHP1 CMP_CHP_0
#define CMP_CHP2 CMP_CHP_1

/* CMP_HYEN_enumeration */
typedef enum
{
    CMP_HYEN0 = 0,
    CMP_HYEN1,
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

/* Current channel for OPA polling enumeration */
typedef enum
{
    O1P0 = 0,
    O1P1,
    O1P2,
    O1P3,
    O1P4,
    O1P5,
} OPA_POLL_NUM_TypeDef;

/* OPA_flags_definition */
#define OPA_FLAG_OUT_OPA1                  ((uint16_t)0x1000)
#define OPA_FLAG_OUT_CNT                   ((uint16_t)0x4000)

/* CMP_WakeUp_IO_mode_definition */
#define CMP_WakeUp_Rising_Falling          ((uint32_t)0x01000000)
#define CMP_WakeUp_Rising                  ((uint32_t)0x02000000)
#define CMP_WakeUp_Falling                 ((uint32_t)0x03000000)

void       OPCM_Unlock(void);
void       OPCM_Lock(void);
void       OPA_Init(OPA_InitTypeDef *OPA_InitStruct);
void       OPA_StructInit(OPA_InitTypeDef *OPA_InitStruct);
void       OPA_Cmd(OPA_Num_TypeDef OPA_NUM, FunctionalState NewState);
void       OPA_LP_Cmd(FunctionalState NewState);
void       OPA_CMP_Init(CMP_InitTypeDef *CMP_InitStruct);
void       OPA_CMP_StructInit(CMP_InitTypeDef *CMP_InitStruct);
void       OPA_CMP_Cmd(CMP_Num_TypeDef CMP_NUM, FunctionalState NewState);
void       OPA_CMP_LP_Cmd(CMP_Num_TypeDef CMP_NUM, FunctionalState NewState);
void       OPA_CMP_WakeUp_ModeConfig(uint32_t CMP_WakeUP_Mode);
FlagStatus OPA_GetFlagStatus( uint16_t OPA_FLAG);
void       OPA_ClearFlag(uint16_t OPA_FLAG);
OPA_POLL_NUM_TypeDef OPA_POLL_CNT(void);

#ifdef __cplusplus
}
#endif

#endif
