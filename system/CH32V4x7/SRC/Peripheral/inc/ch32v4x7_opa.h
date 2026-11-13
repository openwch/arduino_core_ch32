/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32v4x7_opa.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/12/01
* Description        : This file contains all the functions prototypes for the 
*                      OPA firmware library.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32V4x7_OPA_H
#define __CH32V4x7_OPA_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "ch32v4x7.h"


/* OPA PSEL enumeration */
typedef enum
{
   CHP0=0,
   CHP1
}OPA_PSEL_TypeDef;

/* OPA NSEL enumeration */
typedef enum
{
   CHN0 = 0,
   CHN1,
   CHN_PGA_4xIN,
   CHN_PGA_8xIN,
   CHN_PGA_16xIN,
   CHN_PGA_32xIN,
   CHN_OFF = 0x7
}OPA_NSEL_TypeDef;

/* OPA out channel enumeration */
typedef enum
{
   OUT_IO_OUT0 = 0,
   OUT_IO_OUT1,
}OPA_Mode_TypeDef;

/* OPA_FB_enumeration */
typedef enum
{
    FB_OFF = 0,
    FB_ON
} OPA_FB_TypeDef;

/* OPA_HS_enumeration */
typedef enum
{
    HS_OFF = 0,
    HS_ON
} OPA_HS_TypeDef;

/* OPA_PGADIF_enumeration */
typedef enum
{
    PGADIF_GND = 0,
    PGADIF_IO 
} OPA_PGADIF_TypeDef;
/* OPA Init Structure definition */
typedef struct
{
  OPA_PSEL_TypeDef  PSEL;         /* Specifies the positive channel of OPA */
  OPA_NSEL_TypeDef  NSEL;         /* Specifies the negative channel of OPA */
  OPA_Mode_TypeDef  Mode;         /* Specifies the mode of OPA */
  OPA_FB_TypeDef  FB;             /* Specifies the mode of OPA */
  OPA_HS_TypeDef  HS;             /* Specifies the mode of OPA */
  OPA_PGADIF_TypeDef   PGADIF;    /* OPA with NSEL1 for use as a PGA */
}OPA_InitTypeDef;


void OPA_DeInit(void);
void OPA_Init(OPA_InitTypeDef* OPA_InitStruct);
void OPA_StructInit(OPA_InitTypeDef* OPA_InitStruct);
void OPA_Cmd(FunctionalState NewState);

#ifdef __cplusplus
}
#endif

#endif 

