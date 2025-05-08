/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32x035_awu.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/04/06
 * Description        : This file contains all the functions prototypes for the
 *                      AWU firmware library.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32X035_AWU_H
#define __CH32X035_AWU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ch32x035.h"

/* PWR_AWU_Prescaler */
#define AWU_Prescaler_1       ((uint32_t)0x00000000)
#define AWU_Prescaler_2       ((uint32_t)0x00000002)
#define AWU_Prescaler_4       ((uint32_t)0x00000003)
#define AWU_Prescaler_8       ((uint32_t)0x00000004)
#define AWU_Prescaler_16      ((uint32_t)0x00000005)
#define AWU_Prescaler_32      ((uint32_t)0x00000006)
#define AWU_Prescaler_64      ((uint32_t)0x00000007)
#define AWU_Prescaler_128     ((uint32_t)0x00000008)
#define AWU_Prescaler_256     ((uint32_t)0x00000009)
#define AWU_Prescaler_512     ((uint32_t)0x0000000A)
#define AWU_Prescaler_1024    ((uint32_t)0x0000000B)
#define AWU_Prescaler_2048    ((uint32_t)0x0000000C)
#define AWU_Prescaler_4096    ((uint32_t)0x0000000D)
#define AWU_Prescaler_10240   ((uint32_t)0x0000000E)
#define AWU_Prescaler_61440   ((uint32_t)0x0000000F)


void AutoWakeUpCmd(FunctionalState NewState);
void AWU_SetPrescaler(uint32_t AWU_Prescaler);
void AWU_SetWindowValue(uint8_t WindowValue);

#ifdef __cplusplus
}
#endif

#endif
