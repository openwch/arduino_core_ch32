/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32v10x_misc.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/01/05
 * Description        : This file contains all the functions prototypes for the 
 *                      miscellaneous firmware library functions.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/   
#ifndef __CH32V10X_MISC_H
#define __CH32V10X_MISC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ch32v10x.h"

/* CSR_INTSYSCR_INEST_definition */
#define INTSYSCR_INEST_NoEN   0x00   /* interrupt nesting disable(PFIC->CFGR bit1 = 1) */
#define INTSYSCR_INEST_EN     0x01   /* interrupt nesting enable(PFIC->CFGR bit1 = 0) */

/* Check the configuration of PFIC->CFGR
 *   interrupt nesting enable(PFIC->CFGR bit1 = 0)
 *     priority - bit[7] - Preemption Priority
 *                bit[6:4] - Sub priority
 *                bit[3:0] - Reserve
 *   interrupt nesting disable(PFIC->CFGR bit1 = 1)
 *     priority - bit[7:4] - Sub priority
 *                bit[3:0] - Reserve
 */

#ifndef INTSYSCR_INEST
#define INTSYSCR_INEST   INTSYSCR_INEST_EN
#endif

/* NVIC Init Structure definition
 *   interrupt nesting enable(PFIC->CFGR bit1 = 0)
 *     NVIC_IRQChannelPreemptionPriority - range from 0 to 1.
 *     NVIC_IRQChannelSubPriority - range from 0 to 7.
 *
 *   interrupt nesting disable(PFIC->CFGR bit1 = 1)
 *     NVIC_IRQChannelPreemptionPriority - range is 0.
 *     NVIC_IRQChannelSubPriority - range from 0 to 0xF.
 *
 */
typedef struct
{
    uint8_t NVIC_IRQChannel;
    uint8_t NVIC_IRQChannelPreemptionPriority;
    uint8_t NVIC_IRQChannelSubPriority;
    FunctionalState NVIC_IRQChannelCmd;
} NVIC_InitTypeDef;

/* Preemption_Priority_Group */
#if (INTSYSCR_INEST == INTSYSCR_INEST_NoEN)
#define NVIC_PriorityGroup_0           ((uint32_t)0x00) /* interrupt nesting disable(PFIC->CFGR bit1 = 1) */
#else
#define NVIC_PriorityGroup_1           ((uint32_t)0x01) /* interrupt nesting enable(PFIC->CFGR bit1 = 0) */
#endif


void NVIC_PriorityGroupConfig(uint32_t NVIC_PriorityGroup);
void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct);

#ifdef __cplusplus
}
#endif

#endif /* __CH32V10x_MISC_H */

