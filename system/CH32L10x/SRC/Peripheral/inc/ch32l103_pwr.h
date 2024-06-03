/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32l103_pwr.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/07/08
 * Description        : This file contains all the functions prototypes for the PWR
 *                      firmware library.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#ifndef __CH32L103_PWR_H
#define __CH32L103_PWR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ch32l103.h"

/* PVD_detection_level  */
#define PWR_PVDLevel_0            ((uint32_t)0x00000000)
#define PWR_PVDLevel_1            ((uint32_t)0x00000020)
#define PWR_PVDLevel_2            ((uint32_t)0x00000040)
#define PWR_PVDLevel_3            ((uint32_t)0x00000060)
#define PWR_PVDLevel_4            ((uint32_t)0x00000080)
#define PWR_PVDLevel_5            ((uint32_t)0x000000A0)
#define PWR_PVDLevel_6            ((uint32_t)0x000000C0)
#define PWR_PVDLevel_7            ((uint32_t)0x000000E0)

/* Regulator_state_is_STOP_mode */
#define PWR_Regulator_ON          ((uint32_t)0x00000000)
#define PWR_Regulator_LowPower    ((uint32_t)0x00000001)

/* STOP_mode_entry */
#define PWR_STOPEntry_WFI         ((uint8_t)0x01)
#define PWR_STOPEntry_WFE         ((uint8_t)0x02)

/* PWR_Flag */
#define PWR_FLAG_WU               ((uint32_t)0x00000001)
#define PWR_FLAG_SB               ((uint32_t)0x00000002)
#define PWR_FLAG_PVDO             ((uint32_t)0x00000004)

void       PWR_DeInit(void);
void       PWR_BackupAccessCmd(FunctionalState NewState);
void       PWR_PVDCmd(FunctionalState NewState);
void       PWR_PVDLevelConfig(uint32_t PWR_PVDLevel);
void       PWR_WakeUpPinCmd(FunctionalState NewState);
void       PWR_EnterSTOPMode(uint32_t PWR_Regulator, uint8_t PWR_STOPEntry);
void       PWR_EnterSTANDBYMode(void);
FlagStatus PWR_GetFlagStatus(uint32_t PWR_FLAG);
void       PWR_ClearFlag(uint32_t PWR_FLAG);
void       PWR_EnterSTANDBYMode_RAM(void);
void       PWR_EnterSTANDBYMode_RAM_LV(void);
void       PWR_EnterSTANDBYMode_RAM_VBAT_EN(void);
void       PWR_EnterSTANDBYMode_RAM_LV_VBAT_EN(void);
void       PWR_EnterSTOPMode_RAM_LV(uint32_t PWR_Regulator, uint8_t PWR_STOPEntry);
void       PWR_LDO_LP_Cmd(FunctionalState NewState);
void       PWR_STOPMode_Auto_LDO_LP_Cmd(FunctionalState NewState);
void       PWR_FLASH_LP_Cmd(FunctionalState NewState);


#ifdef __cplusplus
}
#endif

#endif
