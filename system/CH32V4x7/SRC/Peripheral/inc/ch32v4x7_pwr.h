/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32v4x7_pwr.h
* Author             : WCH
* Version            : V1.0.1
* Date               : 2025/04/10
* Description        : This file contains all the functions prototypes for the PWR  
*                      firmware library.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32V4x7_PWR_H
#define __CH32V4x7_PWR_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "ch32v4x7.h"

/* PVD_detection_level  */
#define PWR_PVDLevel_MODE0          ((uint32_t)0x00000000)
#define PWR_PVDLevel_MODE1          ((uint32_t)0x00000020)
#define PWR_PVDLevel_MODE2          ((uint32_t)0x00000040)
#define PWR_PVDLevel_MODE3          ((uint32_t)0x00000060)
	 
/* Regulator_state_is_STOP_mode */
#define PWR_Regulator_ON            ((uint32_t)0x00000000)
#define PWR_Regulator_LowPower      ((uint32_t)0x00000001)

/* VDD18_level */
#define PWR_VDD18_Level0            ((uint32_t)0x00000000)
#define PWR_VDD18_Level1            ((uint32_t)0x00001000)

/* VDDK_detection_level  */
#define PWR_VDDK_Level0             ((uint32_t)0x00000000)
#define PWR_VDDK_Level1             ((uint32_t)0x01000000)
#define PWR_VDDK_Level2             ((uint32_t)0x02000000)
#define PWR_VDDK_Level3             ((uint32_t)0x03000000)
#define PWR_VDDK_Level4             ((uint32_t)0x04000000)
#define PWR_VDDK_Level5             ((uint32_t)0x05000000)
#define PWR_VDDK_Level6             ((uint32_t)0x06000000)
#define PWR_VDDK_Level7             ((uint32_t)0x07000000)

/* STOP_mode_entry */
#define PWR_STOPEntry_WFI           ((uint8_t)0x01)
#define PWR_STOPEntry_WFE           ((uint8_t)0x02)
 
/* PWR_Flag */
#define PWR_FLAG_WU                 ((uint32_t)0x00000001)
#define PWR_FLAG_SB                 ((uint32_t)0x00000002)
#define PWR_FLAG_PVDO               ((uint32_t)0x00000004)


void PWR_DeInit(void);
void PWR_BackupAccessCmd(FunctionalState NewState);
void PWR_PVDCmd(FunctionalState NewState);
void PWR_PVDLevelConfig(uint32_t PWR_PVDLevel);
void PWR_WakeUpPinCmd(FunctionalState NewState);
void PWR_EnterSTOPMode(uint32_t PWR_Regulator, uint8_t PWR_STOPEntry);
void PWR_EnterSTANDBYMode(void);
FlagStatus PWR_GetFlagStatus(uint32_t PWR_FLAG);
void PWR_ClearFlag(uint32_t PWR_FLAG);
void PWR_EnterSTANDBYMode_RAM(void);
void PWR_EnterSTANDBYMode_RAM_LV(void);
void PWR_EnterSTANDBYMode_RAM_VBAT_EN(void);
void PWR_EnterSTANDBYMode_RAM_LV_VBAT_EN(void);
void PWR_EnterSTOPMode_RAM_LV(uint32_t PWR_Regulator, uint8_t PWR_STOPEntry);
void PWR_VDD18Cmd(FunctionalState NewState);
void PWR_VDD18LevelConfig(uint32_t PWR_VDD18_Level);

#ifdef __cplusplus
}
#endif

#endif 

