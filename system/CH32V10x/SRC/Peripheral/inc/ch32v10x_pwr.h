/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32v10x_pwr.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/06/14
 * Description        : This file contains all the functions prototypes for the PWR
 *                      firmware library.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32V10x_PWR_H
#define __CH32V10x_PWR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ch32v10x.h"

/* PVD_detection_level  */
#define PWR_PVDLevel_MODE0          ((uint32_t)0x00000000)
#define PWR_PVDLevel_MODE1          ((uint32_t)0x00000020)
#define PWR_PVDLevel_MODE2          ((uint32_t)0x00000040)
#define PWR_PVDLevel_MODE3          ((uint32_t)0x00000060)
#define PWR_PVDLevel_MODE4          ((uint32_t)0x00000080)
#define PWR_PVDLevel_MODE5          ((uint32_t)0x000000A0)
#define PWR_PVDLevel_MODE6          ((uint32_t)0x000000C0)
#define PWR_PVDLevel_MODE7          ((uint32_t)0x000000E0)

#define PWR_PVDLevel_2V7            PWR_PVDLevel_MODE0
#define PWR_PVDLevel_2V9            PWR_PVDLevel_MODE1
#define PWR_PVDLevel_3V1            PWR_PVDLevel_MODE2
#define PWR_PVDLevel_3V3            PWR_PVDLevel_MODE3
#define PWR_PVDLevel_3V5            PWR_PVDLevel_MODE4
#define PWR_PVDLevel_3V8            PWR_PVDLevel_MODE5
#define PWR_PVDLevel_4V1            PWR_PVDLevel_MODE6
#define PWR_PVDLevel_4V4            PWR_PVDLevel_MODE7

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

/* PWR_VDD_Supply_Voltage */
typedef enum {PWR_VDD_5V = 0, PWR_VDD_3V3 = !PWR_VDD_5V} PWR_VDD;

void       PWR_DeInit(void);
void       PWR_BackupAccessCmd(FunctionalState NewState);
void       PWR_PVDCmd(FunctionalState NewState);
void       PWR_PVDLevelConfig(uint32_t PWR_PVDLevel);
void       PWR_WakeUpPinCmd(FunctionalState NewState);
void       PWR_EnterSTOPMode(uint32_t PWR_Regulator, uint8_t PWR_STOPEntry);
void       PWR_EnterSTANDBYMode(void);
FlagStatus PWR_GetFlagStatus(uint32_t PWR_FLAG);
void       PWR_ClearFlag(uint32_t PWR_FLAG);
PWR_VDD    PWR_VDD_SupplyVoltage(void);

#ifdef __cplusplus
}
#endif

#endif /* __CH32V10x_PWR_H */
