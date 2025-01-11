/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32x035_pwr.h
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
#ifndef __CH32X035_PWR_H
#define __CH32X035_PWR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ch32x035.h"

/* PVD_detection_level  */
#define PWR_PVDLevel_0            ((uint32_t)0x00000000)
#define PWR_PVDLevel_1            ((uint32_t)0x00000020)
#define PWR_PVDLevel_2            ((uint32_t)0x00000040)
#define PWR_PVDLevel_3            ((uint32_t)0x00000060)

#define PWR_PVDLevel_2V1          PWR_PVDLevel_0
#define PWR_PVDLevel_2V3          PWR_PVDLevel_1
#define PWR_PVDLevel_3V0          PWR_PVDLevel_2
#define PWR_PVDLevel_4V0          PWR_PVDLevel_3

/* STOP_mode_entry */
#define PWR_STOPEntry_WFI         ((uint8_t)0x01)
#define PWR_STOPEntry_WFE         ((uint8_t)0x02)

/* PWR_Flag */
#define PWR_FLAG_PVDO             ((uint32_t)0x00000004)
#define PWR_FLAG_FLASH            ((uint32_t)0x00000020)

/* PWR_VDD_Supply_Voltage */
typedef enum {PWR_VDD_5V = 0, PWR_VDD_3V3 = !PWR_VDD_5V} PWR_VDD;

void       PWR_DeInit(void);
void       PWR_PVDLevelConfig(uint32_t PWR_PVDLevel);
void       PWR_EnterSTOPMode(uint8_t PWR_STOPEntry);
void       PWR_EnterSTANDBYMode(void);
FlagStatus PWR_GetFlagStatus(uint32_t PWR_FLAG);
PWR_VDD    PWR_VDD_SupplyVoltage(void);

#ifdef __cplusplus
}
#endif

#endif
