/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32l103_bkp.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/07/08
 * Description        : This file contains all the functions prototypes for the
 *                      BKP firmware library.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#ifndef __CH32L103_BKP_H
#define __CH32L103_BKP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ch32l103.h"

/* Tamper_Pin_active_level */
#define BKP_TamperPinLevel_High           ((uint16_t)0x0000)
#define BKP_TamperPinLevel_Low            ((uint16_t)0x0001)

/* RTC_output_source_to_output_on_the_Tamper_pin */
#define BKP_RTCOutputSource_None          ((uint16_t)0x0000)
#define BKP_RTCOutputSource_CalibClock    ((uint16_t)0x0080)
#define BKP_RTCOutputSource_Alarm         ((uint16_t)0x0100)
#define BKP_RTCOutputSource_Second        ((uint16_t)0x0300)

/* Data_Backup_Register */
#define BKP_DR1                           ((uint16_t)0x0004)
#define BKP_DR2                           ((uint16_t)0x0008)
#define BKP_DR3                           ((uint16_t)0x000C)
#define BKP_DR4                           ((uint16_t)0x0010)
#define BKP_DR5                           ((uint16_t)0x0014)
#define BKP_DR6                           ((uint16_t)0x0018)
#define BKP_DR7                           ((uint16_t)0x001C)
#define BKP_DR8                           ((uint16_t)0x0020)
#define BKP_DR9                           ((uint16_t)0x0024)
#define BKP_DR10                          ((uint16_t)0x0028)
#define BKP_DR11                          ((uint16_t)0x0040)
#define BKP_DR12                          ((uint16_t)0x0044)
#define BKP_DR13                          ((uint16_t)0x0048)


void       BKP_DeInit(void);
void       BKP_TamperPinLevelConfig(uint16_t BKP_TamperPinLevel);
void       BKP_TamperPinCmd(FunctionalState NewState);
void       BKP_ITConfig(FunctionalState NewState);
void       BKP_RTCOutputConfig(uint16_t BKP_RTCOutputSource);
void       BKP_SetRTCCalibrationValue(uint8_t CalibrationValue);
void       BKP_WriteBackupRegister(uint16_t BKP_DR, uint16_t Data);
uint16_t   BKP_ReadBackupRegister(uint16_t BKP_DR);
FlagStatus BKP_GetFlagStatus(void);
void       BKP_ClearFlag(void);
ITStatus   BKP_GetITStatus(void);
void       BKP_ClearITPendingBit(void);

#ifdef __cplusplus
}
#endif

#endif
