/**
 *******************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * This software component is licensed by WCH under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BACKUP_H
#define __BACKUP_H

/* Includes ------------------------------------------------------------------*/
#include "ch32_def.h"
#include "ch32yyxx_rtc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported macro ------------------------------------------------------------*/
#if !defined(RTC_BKP_BASE) && defined(BKP_DR1)
#define RTC_BKP_BASE BKP_DR1
#else
#define RTC_BKP_BASE 0
#endif
#ifndef RTC_BKP_VALUE
#define RTC_BKP_VALUE 0x4348
#endif


/* Exported functions ------------------------------------------------------- */
static inline void resetBackupDomain(void)
{
#ifdef PWR_MODULE_ENABLED
  PWR_BackupAccessCmd(ENABLE);
#endif
#ifndef CH32V00x
  RCC_BackupResetCmd(ENABLE);
  RCC_BackupResetCmd(DISABLE);
#endif
}

static inline void enableBackupDomain(void)
{
#ifdef PWR_MODULE_ENABLED
#if defined(CH32L10x)
  RCC_PB1PeriphClockCmd(RCC_PB1Periph_PWR,ENABLE);
#else
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
#endif
  /* Allow access to Backup domain */
  PWR_BackupAccessCmd(ENABLE);
#endif
#ifdef RCC_APB1Periph_BKP
  /* Enable BKP CLK for backup registers */
  #if defined(CH32L10x)
  RCC_PB1PeriphClockCmd(RCC_PB1Periph_BKP, ENABLE);  
  #else
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_BKP, ENABLE);
  #endif
#endif
}

static inline void disableBackupDomain(void)
{
#ifdef PWR_MODULE_ENABLED
  /* Forbid access to Backup domain */
  PWR_BackupAccessCmd(DISABLE);
  #if defined(CH32L10x)  
  RCC_PB1PeriphClockCmd(RCC_PB1Periph_PWR, DISABLE);
  #else
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, DISABLE);
  #endif
#endif
#ifdef RCC_APB1Periph_BKP
  /* Disable BKP CLK for backup registers */
  #if defined(CH32L10x)
  RCC_PB1PeriphClockCmd(RCC_PB1Periph_BKP, DISABLE);
  #else
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_BKP, DISABLE);
  #endif
#endif
}

static inline void setBackupRegister(uint32_t index, uint32_t value)
{
#if defined(BKP_BASE)
  BKP_WriteBackupRegister(index*4 + RTC_BKP_BASE, value);
#else
  (void)(index);
  (void)(value);
#endif
}

static inline uint32_t getBackupRegister(uint32_t index)
{
#if defined(BKP_BASE)
  return BKP_ReadBackupRegister(index*4 + RTC_BKP_BASE);
#else
  (void)(index);
  return 0;
#endif
}



#ifdef __cplusplus
}
#endif

#endif /* __BACKUP_H */


