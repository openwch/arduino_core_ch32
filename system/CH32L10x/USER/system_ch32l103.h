/********************************** (C) COPYRIGHT *******************************
 * File Name          : system_ch32l103.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/07/08
 * Description        : CH32L103 Device Peripheral Access Layer System Header File.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __SYSTEM_ch32l103_H
#define __SYSTEM_ch32l103_H

#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t SystemCoreClock; /* System Clock Frequency (Core Clock) */

/* System_Exported_Functions */
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);

#ifdef __cplusplus
}
#endif

#endif
