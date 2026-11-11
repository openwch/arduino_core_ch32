/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH572_common.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : head file(ch572/ch570)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/


#ifndef __CH572_COMM_H__
#define __CH572_COMM_H__

#ifdef __cplusplus
 extern "C" {
#endif


#ifndef  NULL
#define  NULL     0
#endif
#define  ALL			0xFFFF

#ifndef __HIGH_CODE
#define __HIGH_CODE   __attribute__((section(".highcode")))
#endif

#ifndef __INTERRUPT
#ifdef INT_SOFT
#define __INTERRUPT   __attribute__((interrupt()))
#else
#define __INTERRUPT   __attribute__((interrupt("WCH-Interrupt-fast")))
#endif
#endif

#ifdef DEBUG
#include <stdio.h>
#endif

#ifdef  DEBUG
#define PRINT(X...) printf(X)
#else
#define PRINT(X...)
#endif

/**
 * @brief  ϵͳ��Ƶʱ�ӣ�Hz��
 */
#ifndef	 FREQ_SYS
#define  FREQ_SYS		100000000
#endif

#ifndef  SAFEOPERATE
#define  SAFEOPERATE   asm volatile("fence.i")
#endif

#include <string.h>
#include <stdint.h>
#include <CH572SFR.h>
#include "core_riscv.h"
#include "CH572_clk.h"
#include "CH572_cmp.h"
#include "CH572_keyscan.h"
#include "CH572_uart.h"
#include "CH572_gpio.h"
#include "CH572_i2c.h"
#include "CH572_flash.h"
#include "CH572_pwr.h"
#include "CH572_pwm.h"
#include "CH572_sys.h"
#include "CH572_timer.h"
#include "CH572_spi.h"
#include "CH572_usbdev.h"
#include "CH572_usbhost.h"
#include "ISP572.h"

 /**
  * @brief  LSIʱ�ӣ�Hz��
  */
 extern uint32_t Freq_LSI;

#define DelayMs(x)      mDelaymS(x)
#define DelayUs(x)      mDelayuS(x)

#define ROM_CFG_VERISON    0x7F010

#ifdef __cplusplus
}
#endif

#endif  // __CH572_COMM_H__

