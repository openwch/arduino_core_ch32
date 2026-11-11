/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH585_common.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : head file(ch585/ch584)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/


#ifndef __CH585_COMM_H__
#define __CH585_COMM_H__

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

#define Debug_UART0        0
#define Debug_UART1        1
#define Debug_UART2        2
#define Debug_UART3        3

#ifdef DEBUG
#include <stdio.h>
#endif

#include <string.h>
#include <stdint.h>
#include <CH585SFR.h>
#include "core_riscv.h"
#include "CH585_clk.h"
#include "CH585_uart.h"
#include "CH585_gpio.h"
#include "CH585_i2c.h"
#include "CH585_flash.h"
#include "CH585_pwr.h"
#include "CH585_pwm.h"
#include "CH585_adc.h"
#include "CH585_sys.h"
#include "CH585_timer.h"
#include "CH585_spi.h"
#include "CH585_usbdev.h"
#include "CH585_usbhost.h"
#include "ISP585.h"

 /**
  * @brief  ϵͳ��Ƶʱ�ӣ�Hz��
  */
#ifndef  SYSCLK_FREQ
#define  SYSCLK_FREQ    CLK_SOURCE_HSE_PLL_62_4MHz
#endif
#ifndef	 FREQ_SYS
#define  FREQ_SYS       62400000
#endif

#ifndef  SAFEOPERATE
#define  SAFEOPERATE   asm volatile("fence.i")
#endif

/**
 * @brief  32Kʱ�ӣ�Hz��
 */
#ifdef CLK_OSC32K
#if ( CLK_OSC32K == 1 )
#define CAB_LSIFQ       32000
#else
#define CAB_LSIFQ       32768
#endif
#else
#define CAB_LSIFQ       32000
#endif

#define DelayMs(x)      mDelaymS(x)
#define DelayUs(x)      mDelayuS(x)

#define ROM_CFG_VERISON    0x7F010

extern uint32_t chip_info;

#ifdef __cplusplus
}
#endif

#endif  // __CH585_COMM_H__

