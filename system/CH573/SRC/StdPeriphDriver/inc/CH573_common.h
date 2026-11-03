/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH573_common.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __CH573_COMM_H__
#define __CH573_COMM_H__

#ifdef __cplusplus
extern "C" {
#endif

#ifndef NULL
  #define NULL    0
#endif
#define ALL            0xFFFF

#define Debug_UART0    0
#define Debug_UART1    1
#define Debug_UART2    2
#define Debug_UART3    3

#ifdef DEBUG
  #include <stdio.h>
#endif

/**
 * @brief  系统主频时钟（Hz）
 */
#ifndef FREQ_SYS
  #define FREQ_SYS    60000000
#endif

/**
 * @brief  32K时钟（Hz）
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

#include <string.h>
#include <stdint.h>
#include "CH573SFR.h"
#include "core_riscv.h"
#include "CH573_clk.h"
#include "CH573_uart.h"
#include "CH573_gpio.h"
#include "CH573_flash.h"
#include "CH573_pwr.h"
#include "CH573_pwm.h"
#include "CH573_adc.h"
#include "CH573_sys.h"
#include "CH573_timer.h"
#include "CH573_spi.h"
#include "CH573_usbdev.h"
#include "CH573_usbhost.h"
#include "ISP573.h"

#define DelayMs(x)    mDelaymS(x)
#define DelayUs(x)    mDelayuS(x)

#ifdef __cplusplus
}
#endif

#endif // __CH573_COMM_H__
