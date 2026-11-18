/*
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
#pragma once

/* ENABLE Peripherals */
#ifndef IDE_MENU_PERIPHERALS   // defined when peripherals are enabled/disabled via the IDE menu
#define                         ADC_MODULE_ENABLED
#define                         UART_MODULE_ENABLED
//#define                         SPI_MODULE_ENABLED  // SPI not supported on QFN12
#define                         I2C_MODULE_ENABLED
#define                         TIM_MODULE_ENABLED
#endif

/* CH32V002D4 Pins - subset of CH32V002F4 TSSOP20 (18x GPIO, 8+3 ADC, 1xSER, no OPA) */
// CH32V002J4M6 pins:  1:PA1, 2:PA2, 3:PD0, 5:PC0, 6:PC3, 7:PC4, 8:PC6, 9:PC7, 10:PD1=SWIO, 11:PD4. 12:PD7/RST
//  CH32V002D4U6 - QFN12 pins: PC0/3/4/6/7, PD0/1/4/7, PA1/2, A0-2/7 - PD1=SWIO, PD7=RST
#define PA1                     PIN_A1
#define PA2                     PIN_A0
#define PC0                     2
#define PC3                     3
#define PC4                     PIN_A2
#define PC6                     5
#define PC7                     6
#define PD0                     7
#define PD1                     8
#define PD4                     PIN_A3
#define PD7                     10

#define NUM_DIGITAL_PINS        11
#define NUM_ANALOG_INPUTS       4

// #define ADC_CTLR_ADCAL          
#define ADC_RESOLUTION          12



// On-board LED pin number
#ifndef LED_BUILTIN
  #define LED_BUILTIN           PNUM_NOT_DEFINED
#endif



// On-board user button
#ifndef USER_BTN
  #define USER_BTN              PNUM_NOT_DEFINED
#endif



// UART Definitions
// CH32V002 only has USART1. CH32V002D4U6 pins:
//    RX: PD1=RX1`=SCL=SWIO (!)
//    TX: PD0=TX1`=SDA
// CH32V002D4U6 QFN12 has RX1/TX1, but beware: RX is shared with SWIO! Use "power-off clear" in Link Utility to program after bricking
// TODO: it would be nice to give some warning, but adding a #warning here causea a whole ugly load of them to show when compiling.
// V002D4 as no pins for standard Serial1 (PD5/PD6), Serial1_1 (PD6/PD5) or Serial1_3 (PC0/PC1)
// It only allows Board default and always uses PD0/PD1 using Remap2
#if defined(SERIAL_UART_ENABLE_ALL)
  #error "CH32V002D4 only supports Serial1; Dual Serial is unavailable"
#elif (defined(SERIAL_UART_INSTANCE) && SERIAL_UART_INSTANCE>1)
  #error "CH32V002D4 only supports Serial1; Serial2 cannot be selected"
#elif (defined(SERIAL_UART_INSTANCE) && SERIAL_UART_INSTANCE==1) || defined(SERIAL_UART_PINMAP)
  #error "CH32V002D4 can only select Board default for Serial (TX/RX=PD0/PD1)"
#endif
#ifndef BOARD_DEFAULT_SERIAL
  #define BOARD_DEFAULT_SERIAL 1
#endif

// Default pins for each hardware serial port.
#ifndef PIN_SERIAL1_RX
  #define PIN_SERIAL1_RX     PD1  /* UART1 default RX pin */
#endif
#ifndef PIN_SERIAL1_TX
  #define PIN_SERIAL1_TX     PD0  /* UART1 default TX pin */
#endif


// SPI definitions (CH32V002D4 has no pins for SPI so it's not available)
#if defined(SPI_MODULE_ENABLED)
  #error "CH32V002D4 QFN12 has no SPI. SPI should be disabled"
#endif

// I2C definitions
#if defined(I2C_MODULE_ENABLED)
#if defined(SERIAL_UART_INSTANCE) && defined(UART_MODULE_ENABLED)
  #error "CH32V002D4 QFN12 I2C cannot be used together with UART"
#endif
#ifndef PIN_WIRE_SDA
  #define PIN_WIRE_SDA          PD0
#endif
#ifndef PIN_WIRE_SCL
  #define PIN_WIRE_SCL          PD1
#endif
#endif // I2C_MODULE_ENABLED

// Timer Definitions
#ifndef TIMER_TONE
  #define TIMER_TONE            TIM2
#endif
#ifndef TIMER_SERVO
  #define TIMER_SERVO           TIM1
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
  // These serial port names are intended to allow libraries and architecture-neutral
  // sketches to automatically default to the correct port name for a particular type
  // of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
  // the first hardware serial port whose RX/TX pins are not dedicated to another use.
  //
  // SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
  //
  // SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
  //
  // SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
  //
  // SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
  //
  // SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
  //                            pins are NOT connected to anything by default.
  #ifndef SERIAL_PORT_MONITOR
    #define SERIAL_PORT_MONITOR   Serial
  #endif
  #ifndef SERIAL_PORT_HARDWARE
    #define SERIAL_PORT_HARDWARE  Serial
  #endif
#endif
