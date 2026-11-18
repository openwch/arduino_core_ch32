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
#ifndef IDE_MENU_PERIPHERALS   // MxRD: defined when peripherals are enabled/disabled via the IDE menu
#define                         ADC_MODULE_ENABLED
#define                         UART_MODULE_ENABLED
//#define                         SPI_MODULE_ENABLED  // No SPI SCK on V003J4, SPI should be disabled
#define                         I2C_MODULE_ENABLED
#define                         TIM_MODULE_ENABLED
#endif

/* CH32V003J4 Pins */
#define PA1                     PIN_A1  // D0
#define PA2                     PIN_A0  // D1
#define PC1                     2       // D2/SDA
#define PC2                     3       // D3/SCL
#define PC4                     PIN_A2  // D4
#define PD1                     5       // D5/SWIO
#define PD4                     PIN_A7	// D6
#define PD5                     PIN_A5	// D7
#define PD6                     PIN_A6	// D8/TX'

// Make sure A3/A4 are unknown enums, to generate errors when used.
// (see analogInputPin[] in variant_...cpp and pins_arduino_analog.h)
#define VARIANT_NO_ANALOG_A3
#define VARIANT_NO_ANALOG_A4

// Alternate pin numbers
#define PC4_ALT1              (PC4  | ALT1)
#define PD1_ALT1              (PD1  | ALT1)

#define PA1_ALT2              (PA1  | ALT2)
#define PA2_ALT2              (PA2  | ALT2)
#define PC1_ALT2              (PC1  | ALT2)
#define PC4_ALT2              (PC4  | ALT2)
#define PD1_ALT2              (PD1  | ALT2)

#define PC4_ALT3              (PC4  | ALT3)
#define PC1_ALT3              (PC1  | ALT3)
#define PD4_ALT3              (PD4  | ALT3)


#define NUM_DIGITAL_PINS        9
#define NUM_ANALOG_INPUTS       8

// #define ADC_CTLR_ADCAL          
#define ADC_RESOLUTION          10



// On-board LED pin number
#ifndef LED_BUILTIN
  #define LED_BUILTIN           PNUM_NOT_DEFINED
#endif



// On-board user button
#ifndef USER_BTN
  #define USER_BTN              PNUM_NOT_DEFINED
#endif



/*
// UART Definitions
#ifndef SERIAL_UART_INSTANCE
  #define SERIAL_UART_INSTANCE  1
#endif
// Default pin used for generic 'Serial' instance
// Mandatory for Firmata
#ifndef PIN_SERIAL_RX
  #define PIN_SERIAL_RX         PD6
#endif
#ifndef PIN_SERIAL_TX
  #define PIN_SERIAL_TX         PD5
#endif
*/

// UART Definitions
// CH32V003 only has USART1. CH32V003J4M6 SOP8 pins:
//    RX: PD6=RX1
//    TX: PD5=TX1=SWIO
// CH32V003J4M6 SOP16 has alternative pins RX/TX: RX_2/TX_2=PD5/PD6,  RX_3=PC1 TX_3 (PC0) is not available
// TX_3 conflicts with I2C SDA and with SPI SS 
#ifndef BOARD_DEFAULT_SERIAL
  #define BOARD_DEFAULT_SERIAL 1
#endif
#ifndef BOARD_DEFAULT_SERIALMAP
  #define BOARD_DEFAULT_SERIALMAP 2
#endif

#if defined(SERIAL_UART_PINMAP) && SERIAL_UART_PINMAP==3
  #error "CH32V003J4 Serial1_3 is not available"
#endif

// Default pins for hardware serial port 1.
#ifndef PIN_SERIAL1_RX
  #define PIN_SERIAL1_RX         PD6
#endif
#ifndef PIN_SERIAL1_TX
  #define PIN_SERIAL1_TX         PD5
#endif

// Mapped serial pins. 
// For V003J4 (and V002J4) these alternate pins allow using TX without blocking SWIO.
#ifndef PIN_SERIAL1_2_RX
  #define PIN_SERIAL1_2_RX     PD5  // UART1_2 pinmap RX pin
#endif
#ifndef PIN_SERIAL1_2_TX
  #define PIN_SERIAL1_2_TX     PD6  // UART1_2 pinmap TX pin 
#endif

// SPI definitions (CH32V003J4 has no pins for SPI so it's not available)
#if defined(SPI_MODULE_ENABLED)
  #error "CH32V003J4 has no SPI. SPI should be disabled"
#endif

// I2C definitions
#ifndef PIN_WIRE_SDA
  #define PIN_WIRE_SDA          PC1
#endif
#ifndef PIN_WIRE_SCL
  #define PIN_WIRE_SCL          PC2
#endif

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
