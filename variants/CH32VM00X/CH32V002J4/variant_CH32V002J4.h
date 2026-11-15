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
//#define                         SPI_MODULE_ENABLED
#define                         I2C_MODULE_ENABLED
#define                         TIM_MODULE_ENABLED
#endif  // IDE_MENU_PERIPHERALS  


/* CH32V002J4 Pins, same as V003J4 */
// CH32V002J4M6 pins:  1:PA1=PD6, 3:PA2, 5:PC1/SDA, 6:PC2/SCL, 7:PC4, 8:PD1=PD4=PD5=SWIO
#define PA1                     PIN_A1  // D0          PA1=PD6
#define PA2                     PIN_A0  // D1
#define PC1                     2       // D2/SDA
#define PC2                     3       // D3/SCL
#define PC4                     PIN_A2  // D4
#define PD1                     5       // D5/SWIO/RX' PD1=PD5=PD4
#define PD4                     PIN_A7	// D6          PD1=PD5=PD4
#define PD5                     PIN_A5	// D7          PD1=PD5=PD4
#define PD6                     PIN_A6	// D8/TX'      PA1=PD6

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
// CH32V002 only has USART1.
// VM00X USART1 pinmaps TX/RX: 0=PD5/PD6 _1=PD6/PD5 [_2=PD0/PD1 _3=PC0/PC1_4=PD1/PB3 _5=PB3/PD1 _6=PC5/PC6 _7=PB5/PB6] _8=PA0/PA1, _9=PA0/PC4
// CH32V002J4M6 pins:  1:PA1=PD6, 3:PA2, 5:PC1/SDA, 6:PC2/SCL, 7:PC4, 8:PD1=PD4=PD5=SWIO
// CH32V002J4M6 supported UART1 pinmaps: 0 _1 _8 _9
//    RX:   PD6=PA1
//    TX:   PD5=PD4=PD1=SWIO
//    RX_1: PD5=PD4=PD1=SWIO
//    TX_1: PD6=PA1
//    RX_2: PD0
//    TX_2: PD1
//    RX_3: PC0
//    TX_3: PC1
// If I2C is enabled we cannot use these alternatives but there are no other combinations available.
// Using both I2C/SPI and UART is too much (but flash gets limited then also)
#if defined(SERIAL_UART_ENABLE_ALL)
  #error "CH32V002J4 only supports Serial1; Dual Serial is unavailable"
#endif
#if (defined(SERIAL_UART_INSTANCE) && SERIAL_UART_INSTANCE>1)
  #error "CH32V002J4 only supports Serial1; Serial2 cannot be selected"
#endif

#ifndef BOARD_DEFAULT_SERIAL
  #define BOARD_DEFAULT_SERIAL 1
#endif
#ifndef BOARD_DEFAULT_SERIALMAP
  #define BOARD_DEFAULT_SERIALMAP 1
#endif

#if defined(SERIAL_UART_INSTANCE) && SERIAL_UART_INSTANCE==1 && defined(SERIAL_UART_PINMAP) && (SERIAL_UART_PINMAP>1)
  #error "CH32V002J4 only supports Serial1_1 pin mapping"
#endif


// Default pins for hardware serial port 1.
#ifndef PIN_SERIAL1_RX
  #define PIN_SERIAL1_RX         PD6
#endif
#ifndef PIN_SERIAL1_TX
  #define PIN_SERIAL1_TX         PD5
#endif

// Mapped serial pins
// The VM00X datasheet shows 9 mappings for USART1 and 6 for USART2. Only Serial1_1 is supported on V002J4.
#ifndef PIN_SERIAL1_1_RX
  #define PIN_SERIAL1_1_RX     PD5  // UART1_1 pinmap RX pin
#endif
#ifndef PIN_SERIAL1_1_TX
  #define PIN_SERIAL1_1_TX     PD6  // UART1_1 pinmap TX pin 
#endif



// SPI definitions (CH32V002J4 has no pins for SPI so it's not available)
#if defined(SPI_MODULE_ENABLED)
  #error "CH32V002J4 has no SPI. SPI should be disabled"
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
