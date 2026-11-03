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
#define                         ADC_MODULE_ENABLED
// #define                         UART_MODULE_ENABLED
// #define                         SPI_MODULE_ENABLED  
// #define                         I2C_MODULE_ENABLED
// #define                         TIM_MODULE_ENABLED

/* CH585 Pins */

#define PA0                     PIN_A9
#define PA1                     PIN_A8
#define PA2                     PIN_A7
#define PA3                     PIN_A6
#define PA4                     PIN_A0
#define PA5                     PIN_A1
#define PA6                     PIN_A10
#define PA7                     PIN_A11
#define PA8                     PIN_A12
#define PA9                     PIN_A13
#define PA10                    10
#define PA11                    11
#define PA12                    PIN_A2
#define PA13                    PIN_A3
#define PA14                    PIN_A4
#define PA15                    PIN_A5
#define PB0                     16
#define PB1                     17
#define PB2                     18
#define PB3                     19
#define PB4                     20
#define PB5                     21
#define PB6                     22
#define PB7                     23
#define PB8                     24
#define PB9                     25
#define PB10                    26
#define PB11                    27
#define PB12                    28
#define PB13                    29
#define PB14                    30
#define PB15                    31
#define PB16                    32
#define PB17                    33
#define PB18                    34
#define PB19                    35
#define PB20                    36
#define PB21                    37
#define PB22                    38
#define PB23                    39


// Alternate pins number
// #define PA0_ALT1                (PA0  | ALT1) 
// #define PA1_ALT1                (PA1  | ALT1)
// #define PA2_ALT1                (PA2  | ALT1)
// #define PA3_ALT1                (PA3  | ALT1)
// #define PA4_ALT1                (PA4  | ALT1)
// #define PA5_ALT1                (PA5  | ALT1)


#define NUM_DIGITAL_PINS        40
#define NUM_ANALOG_INPUTS       14
#define ADC_RESOLUTION          12


// On-board LED pin number
#ifndef LED_BUILTIN
  #define LED_BUILTIN           PNUM_NOT_DEFINED
#endif



// On-board user button
#ifndef USER_BTN
  #define USER_BTN              PNUM_NOT_DEFINED
#endif

// SPI definitions
#ifndef PIN_SPI_SS
  #define PIN_SPI_SS            PA4
#endif
#ifndef PIN_SPI_SS1
  #define PIN_SPI_SS1           PNUM_NOT_DEFINED
#endif
#ifndef PIN_SPI_SS2
  #define PIN_SPI_SS2           PNUM_NOT_DEFINED
#endif
#ifndef PIN_SPI_SS3
  #define PIN_SPI_SS3           PNUM_NOT_DEFINED
#endif

#ifndef PIN_SPI_MOSI
  #define PIN_SPI_MOSI          PA7
#endif
#ifndef PIN_SPI_MISO
  #define PIN_SPI_MISO          PA6
#endif
#ifndef PIN_SPI_SCK
  #define PIN_SPI_SCK           PA5
#endif

// I2C definitions
  #define PIN_WIRE_SDA          PNUM_NOT_DEFINED
  #define PIN_WIRE_SCL          PNUM_NOT_DEFINED

// Timer Definitions
// Use TIM6/TIM7 when possible as servo and tone don't need GPIO output pin
#ifndef TIMER_TONE
  #define TIMER_TONE            TIM3
#endif
#ifndef TIMER_SERVO
  #define TIMER_SERVO           TIM2
#endif


// UART Definitions
#ifndef SERIAL_UART_INSTANCE
  #define SERIAL_UART_INSTANCE  1
  #define HAVE_HWSERIAL1
  #define HAVE_HWSERIAL2
  #define HAVE_HWSERIAL3
  #define HAVE_HWSERIAL4
#endif
// Default pin used for generic 'Serial' instance
// Mandatory for Firmata
// USE UART1 as it is available on all packages
#ifndef PIN_SERIAL_RX
  #define PIN_SERIAL_RX         PA2
#endif
#ifndef PIN_SERIAL_TX
  #define PIN_SERIAL_TX         PA3
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


