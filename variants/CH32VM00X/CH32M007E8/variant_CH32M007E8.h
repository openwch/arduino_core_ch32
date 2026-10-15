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
#define                         SPI_MODULE_ENABLED
#define                         I2C_MODULE_ENABLED
#define                         TIM_MODULE_ENABLED
#endif

#if defined(SPI_MODULE_ENABLED)
  #define SPI_INTERFACES_COUNT 1  // MMOLE: used by libraries such as Adafruit BusIO. Should be defined in some variants.h file somewhere
#endif

/* CH32M007E Pins */
#define PA0                     7
#define PA1                     PIN_A1
#define PA2                     PIN_A0
#define PA3                     6
#define PA4                     4
#define PA5                     5
#define PB0                     8
#define PB1                     10
#define PB3                     12
//#define PC0                     19
#define PC1                     13
#define PC2                     14
#define PC4                     PIN_A2
#define PC5                     16
#define PD0                     11
#define PD1                     18
#define PD2                     PIN_A3
#define PD3                     PIN_A4 
#define PD4                     PIN_A7
#define PD5                     PIN_A5
#define PD6                     PIN_A6 
#define PD7                     3


// Alternate pins number
//#define PD5_ALT1                (PD5  | ALT1)
//#define PD6_ALT1                (PD6  | ALT1)


#define NUM_DIGITAL_PINS        21
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
// TODO: M007E8R6 QSOP24 UART2 has no PA7 pin. It has RX/TX as alternative functions UART2_1 U2RX_/U2TX_ on pins PA4/PA5)
// CH32V007:
//    RX: PD6=RX1, PB3=RX2, PA5=RX2_1
//    TX: PD5=TX1, PA7=TX2, PA4=TX2_1

#ifndef SERIAL_UART_INSTANCES
  #define SERIAL_UART_INSTANCES 1
#endif

#if (SERIAL_UART_INSTANCES == 1)
  //  Using only one Serial port
  #ifndef SERIAL_UART_INSTANCE
    #define SERIAL_UART_INSTANCE 1   // use Serial1 as default for Serial
  #endif
#else
  // Using multple Serial ports
  // When having multiple Serial instances SERIAL_UART_INSTANCE cannot be defined as it will skip defining the other ports.
  #undef SERIAL_UART_INSTANCE
  #define ENABLE_HWSERIAL1 1
  #define ENABLE_HWSERIAL2 1
  #define Serial Serial1
#endif

// Default pin used for generic 'Serial' instance
// Mandatory for Firmata, no longer used by HardwareSerial.
#if SERIAL_UART_INSTANCE==1 || SERIAL_UART_INSTANCES>1
    #ifndef PIN_SERIAL_RX
        #define PIN_SERIAL_RX     PD6
    #endif
    #ifndef PIN_SERIAL_TX
        #define PIN_SERIAL_TX     PD5
    #endif
#elif  SERIAL_UART_INSTANCE==2
    #ifndef PIN_SERIAL_RX
      #define PIN_SERIAL_RX       PA5
    #endif
    #ifndef PIN_SERIAL_TX
      #define PIN_SERIAL_TX       PA4
    #endif
#endif


// SPI definitions
#ifndef PIN_SPI_SS
  #define PIN_SPI_SS            PC4
#endif
#ifndef PIN_SPI_MOSI
  #define PIN_SPI_MOSI          PD3
#endif
#ifndef PIN_SPI_MISO
  #define PIN_SPI_MISO          PB3
#endif
#ifndef PIN_SPI_SCK
  #define PIN_SPI_SCK           PD2
#endif

// I2C definitions
#ifndef PIN_WIRE_SDA
  #define PIN_WIRE_SDA          PC1
#endif
#ifndef PIN_WIRE_SCL
  #define PIN_WIRE_SCL          PC2
#endif

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


