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

/* CH32VX033F8P6 Pins */
#define PA0                     PIN_A0
#define PA1                     PIN_A1
#define PA2                     PIN_A2
#define PA3                     PIN_A3
#define PA4                     PIN_A4
#define PA5                     PIN_A5
#define PA6                     PIN_A6
#define PA7                     PIN_A8
#define PB0                     PIN_A8
#define PB1                     PIN_A9
#define PB7                     9       // can be configured as RST
#define PC16                    10
#define PC17                    11
#define PA9                     12
#define PA11                    13
#define PA10                    14
#define PC3                     15      // PIN_A13
#define PC18                    16
#define PC19                    17

// Alternate pins number TODO
#define PA0_ALT1                (PA0  | ALT1)
#define PA1_ALT1                (PA1  | ALT1)
#define PA2_ALT1                (PA2  | ALT1)
#define PA3_ALT1                (PA3  | ALT1)
#define PA4_ALT1                (PA4  | ALT1)
#define PA5_ALT1                (PA5  | ALT1)
#define PA6_ALT1                (PA6  | ALT1)
#define PA7_ALT1                (PA7  | ALT1)
#define PB0_ALT1                (PB0  | ALT1)
#define PB1_ALT1                (PB1  | ALT1)
#define PC0_ALT1                (PC0  | ALT1)
#define PC3_ALT1                (PC3  | ALT1)



#define NUM_DIGITAL_PINS        18      // 27
#define NUM_ANALOG_INPUTS       14      // 14 to allow PIN_13/A13 definition for PC3 (no A10, A11, A12 on X033F8P6)
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
#ifndef PIN_WIRE_SDA
  #define PIN_WIRE_SDA          PC17
#endif
#ifndef PIN_WIRE_SCL
  #define PIN_WIRE_SCL          PC16
#endif

// Timer Definitions
#ifndef TIMER_TONE
  #define TIMER_TONE            TIM3
#endif
#ifndef TIMER_SERVO
  #define TIMER_SERVO           TIM2
#endif


// UART Definitions
#ifndef SERIAL_UART_INSTANCES
  // Define the number of UART instances that can be used.
  // For CH32X033F8P6 SSOP20 the supported maximum is currently two.
  // These are UART1 and UART2 on pins PA10=TX1_1, PA11=RX1_1 and on PA2=TX2, PA3=RX2
  // (two instances will cost 132/136 bytes more flash/ram than one instance)
  #define SERIAL_UART_INSTANCES  2    // select 1 or 2 instances
#endif

#if (SERIAL_UART_INSTANCES==1)
  // If using only one UART inactance, select which to use: UART1 or UART2
  #ifndef SERIAL_UART_INSTANCE
    //  #define SERIAL_UART_INSTANCE  1  // UART1: PA10=TX1_1, PA11=RX1_1
    #define SERIAL_UART_INSTANCE 2  // UART2: PA2=TX2, PA3=RX2
  #endif
#else
  // multiple instances, max 2 for CH32X033F8P SSOP20
  // NOTE: do not define SERIAL_UART_INSTANCE when using multiple instances!
  #undef SERIAL_UART_INSTANCE
  #define ENABLE_HWSERIAL1 1
  #define ENABLE_HWSERIAL2 1
#endif

// Default pin used for generic 'Serial' instance
// Mandatory for Firmata
// For CH32X033F8P6 serial pins RX2=PA3/TX2=PA2 or alternative RX1_1=PA11/TX1_1=PA10

// Pins used for Serial2 instance (used by HardwareSerial constructor)
#if (SERIAL_UART_INSTANCES==1)
  // one single UART instance, specify which oins to be used
  #if (SERIAL_UART_INSTANCE==1)
    // Use UART1 alternative pins RX1_1/TX1_1 (PA11/PA10)
    #ifndef PIN_SERIAL_RX
      #define PIN_SERIAL_RX         PA11
    #endif
    #ifndef PIN_SERIAL_TX
      #define PIN_SERIAL_TX         PA10
    #endif
  #elif (SERIAL_UART_INSTANCE==2)
    // Use UART2 RX2/TX2 (PA3/PA2)
    #ifndef PIN_SERIAL_RX
      #define PIN_SERIAL_RX         PA3
    #endif
    #ifndef PIN_SERIAL_TX
      #define PIN_SERIAL_TX         PA2
    #endif
  #endif
#else
  // multiple instances. Define each pin for each UART (Serial=Serial2)
  #define Serial Serial2  // specify which UART to use as 'Serial'
  #ifndef PIN_SERIAL_RX
    #define PIN_SERIAL_RX         PA11   // supported: PA3=RX2, alternative PA11=RX1_1 (X035: PB11)
  #endif
  #ifndef PIN_SERIAL_TX
    #define PIN_SERIAL_TX         PA10   // supported:  PA2=TX2, alternative PA10=TX1_1 (X035: PB10)
  #endif
  #ifndef PIN_SERIAL_RX2
    #define PIN_SERIAL_RX2         PA3   // supported: PA3=RX2, alternative PA11=RX1_1 (X035: PB11)
  #endif
  #ifndef PIN_SERIAL_TX2
    #define PIN_SERIAL_TX2         PA2   // supported:  PA2=TX2, alternative PA10=TX1_1 (X035: PB10)
  #endif
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


