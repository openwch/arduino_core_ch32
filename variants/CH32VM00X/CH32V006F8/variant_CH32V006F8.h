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

/* CH32V006F Pins */
#define PA1                     PIN_A1
#define PA2                     PIN_A0
#define PC0                     2
#define PC1                     3
#define PC2                     4
#define PC3                     5 
#define PC4                     PIN_A2
#define PC5                     7
#define PC6                     8
#define PC7                     9
#define PD0                     10
#define PD1                     11
#define PD2                     PIN_A3
#define PD3                     PIN_A4 
#define PD4                     PIN_A7
#define PD5                     PIN_A5
#define PD6                     PIN_A6 
#define PD7                     17

/*
#define PA0                     0
#define PA1                     PIN_A1
#define PA2                     PIN_A0
#define PA3                     3
#define PA4                     4 
#define PA5                     5
#define PA6                     6
#define PA7                     7 
#define PB0                     8    
#define PB1                     9
#define PB2                     10
#define PB3                     11 
#define PB4                     12 
#define PB5                     13
#define PB6                     14 
#define PC0                     15
#define PC1                     16
#define PC2                     17
#define PC3                     18
#define PC4                     PIN_A2
#define PC5                     20
#define PC6                     21
#define PC7                     22
#define PD0                     23
#define PD1                     24
#define PD2                     PIN_A3
#define PD3                     PIN_A4 
#define PD4                     PIN_A7
#define PD5                     PIN_A5
#define PD6                     PIN_A6 
#define PD7                     30
*/

// Alternate pins number
#define PD5_ALT1                (PD5  | ALT1)
#define PD6_ALT1                (PD6  | ALT1)


#define NUM_DIGITAL_PINS        18
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
#ifndef SERIAL_UART_INSTANCES
  // Define the number of UART instances that can be used.
  // For CH32V006F8 SSOP20 the supported maximum is currently two.
  // These are UART1 and UART2 on pins PD5=TX1, PD6=RX1 and on PD2=TX2, PD3=RX2
  #define SERIAL_UART_INSTANCES  2    // select 1 or 2 instances
#endif

// TSSOP20 UART2 has RX/TX as alternative functions U2RX_/U2TX_ on pins 20/19 (PD3/PD2)
#if (SERIAL_UART_INSTANCES==1)
  // If using only one UART inactance, select which to use: UART1 or UART2
  #ifndef SERIAL_UART_INSTANCE
    #define SERIAL_UART_INSTANCE  1
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
// For CH32V006F8 serial pins RX1=PD6/TX1=PD5 or alternative U2RX_/U2TX_ on pins 20/19 (PD3/PD2)

// Pins used for Serial2 instance (used by HardwareSerial constructor)
#if (SERIAL_UART_INSTANCES==1)
  // one single UART instance, specify which pins to be used
  #if (SERIAL_UART_INSTANCE==1)
    #ifndef PIN_SERIAL_RX
      #define PIN_SERIAL_RX         PD6
    #endif
    #ifndef PIN_SERIAL_TX
      #define PIN_SERIAL_TX         PD5
    #endif
  #elif (SERIAL_UART_INSTANCE==2)
    // Use UART2 RX2/TX2 (PD3/PD2)
    #ifndef PIN_SERIAL_RX
      #define PIN_SERIAL_RX         PD3
    #endif
    #ifndef PIN_SERIAL_TX
      #define PIN_SERIAL_TX         PD2
    #endif
  #endif //  #if (SERIAL_UART_INSTANCE==1)
#else
  // multiple instances. Define each pin for each UART (Serial=Serial1)
  #define Serial Serial1  // specify which UART to use as 'Serial'
  #ifndef PIN_SERIAL_RX
    #define PIN_SERIAL_RX           PD6   // supported: PD6=RX1 (not supported alternatives PD5/PD1/PC6/PC1)
  #endif
  #ifndef PIN_SERIAL_TX
    #define PIN_SERIAL_TX           PD5   // supported:  PD5=TX1 (not supported alternatives PD6/PC5/PC0/PD0)
  #endif
  #ifndef PIN_SERIAL_RX2
    #define PIN_SERIAL_RX2          PD3   // supported: PD3=RX2_
  #endif
  #ifndef PIN_SERIAL_TX2
    #define PIN_SERIAL_TX2          PD2   // supported:  PD2=TX2_/USART2_TX_3 (not supported alternative PA2=USART2_TX_2/PC4=USART2_TX_5/PD7=PA4=USART2_TX_1)
    // Datasheet Note 4: For CH32V006F8U6, CH32V006F8P6, CH32V005F6U6, CH32V005F6P6 and CH32V005D6U6 chips,
    // the PA4 and PD7 pins are short-jointed and sealed inside the chip, and it is forbidden that both of the two IMAGO
    // are configured as output functions. Note 5: PD7 is the reset pin.
  #endif
#endif // #if (SERIAL_UART_INSTANCES==1)


// SPI definitions
#ifndef PIN_SPI_SS
  #define PIN_SPI_SS            PC4
#endif
#ifndef PIN_SPI_MOSI
  #define PIN_SPI_MOSI          PC6
#endif
#ifndef PIN_SPI_MISO
  #define PIN_SPI_MISO          PC7
#endif
#ifndef PIN_SPI_SCK
  #define PIN_SPI_SCK           PC5
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


