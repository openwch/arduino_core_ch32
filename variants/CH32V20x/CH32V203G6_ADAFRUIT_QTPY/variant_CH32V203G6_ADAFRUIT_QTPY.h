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
#define                         UART_MODULE_ENABLED
#define                         SPI_MODULE_ENABLED  
#define                         I2C_MODULE_ENABLED
#define                         TIM_MODULE_ENABLED

/* CH32V203G6 Pins */
#define PA0                     3
#define PA1                     2
#define PA2                     4
#define PA3                     5
#define PA4                     6
#define PA5                     7
#define PA6                     8
#define PA7                     9
#define PA8                     PNUM_NOT_DEFINED
#define PA9                     PNUM_NOT_DEFINED
#define PA10                    PNUM_NOT_DEFINED
#define PA11                    PNUM_NOT_DEFINED
#define PA12                    PNUM_NOT_DEFINED
#define PA13                    PNUM_NOT_DEFINED
#define PA14                    PNUM_NOT_DEFINED
#define PA15                    PNUM_NOT_DEFINED
#define PB0                     PIN_A1
#define PB1                     PIN_A0
#define PB2                     PNUM_NOT_DEFINED
#define PB3                     PNUM_NOT_DEFINED
#define PB4                     PNUM_NOT_DEFINED
#define PB5                     PNUM_NOT_DEFINED
#define PB6                     10
#define PB7                     11
#define PB8                     12

// Alternate pins number
#define PA0_ALT1                (PA0  | ALT1)
#define PA1_ALT1                (PA1  | ALT1)
#define PA2_ALT1                (PA2  | ALT1)
#define PA3_ALT1                (PA3  | ALT1)
#define PA4_ALT1                (PA4  | ALT1)
#define PA5_ALT1                (PA5  | ALT1)
#define PA6_ALT1                (PA6  | ALT1)
#define PA7_ALT1                (PA7  | ALT1)
#define PA8_ALT1                (PA8  | ALT1)
#define PA9_ALT1                (PA9  | ALT1)
#define PA10_ALT1               (PA10 | ALT1)
#define PA11_ALT1               (PA11 | ALT1)
#define PA15_ALT1               (PA15 | ALT1)
#define PB0_ALT1                (PB0  | ALT1)
#define PB0_ALT2                (PB0  | ALT2)
#define PB1_ALT1                (PB1  | ALT1)
#define PB1_ALT2                (PB1  | ALT2)
#define PB3_ALT1                (PB3  | ALT1)
#define PB10_ALT1               (PB10 | ALT1)
#define PB11_ALT1               (PB11 | ALT1)

#define NUM_DIGITAL_PINS        13
#define NUM_ANALOG_INPUTS       4
// #define ADC_CTLR_ADCAL          
#define ADC_RESOLUTION          12


// On-board LED pin number
#ifndef LED_BUILTIN
  #define LED_BUILTIN           PNUM_NOT_DEFINED
#endif

// On-board user button
#ifndef USER_BTN
  #define USER_BTN              PB8
#endif

// SPI definitions
#ifndef PIN_SPI_SS
  #define PIN_SPI_SS            PNUM_NOT_DEFINED
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
#ifndef PIN_WIRE_SDA
  #define PIN_WIRE_SDA          PB7
#endif
#ifndef PIN_WIRE_SCL
  #define PIN_WIRE_SCL          PB6
#endif

// Timer Definitions
// Use TIM6/TIM7 when possible as servo and tone don't need GPIO output pin
#ifndef TIMER_TONE
  #define TIMER_TONE            TIM3
#endif
#ifndef TIMER_SERVO
  #define TIMER_SERVO           TIM2
#endif


// UART Definitions
// CH32V203: Serial3/4 only on C8/RB, F6 has only Serial2
// V203G6U6 (QFN28): 2xUSART - RX/TX/RTS/CTS/CK: USART2:PA3/PA2/PA1/PA0/PA4 USART1: PA10/PA9/PA12/PA11/PA8
//    RX: PA10=RX1, PA3=RX2, PB11=RX3, PB1=RX4
//    TX: PA9=TX1, PA2=TX2, PB10=TX3, PB0=TX4
// The QTPY is a very small board with only few pins: https://learn.adafruit.com/adafruit-qt-py-ch32v203/pinouts
// TODO: It only exposes UART2 and UART4. These don't match V203G6 datasheet which shows USART1 and USART2.
// V203G6 LANA also shows UART4: TX4=PB0, RX4=PB1. Correct pinout should be confirmed!
#ifndef SERIAL_UART_INSTANCES
  #define SERIAL_UART_INSTANCES 1
#endif

#if (SERIAL_UART_INSTANCES == 1)
  //  Using only one Serial port
  #ifndef SERIAL_UART_INSTANCE
    #define SERIAL_UART_INSTANCE 2   // use Serial2 as default for Serial
  #endif
#else
  // Using multple Serial ports
  // When having multiple Serial instances SERIAL_UART_INSTANCE cannot be defined as it will skip defining the other ports.
  // V203G6U6 (QFN28): only Serial1/Serial2, no Serial3/Serial4
  #undef SERIAL_UART_INSTANCE
  #define ENABLE_HWSERIAL1 1
  #define ENABLE_HWSERIAL2 1
  #undef SERIAL_UART_INSTANCES
  #define SERIAL_UART_INSTANCES 2
  #define Serial Serial2
#endif

// Default pin used for generic 'Serial' instance
// Mandatory for Firmata, no longer used by HardwareSerial.
#if SERIAL_UART_INSTANCE==1 || SERIAL_UART_INSTANCES>1
    #ifndef PIN_SERIAL_RX
        #define PIN_SERIAL_RX     PA10
    #endif
    #ifndef PIN_SERIAL_TX
        #define PIN_SERIAL_TX     PA9
    #endif
#elif  SERIAL_UART_INSTANCE==2
    #ifndef PIN_SERIAL_RX
      #define PIN_SERIAL_RX       PA3
    #endif
    #ifndef PIN_SERIAL_TX
      #define PIN_SERIAL_TX       PA2
    #endif
#elif  SERIAL_UART_INSTANCE==3
    #error No Serial3 on CH32V203G6
#elif  SERIAL_UART_INSTANCE==4
    #error No Serial4 on CH32V203G6
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


