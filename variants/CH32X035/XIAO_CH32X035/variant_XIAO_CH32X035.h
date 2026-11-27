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
 * Seeed Studio XIAO CH32X035 (CH32X035G8U6, QFN28)
 * The Dn numbers below refer to the core-generated digital pin enum,
 * i.e. the position in digitalPin[] (see variant_XIAO_CH32X035.cpp).
 * D13-D17 are test pads; D15 (PB1) is bonded to PB5 on the QFN28 package.
 * PC14/PC15 (USB CC) and PC16/PC17 (USB DM/DP) are not broken out on the
 * XIAO edge castellation.
 *******************************************************************************
 */
#pragma once

/* ENABLE Peripherals */
#define                         ADC_MODULE_ENABLED
#define                         UART_MODULE_ENABLED
#define                         SPI_MODULE_ENABLED
#define                         I2C_MODULE_ENABLED
#define                         TIM_MODULE_ENABLED

/* XIAO silk name to chip pin:
 * D0/A0=PA0, D1/A1=PA1, D2/A2=PA2, D3/A3=PA3, D4/SDA=PC19, D5/SCL=PC18,
 * D6/TX=PB10, D7/RX=PB11, D8/SCK=PA5, D9/MISO=PA6, D10/MOSI=PA7,
 * D11/A4=PC3, D12/A5=PB0, D13/SS=PB3, D18=User LED, D19=RGB DIN,
 * D20=BAT_EN, D21=user button, D22/A6=battery sense */

/* Port name aliases, ordered by the digital pin number they resolve to */
#define PA0                     PIN_A0  /* D0,  A0  */
#define PA1                     PIN_A1  /* D1,  A1  */
#define PA2                     PIN_A2  /* D2,  A2  */
#define PA3                     PIN_A3  /* D3,  A3  */
#define PC19                    4       /* D4,  SDA  */
#define PC18                    5       /* D5,  SCL  */
#define PB10                    6       /* D6,  TX   */
#define PB11                    7       /* D7,  RX   */
#define PA5                     8       /* D8,  SCK  */
#define PA6                     9       /* D9,  MISO */
#define PA7                     10      /* D10, MOSI */
#define PC3                     PIN_A4  /* D11, A4   */
#define PB0                     PIN_A5  /* D12, A5   */
#define PB3                     13      /* D13, SS   */
#define PB4                     14      /* D14 (TP4) */
#define PB1                     15      /* D15 (TP9, bonded to PB5) */
#define PB6                     16      /* D16 (TP10) */
#define PB7                     17      /* D17 (TP11) */
#define PA4                     18      /* D18, User LED (active LOW) */
#define PB8                     19      /* D19, RGB DIN */
#define PB9                     20      /* D20, BAT_EN (active HIGH) */
#define PB12                    21      /* D21, user button K1 (pressed LOW) */
#define PC0                     PIN_A6  /* D22, A6, battery sense (1/2 divider, enable via D20) */

// Alternate pins number
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

#define NUM_DIGITAL_PINS        23
#define NUM_ANALOG_INPUTS       7
#define ADC_RESOLUTION          12

// On-board LED pin number (driven from 3V3 through the LED: LOW = on)
#ifndef LED_BUILTIN
  #define LED_BUILTIN           PA4
#endif

// On-board user button (K1, external 10K pull-up: LOW = pressed)
#ifndef USER_BTN
  #define USER_BTN              PB12
#endif

// RGB LED data-in
#ifndef PIN_NEOPIXEL
  #define PIN_NEOPIXEL          PB8
#endif

// SPI definitions
#ifndef PIN_SPI_SS
  #define PIN_SPI_SS            PB3
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

// I2C definitions (I2C1 full remap: SDA=PC19, SCL=PC18)
#ifndef PIN_WIRE_SDA
  #define PIN_WIRE_SDA          PC19
#endif
#ifndef PIN_WIRE_SCL
  #define PIN_WIRE_SCL          PC18
#endif

// Timer Definitions
#ifndef TIMER_TONE
  #define TIMER_TONE            TIM3
#endif
#ifndef TIMER_SERVO
  #define TIMER_SERVO           TIM2
#endif

// UART Definitions
#ifndef SERIAL_UART_INSTANCE
  #define SERIAL_UART_INSTANCE  1
#endif
// Default pin used for generic 'Serial' instance
// Mandatory for Firmata
#ifndef PIN_SERIAL_RX
  #define PIN_SERIAL_RX         PB11
#endif
#ifndef PIN_SERIAL_TX
  #define PIN_SERIAL_TX         PB10
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
  // These serial port names are intended to allow libraries and architecture-neutral
  // sketches to automatically default to the correct port name for a particular type
  // of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
  // the first hardware serial port whose RX & TX pins are not dedicated to another use.
  //
  // SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
  //
  // SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
  //
  // SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
  //
  // SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
  //                            pins are NOT connected to anything by default.
  #ifndef SERIAL_PORT_MONITOR
    #define SERIAL_PORT_MONITOR   Serial
  #endif
  #ifndef SERIAL_PORT_HARDWARE
    #if defined(CH32X035_USB_CDC)
      #define SERIAL_PORT_HARDWARE Serial1
    #else
      #define SERIAL_PORT_HARDWARE Serial
    #endif
  #endif
  #if defined(CH32X035_USB_CDC) && !defined(SERIAL_PORT_USBVIRTUAL)
    #define SERIAL_PORT_USBVIRTUAL SerialUSB
  #endif
#endif
