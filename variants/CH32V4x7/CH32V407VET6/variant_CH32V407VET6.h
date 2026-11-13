/*
 *******************************************************************************
 * Copyright (c) 2026 Nanjing Qinheng Microelectronics Co., Ltd.
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
#define                         UART_MODULE_ENABLED
#define                         SPI_MODULE_ENABLED
#define                         I2C_MODULE_ENABLED
#define                         ADC_MODULE_ENABLED
#define                         TIM_MODULE_ENABLED

/*
 * CH32V407VET6 (LQFP100). D0-D21 follow the CH32V307VCT6 variant, the other
 * GPIOs follow in port order. PA13/PA14 (SWD) and PE13-PE15 are not included.
 * A0-A15 are ADC_IN0-ADC_IN15; analogRead(PA7) and analogRead(A7) both work.
 */
#define PA0                     PIN_A0   // D0   ADC_IN0
#define PA1                     PIN_A1   // D1   ADC_IN1
#define PA2                     PIN_A2   // D2   ADC_IN2
#define PA3                     PIN_A3   // D3   ADC_IN3
#define PA4                     PIN_A4   // D4   ADC_IN4
#define PA5                     PIN_A5   // D5   ADC_IN5
#define PA10                    6
#define PA9                     7
#define PA8                     8
#define PA7                     PIN_A7   // D9   ADC_IN7
#define PA6                     PIN_A6   // D10  ADC_IN6
#define PB5                     11
#define PB8                     12
#define PB9                     13
#define PB1                     PIN_A9   // D14  ADC_IN9
#define PB0                     PIN_A8   // D15  ADC_IN8
#define PB12                    16
#define PB15                    17
#define PB14                    18
#define PB13                    19
#define PB11                    20
#define PB10                    21
#define PA11                    22  // USBHS1_DM
#define PA12                    23  // USBHS1_DP
#define PA15                    24
#define PB2                     25  // BOOT1
#define PB3                     26
#define PB4                     27
#define PB6                     28  // USBHS2_DM
#define PB7                     29  // USBHS2_DP
#define PC0                     PIN_A10  // D30  ADC_IN10
#define PC1                     PIN_A11  // D31  ADC_IN11
#define PC2                     PIN_A12  // D32  ADC_IN12
#define PC3                     PIN_A13  // D33  ADC_IN13
#define PC4                     PIN_A14  // D34  ADC_IN14
#define PC5                     PIN_A15  // D35  ADC_IN15
#define PC6                     36
#define PC7                     37
#define PC8                     38
#define PC9                     39
#define PC10                    40
#define PC11                    41
#define PC12                    42
#define PC13                    43
#define PC14                    44  // OSC32_IN
#define PC15                    45  // OSC32_OUT
#define PD0                     46
#define PD1                     47
#define PD2                     48
#define PD3                     49
#define PD4                     50
#define PD5                     51
#define PD6                     52
#define PD7                     53
#define PD8                     54
#define PD9                     55
#define PD10                    56
#define PD11                    57
#define PD12                    58
#define PD13                    59
#define PD14                    60
#define PD15                    61
#define PE0                     62
#define PE1                     63
#define PE2                     64
#define PE3                     65
#define PE4                     66
#define PE5                     67
#define PE6                     68
#define PE7                     69
#define PE8                     70  // ETH LED0
#define PE9                     71  // ETH LED1
#define PE10                    72
#define PE11                    73
#define PE12                    74  // BOOT0

#define NUM_DIGITAL_PINS        75
#define NUM_ANALOG_INPUTS       16

// analogRead() is 12-bit (0-4095), analogWrite() is 8-bit (0-255) at 1 kHz
#define ADC_RESOLUTION          12
#define PWM_RESOLUTION          8
#define PWM_FREQUENCY           1000

// Timers with no PWM pins, used by tone() and Servo
#ifndef TIMER_TONE
  #define TIMER_TONE            TIM6
#endif
#ifndef TIMER_SERVO
  #define TIMER_SERVO           TIM7
#endif

// LED1 on the EVT: connect it to PA0 with a jumper (active low)
#ifndef LED_BUILTIN
  #define LED_BUILTIN           PA0
#endif

// On-board user button
#ifndef USER_BTN
  #define USER_BTN              PNUM_NOT_DEFINED
#endif

// SPI definitions (SPI1)
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

// I2C1 on PB8/PB9 (remapped, as in the WCH EVT examples)
#ifndef PIN_WIRE_SDA
  #define PIN_WIRE_SDA          PB9
#endif
#ifndef PIN_WIRE_SCL
  #define PIN_WIRE_SCL          PB8
#endif

// USART1 (PA9/PA10) goes to the WCH-Link serial port
#ifndef SERIAL_UART_INSTANCE
  #define SERIAL_UART_INSTANCE  1
#endif
// Default pin used for generic 'Serial' instance
#ifndef PIN_SERIAL_RX
  #define PIN_SERIAL_RX         PA10
#endif
#ifndef PIN_SERIAL_TX
  #define PIN_SERIAL_TX         PA9
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
  #ifndef SERIAL_PORT_MONITOR
    #define SERIAL_PORT_MONITOR   Serial
  #endif
  #ifndef SERIAL_PORT_HARDWARE
    #define SERIAL_PORT_HARDWARE  Serial
  #endif
#endif
