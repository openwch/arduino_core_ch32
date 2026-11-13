/**
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

#include "pins_arduino.h"


// Digital pins: index = Arduino pin number
const PinName digitalPin[] = {
  PA_0,   //D0
  PA_1,   //D1
  PA_2,   //D2      USART2_TX
  PA_3,   //D3      USART2_RX
  PA_4,   //D4      SPI1_NSS
  PA_5,   //D5      SPI1_SCK
  PA_10,  //D6      USART1_RX
  PA_9,   //D7      USART1_TX
  PA_8,   //D8
  PA_7,   //D9      SPI1_MOSI
  PA_6,   //D10     SPI1_MISO
  PB_5,   //D11     SPI3_MOSI
  PB_8,   //D12     I2C1_SCL (remap)
  PB_9,   //D13     I2C1_SDA (remap)
  PB_1,   //D14
  PB_0,   //D15
  PB_12,  //D16     SPI2_NSS
  PB_15,  //D17     SPI2_MOSI
  PB_14,  //D18     SPI2_MISO
  PB_13,  //D19     SPI2_SCK
  PB_11,  //D20     USART3_RX
  PB_10,  //D21     USART3_TX
  PA_11,  //D22     USBHS1_DM
  PA_12,  //D23     USBHS1_DP
  PA_15,  //D24     SPI3_NSS
  PB_2,   //D25     BOOT1
  PB_3,   //D26     SPI3_SCK
  PB_4,   //D27     SPI3_MISO
  PB_6,   //D28     I2C1_SCL / USBHS2_DM
  PB_7,   //D29     I2C1_SDA / USBHS2_DP
  PC_0,   //D30     USART6_TX
  PC_1,   //D31     USART6_RX
  PC_2,   //D32     USART7_TX
  PC_3,   //D33     USART7_RX
  PC_4,   //D34     USART8_TX
  PC_5,   //D35     USART8_RX
  PC_6,   //D36
  PC_7,   //D37
  PC_8,   //D38
  PC_9,   //D39
  PC_10,  //D40     USART4_TX
  PC_11,  //D41     USART4_RX
  PC_12,  //D42     USART5_TX
  PC_13,  //D43
  PC_14,  //D44
  PC_15,  //D45
  PD_0,   //D46
  PD_1,   //D47
  PD_2,   //D48     USART5_RX
  PD_3,   //D49
  PD_4,   //D50
  PD_5,   //D51
  PD_6,   //D52
  PD_7,   //D53
  PD_8,   //D54
  PD_9,   //D55
  PD_10,  //D56
  PD_11,  //D57
  PD_12,  //D58
  PD_13,  //D59
  PD_14,  //D60
  PD_15,  //D61
  PE_0,   //D62
  PE_1,   //D63
  PE_2,   //D64
  PE_3,   //D65
  PE_4,   //D66
  PE_5,   //D67
  PE_6,   //D68
  PE_7,   //D69
  PE_8,   //D70
  PE_9,   //D71
  PE_10,  //D72
  PE_11,  //D73
  PE_12   //D74     BOOT0
};

// Analog pins: index = ADC channel, value = digital pin number
const uint32_t analogInputPin[] = {
  0,   // A0,  PA0   ADC_IN0
  1,   // A1,  PA1   ADC_IN1
  2,   // A2,  PA2   ADC_IN2
  3,   // A3,  PA3   ADC_IN3
  4,   // A4,  PA4   ADC_IN4
  5,   // A5,  PA5   ADC_IN5
  10,  // A6,  PA6   ADC_IN6
  9,   // A7,  PA7   ADC_IN7
  15,  // A8,  PB0   ADC_IN8
  14,  // A9,  PB1   ADC_IN9
  30,  // A10, PC0   ADC_IN10
  31,  // A11, PC1   ADC_IN11
  32,  // A12, PC2   ADC_IN12
  33,  // A13, PC3   ADC_IN13
  34,  // A14, PC4   ADC_IN14
  35   // A15, PC5   ADC_IN15
};
