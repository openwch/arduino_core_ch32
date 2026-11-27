/**
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

#include "pins_arduino.h"




// Digital PinName array
const PinName digitalPin[] = {

  // --- PORT A ---

  PA_0,   // ADC0, TIM2_CH1, WKUP
  PA_1,   // ADC1, TIM2_CH2
  PA_2,   // ADC2, TIM2_CH3, USART2_TX
  PA_3,   // ADC3, TIM2_CH4, USART2_RX
  PA_4,   // ADC4, SPI1_NSS, DAC_OUT1
  PA_5,   // ADC5, SPI1_SCK, DAC_OUT2
  PA_6,   // ADC6, SPI1_MISO, TIM3_CH1
  PA_7,   // ADC7, SPI1_MOSI, TIM3_CH2
  PA_8,   // MCO, TIM1_CH1, USART1_CK
  PA_9,   // USART1_TX, TIM1_CH2
  PA_10,  // USART1_RX, TIM1_CH3
  PA_11,  // USB_DM, CAN_RX
  PA_12,  // USB_DP, CAN_TX
  PA_15,  // TIM2_CH1_ETR, SPI1_NSS (remap), JTDI

  // --- PORT B ---

  PB_0,   // ADC8, TIM3_CH3
  PB_1,   // ADC9, TIM3_CH4
  PB_2,   // BOOT1
  PB_3,   // JTDO, SPI1_SCK (remap), TIM2_CH2
  PB_4,   // NJTRST, SPI1_MISO (remap), TIM3_CH1
  PB_5,   // SPI1_MOSI (remap), TIM3_CH2
  PB_6,   // I2C1_SCL, USART1_TX (remap), TIM4_CH1
  PB_7,   // I2C1_SDA, USART1_RX (remap), TIM4_CH2
  PB_8,   // TIM4_CH3, I2C1_SCL (remap), CAN_RX
  PB_9,   // TIM4_CH4, I2C1_SDA (remap), CAN_TX
  PB_10,  // I2C2_SCL, USART3_TX
  PB_11,  // I2C2_SDA, USART3_RX
  PB_12,  // SPI2_NSS, I2C2_SMBA
  PB_13,  // SPI2_SCK, USART3_CTS
  PB_14,  // SPI2_MISO, USART3_RTS
  PB_15,  // SPI2_MOSI

  // --- PORT C ---

  PC_13,  // RTC_TAMPER, GPIO
  PC_14,  // OSC32_IN (LSE)
  PC_15   // OSC32_OUT (LSE)
};

// Analog (Ax) pin number array
const uint32_t analogInputPin[] = {
  0,  // A0,  PA0
  1,  // A1,  PA1
  2,  // A2,  PA2
  3,  // A3,  PA3
  4,  // A4,  PA4
  5   // A5,  PA5
};



