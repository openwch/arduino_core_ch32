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
#include "hw_config.h"

// Digital PinName array
const PinName digitalPin[] = {
  PA_0,   // D0/A0
  PA_1,   // D1/A1
  PA_2,   // D2/A2
  PA_3,   // D3/A3
  PA_4,   // D4/A4
  PA_5,   // D5/A5
  PA_6,   // D6/A6
  PA_7,   // D7/A7
  PA_9,   // D8
  PA_13,  // D9
  PA_14,  // D10
  PA_15,  // D11
  PB_0,   // D12/A8
  PB_1,   // D13/A9
  PB_3,   // D14
  PB_4,   // D15
  PB_5,   // D16
  PB_6,   // D17
  PB_7,   // D18
  PB_8,   // D19/BOOT0
  PD_0,   // D20/NEOPIXEL_PIN
  PD_1,   // D21
};
// note: PA_11 and PA_12 are connected to USB connector
// note: PA_8, PA_10 and PB_2 are not available on CH32V203G6U6

// Analog (Ax) pin number array
const uint32_t analogInputPin[] = {
  0,  // A0, PA0
  1,  // A1, PA1
  2,  // A2, PA2
  3,  // A3, PA3
  4,  // A4, PA4
  5,  // A5, PA5
  6,  // A6, PA6
  7,  // A7, PA7
  12, // A8, PB0
  13, // A9, PB1
};

extern "C" {

void pre_init(void) {
  // Enable Flash enhance read mode for full 224KB
  FLASH->KEYR = 0x45670123; // FLASH_Unlock_Fast();
  FLASH->KEYR = 0xCDEF89AB;

  FLASH->CTLR |= (1 << 24); // Enhanced Read Mode

  FLASH->CTLR |= (1 << 15); // FLASH_Lock_Fast();

  hw_config_init();

  // Enable PD0 and PD1
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_PD01, ENABLE);

  // Enable PA13 and PA14
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
}

}