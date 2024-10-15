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
  PA_0,   // D0  A0
  PA_1,   // D1  A1
  PA_2,   // D2  TX2/A2
  PA_3,   // D3  RX2/A3
  PA_4,   // D4  CS/A4
  PA_5,   // D5  SCK/A5
  PA_6,   // D6  MISO/A6
  PB_0,   // D7  MOSI/A8  PA7 is tied to PB0 => input only 
  PB_1,   // D8  A9
  PB_7,   // D9  RST
  PC_16,  // D10 USBDM  tied to PC11=input only 
  PC_17,  // D11 USBPD  tied to PC10=input only 
  PA_9,   // D12
  PA_11,  // D13 SDA (not in 0-series)
  PA_10,  // D14 SCL (not in 0-series)
  PC_3,   // D15 A13
  PC_18,  // D16 SWDIO
  PC_19,  // D17 SWCLK
};

// Analog (Ax) pin number array, refers to PinName in digitalPin[]?
const uint32_t analogInputPin[] = {
  0,  // A0,  PA0 
  1,  // A1,  PA1
  2,  // A2,  PA2
  3,  // A3,  PA3 (no ADC Ch3 in 0-series!)
  4,  // A4,  PA4
  5,  // A5,  PA5
  6,  // A6,  PA6
  7,  // A7,  PA7=PB0, input only
  7,  // A8,  PB0=PA7, input only
  8,  // A9,  PB1
  10, // A10, none
  11, // A11, none
  12, // A12, none
  15, // A13, PC3
};



