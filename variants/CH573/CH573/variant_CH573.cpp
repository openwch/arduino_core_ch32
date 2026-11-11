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
  PA_4,   //D0/A0    RXD3/AIN0
  PA_5,   //D1/A1    TXD3/AIN1
  PA_8,   //D2/A12   RXD1/AIN12
  PA_9,   //D3/A13   TMR0/TXD1/AIN13
  PA_10,  //D4       X32KI/TMR1
  PA_11,  //D5       X32KO/TMR2
  PA_12,  //D6/A2    SCS/PWM4/AIN2
  PA_13,  //D7/A3    SCK0/PWM5/AIN3
  PA_14,  //D8/A4    MOSI/TXD0_/AIN4
  PA_15,  //D9/A5    MISO/RXD0_/AIN5
  PB_0,   //D10/A8   PWM6/CTS/AIN8
  PB_4,   //D11      RXD0/PWM7
  PB_6,   //D12/A9   RTS/PWM8/AIN9
  PB_7,   //D13      TXD0/PWM9 
  PB_10,  //D14      UD-/TMR1_
  PB_11,  //D15      UD+/TMR2_
  PB_12,  //D16      SCS_/RXD1_ (5VT)
  PB_13,  //D17      SCK0_/TXD1_ (5VT)
  PB_14,  //D18      TIO/MOSI_/PWM10/DSR (5VT)
  PB_15,  //D19      TCK/MISO_/DTR (5VT)
  PB_22,  //D20      TMR3/RXD2
  PB_23   //D21      RST#/TMR0_/TXD2/PWM11
};

// Analog (Ax) pin number array
const uint32_t analogInputPin[] = {
  0,  // A0,  PA0
  1,  // A1,  PA1
  6,  // A2,  PA12
  7,  // A3,  PA13
  8,  // A4,  PA14
  9,  // A5,  PA15
  0xFFFFFFFF, // A6 does not exist
  0xFFFFFFFF, // A7 does not exist 
  10, // A8,  PB0
  12, // A9,  PB6
  0xFFFFFFFF, // A10 does not exist
  0xFFFFFFFF, // A11 does not exist
  2,  // A12, PA8
  3,  // A13, PA9
};

