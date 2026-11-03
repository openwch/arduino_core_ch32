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
  PA_0,   //D0/A9    SCK1/LED0/A9
  PA_1,   //D1/A8    MOSI1/LED1/A8
  PA_2,   //D2/A7    TMR3_/MISO1/RI/LED2/A7
  PA_3,   //D3/A6    LED3/A6
  PA_4,   //D4/A0    RXD3/LEDC/A0
  PA_5,   //D5/A1    TXD3/LED4/A1
  PA_6,   //D6/A10   RXD2/PWM4_/LED5/A10
  PA_7,   //D7/A11   TXD2/PWM5_/LED6/A11
  PA_8,   //D8/A12   RXD1/LED7/A12
  PA_9,   //D9/A13   TMR0/TXD1/A13
  PA_10,  //D10      X32KI/TMR1
  PA_11,  //D11      X32KO/TMR2
  PA_12,  //D12/A2   SCS/PWM4/A2
  PA_13,  //D13/A3   SCK0/PWM5/A3
  PA_14,  //D14/A4   MOSI/TXD0_/A4
  PA_15,  //D15/A5   MISO/RXD0_/A5
  PB_0,   //D16      CTS/PWM6
  PB_1,   //D17      DSR/PWM7_
  PB_2,   //D18      PWM8_
  PB_3,   //D19      DCD/PWM9_
  PB_4,   //D20      RXD0/PWM7
  PB_5,   //D21      DTR
  PB_6,   //D22      RTS/PWM8
  PB_7,   //D23      TXD0/PWM9
  PB_8,   //D24      NFCM
  PB_9,   //D25      NFCI
  PB_10,  //D26      UD-/TMR1_
  PB_11,  //D27      UD+/TMR2_
  PB_12,  //D28      U2D-/SCS_/SDA/RXD1_
  PB_13,  //D29      U2D+/SCK0_/SCL/TXD1_
  PB_14,  //D30      TIO/MOSI_/PWM10/DSR_
  PB_15,  //D31      TCK/MISO_/DTR_
  PB_16,  //D32      NFC-
  PB_17,  //D33      NFC+
  PB_18,  //D34      
  PB_19,  //D35
  PB_20,  //D36      SDA_/RXD3_
  PB_21,  //D37      SCL_/TXD3_
  PB_22,  //D38      TMR3/RXD2_
  PB_23   //D39      TMR0_/TXD2_/PWM11/RST
};

// Analog (Ax) pin number array
const uint32_t analogInputPin[] = {
  4,  // A0,  PA4
  5,  // A1,  PA5
  12, // A2,  PA12
  13, // A3,  PA13
  14, // A4,  PA14
  15, // A5,  PA15
  3,  // A6,  PA3
  2,  // A7,  PA2
  1,  // A8,  PA1
  0,  // A9,  PA0
  6,  // A10, PA6
  7,  // A11, PA7
  8,  // A12, PA8
  9   // A13, PA9
};