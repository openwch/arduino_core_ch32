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
 * Seeed Studio XIAO CH32X035 (CH32X035G8U6, QFN28)
 *******************************************************************************
 */

#include "pins_arduino.h"

// Digital PinName array
// D13-D17 are test pads; D15 (PB1) is bonded to PB5 on the QFN28 package.
// PC14/PC15 (USB CC) and PC16/PC17 (USB DM/DP) are not broken out on the
// XIAO edge castellation and are left out of the digital numbering.
const PinName digitalPin[] = {
  PA_0,   // D0,  A0
  PA_1,   // D1,  A1
  PA_2,   // D2,  A2
  PA_3,   // D3,  A3
  PC_19,  // D4,  SDA
  PC_18,  // D5,  SCL
  PB_10,  // D6,  TX
  PB_11,  // D7,  RX
  PA_5,   // D8,  SCK
  PA_6,   // D9,  MISO
  PA_7,   // D10, MOSI
  PC_3,   // D11, A4
  PB_0,   // D12, A5
  PB_3,   // D13, SS (TP3)
  PB_4,   // D14 (TP4)
  PB_1,   // D15 (TP9, bonded to PB5)
  PB_6,   // D16 (TP10)
  PB_7,   // D17 (TP11)
  PA_4,   // D18, User LED (active LOW)
  PB_8,   // D19, RGB DIN
  PB_9,   // D20, BAT_EN (active HIGH)
  PB_12,  // D21, user button K1 (pressed LOW)
  PC_0    // D22, A6, battery sense (1/2 divider)
};

// Analog (Ax) pin number array
const uint32_t analogInputPin[] = {
  0,   // A0, PA0
  1,   // A1, PA1
  2,   // A2, PA2
  3,   // A3, PA3
  11,  // A4, PC3
  12,  // A5, PB0
  22,  // A6, PC0
};
