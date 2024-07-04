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
  PB_1,  // D0/A0
  PB_0,  // D1/A1
  PA_1,  // D2/A2
  PA_0,  // D3/A3
  PA_2,  // D4/TX
  PA_3,  // D5/RX
  PA_4,  // D6/Neopixel
  PA_5,  // D7/SCK
  PA_6,  // D8/MISO
  PA_7,  // D9/MOSI
  PB_6,  // D10/SCL
  PB_7,  // D11/SDA
  PB_8,  // D12/BOOT0 Button
};

// Analog (Ax) pin number array
const uint32_t analogInputPin[] = {
  0,  // A0,  PB1
  1,  // A1,  PB0
  2,  // A2,  PA1
  3,  // A3,  PA0
};



