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




// Digital PinName array,Some GPIOs are bound to the same pin. 
const PinName digitalPin[] = {
  PA_1,   // D0/A1   
  PA_2,   // D1/A0
  PC_0,   // D2
  PC_1,   // D3
  PC_2,   // D4
  PC_3,   // D5 
  PC_4,   // D6/A2
  PC_5,   // D7
  PC_6,   // D8
  PC_7,   // D9
  PD_0,   // D10
  PD_1,   // D11
  PD_2,   // D12/A3
  PD_3,   // D13/A4
  PD_4,   // D14/A7
  PD_5,   // D15/A5
  PD_6,   // D16/A6  
  PD_7    // D17
};

// Analog (Ax) pin number array
const uint32_t analogInputPin[] = {
  1,    // A0/PA2
  0,    // A1/PA1
  6,    // A2/PC4
  12,   // A3/PD2
  13,   // A4/PD3
  15,   // A5/PD5
  16,   // A6/PD6
  14    // A7/PD4 
};



