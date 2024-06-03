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
  PA_0,   //D0 
  PA_1,   //D1/A1
  PA_2,   //D2/A0
  PA_3,   //D3
  PA_4,   //D4
  PA_5,   //D5
  PA_6,   //D6
  PA_7,   //D7
  PB_0,   //D8
  PB_1,   //D9 
  PB_2,   //D10
  PB_3,   //D11
  PB_4,   //D12
  PB_5,   //D13
  PB_6,   //D14
  PC_0,   //D15 
  PC_1,   //D16 
  PC_2,   //D17
  PC_3,   //D18
  PC_4,   //D19/A2
  PC_5,   //D20
  PC_6,   //D21 
  PC_7,   //D22
  PD_0,   //D23
  PD_1,   //D24 
  PD_2,   //D25/A3
  PD_3,   //D26/A4
  PD_4,   //D27/A7
  PD_5,   //D28/A5
  PD_6,   //D29/A6 
  PD_7    //D30
};

// Analog (Ax) pin number array
const uint32_t analogInputPin[] = {
  2,     // A0/PA2
  1,     // A1/PA1
  19,    // A2/PC4
  25,    // A3/PD2
  26,    // A4/PD3
  28,    // A5/PD5
  29,    // A6/PD6
  27     // A7/PD4 
};



