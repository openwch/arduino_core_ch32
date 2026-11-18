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
  PC_1,   // D2/SDA
  PC_2,   // D3/SCL
  PC_4,   // D4/A2
  PD_1,   // D5/SWIO
  PD_4,   // D6/A7
  PD_5,   // D7/A5
  PD_6    // D8/A6  
};

// Analog (Ax) pin number array
const uint32_t analogInputPin[] = {
  1,    // A0/PA2
  0,    // A1/PA1
  4,    // A2/PC4
  NC,   // A3/PD2 no pin
  NC,   // A4/PD3 no pin
  7,    // A5/PD5
  8,    // A6/PD6
  6     // A7/PD4 
};



