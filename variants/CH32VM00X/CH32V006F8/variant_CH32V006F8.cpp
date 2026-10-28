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


/*
These CH32V006F packages have 20 pins:
  CH32V006F8P6 - TSSOP20 - PC0-PC7, PD0-PD7, PA1/PA2, A0-A7      - PD1=SWIO, PD7=RST, PD6=RX, PD5=TX
  CH32V006F8U6 - QFN20   - PC0-PC7, PD0-PD7, PA1/PA2, A0-A7      - PD1=SWIO, PD7=RST, PD6=RX, PD5=TX
  CH32V006F4U6 - QFN20   - PC0-PC7, PD0-PD7, PA1/PA2, A0-A7      - PD1=SWIO, PD7=RST, PD6=RX, PD5=TX, 16K Flash, 4K RAM, no OPA

V003/V002/V006 TSSOP20 PINOUT
            +-----v-----+
   14/A7 1 -+PD4     PD3+- 20  13/A4
   TX/A5 2 -+PD5     PD2+- 19  12/A3
   RX/A6 3 -+PD6     PD1+- 18  SWIO
     RST 4 -+PD7     PC7+- 17  9
    0/A1 5 -+PA1     PC6+- 16  8
    1/A0 6 -+PA2     PC5+- 15  7
     GND 7 -+VSS     PC4+- 14  6/A2
      10 8 -+PD0     PC3+- 13  5
     VCC 9 -+VDD     PC2+- 12  4/SCL
      2 10 -+PC0     PC1+- 11  3/SDA
            +-----------+        
*/

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
  PD_1,   // D11/SWIO
  PD_2,   // D12/A3
  PD_3,   // D13/A4
  PD_4,   // D14/A7
  PD_5,   // D15/A5/TX
  PD_6,   // D16/A6/RX
  PD_7    // D17/RST
/*    
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
*/
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
/*
  2,     // A0/PA2
  1,     // A1/PA1
  19,    // A2/PC4
  25,    // A3/PD2
  26,    // A4/PD3
  28,    // A5/PD5
  29,    // A6/PD6
  27     // A7/PD4 
*/
};



