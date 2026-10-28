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
These CH32V00X packages have 24 pins:
  CH32V006E8R6 - PA0-5, PB0/1/3, PC0-2/4/5, PD0-7 - PD1=SWIO, PC5=RST, PD6=RX, PD5=TX, 64K flash, 8K RAM, OPA, CAPKEY, 2xUART, 3xTMR
  CH32V005E6R6 - 32K flash, 6K RAM, 2xTMR, no CAPKEY, no OPA polling, 2xUART

CH32V006E8R6 QSOP24 PINOUT
                   +-----v-----+
       SWIO D18~ 1-+PD1     PB3+-24  D17 MISO
            D19~ 2-+PC0     PB1+-23  D16
        SDA D20  3-+PC1     PB0+-22  D15
        SCL D0   4-+PC2     PA3+-21  D14 
            GND  5-+VSS     PD0+-20 ~D13 
            VCC  6-+VDD     PA2+-19 ~D12 A0 
        RST D21  7-+PC5     PA0+-18  D11  
      SS/A2 D1~  8-+PC4     PA1+-17  D10* A1  
     SCK/A3 D2~  9-+PD2     PA5+-16  D9  RX2
    MOSI/A4 D3~ 10-+PD3     PA4+-15  D8  TX2
         A7 D4  11-+PD4     PD7+-14 ~D7
      TX/A5 D5  12-+PD5     PD6+-13  D6 RX/A6
                   +-----------+         
*/

// Digital PinName array, PA1 is shorted to PA6
const PinName digitalPin[] = {
  PC_2,   // D0/SCL
  PC_4,   // D1/A2/SS
  PD_2,   // D2/A3/SCK
  PD_3,   // D3/A4/MOSI
  PD_4,   // D4/A7
  PD_5,   // D5/A5/TX
  PD_6,   // D6/A6/RX
  PD_7,   // D7
  PA_4,   // D8/TX2
  PA_5,   // D9/RX2
  PA_1,   // D10*/A1
  PA_0,   // D11
  PA_2,   // D12/A0
  PD_0,   // D13/
  PA_3,   // D14/
  PB_0,   // D15/
  PB_1,   // D16/
  PB_3,   // D17/MISO
  PD_1,   // D18/SWIO
  PC_0,   // D19
  PC_1,   // D20/SDA
  PC_5    // D21/RST
/*
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
*/  

};

// Analog (Ax) pin number array
const uint32_t analogInputPin[] = {
  12,    // A0/PA2
  10,    // A1/PA1
  1,    // A2/PC4
  2,   // A3/PD2
  3,   // A4/PD3
  5,   // A5/PD5
  6,   // A6/PD6
  4    // A7/PD4 
};



