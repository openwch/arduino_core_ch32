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
  CH32V007E8R6 - like V006E8: PA0-5, PB0/1/3, PC0-2/4/5, PD0-7 - PD1=SWIO, PC5=RST, PD6=RX, PD5=TX, 64K flash, 8K RAM, OPA, CAPKEY, 2xUART, 3xTMR
  CH32M007E8R6 - for motors 6-24V P+N - PA0-5, PB0/1/3, PC1/2/4/5, PD0-7, 3xLO/HO, different pinout from V007E8 

CH32M007E8R6 QSOP24 PINOUT - 6-24V
                   +-----v-----+
       SWIO D18  1-+PD1     PA1+-24 ~D17 A1
     SCK/A3 D19~ 2-+PD2     PC5+-23  D16 RST
    MOSI/A4 D20~ 3-+PD3     PC4+-22 ~D15 SS/A2
          A7 D0  4-+PD4     PC2+-21  D14 SCL 
       A5 TX/D1  5-+PD5     PC1+-20  D13 SDA
       A6 RX/D2  6-+PD6     PB3+-19  D12 MISO
             D3~ 7-+PD7     PD0+-18  D11 LO3  
         TX2/D4  8-+PA4     PB1+-17  D10 HO3
         RX2/D5  9-+PA5     PA2+-16  D9  LO2
            VDD 10-+VDD     PB0+-15  D8  HO2
            VSS 11-+VSS     PA0+-14  D7  LO1
          6-24V 12-+VHV     PA3+-13  D6  HO1
                   +-----------+
SPI
     def  _1 *_2  _3  _4  _5  _6
MISO   x   x PB3   x PD5   x   x
MOSI   x   x PD3   x PD6   x   x
SCK  PC5 PC5 PD2   x PD4 PA1   x
SS   PC1   x PC4   x PD3 PC1 PC4

UART
    def *_1  _2  _3  _4  _5  _6
TX2 PA7 PA4 PA2 PD2 PB0 PC4 PA6
RX2 PB3 PA5 PA3 PD3 PB1 PD1 PA5

*/

// Digital PinName array, PA1 is shorted to PA6
const PinName digitalPin[] = {
  PD_4,   // D0/A7
  PD_5,   // D1/A5/TX
  PD_6,   // D2/A6/RX
  PD_7,   // D3/
  PA_4,   // D4/TX2
  PA_5,   // D5/RX2
  PA_3,   // D6/HO1
  PA_0,   // D7/LO1
  PB_0,   // D8/HO2
  PA_2,   // D9/LO2
  PB_1,   // D10/HO3
  PD_0,   // D11/LO3
  PB_3,   // D12/MISO
  PC_1,   // D13/SDA
  PC_2,   // D14/SCL
  PC_4,   // D15/SS/A2
  PC_5,   // D16/RST
  PA_1,   // D17/A1
  PD_1,   // D18/SWIO
  PD_2,   // D19/A3/SCK
  PD_3    // D20/A4/MOSI
};

// Analog (Ax) pin number array
const uint32_t analogInputPin[] = {
  9,   // A0/PA2 (A0 cannot be used, PA2 occupied by LO2)
  17,  // A1/PA1
  15,  // A2/PC4
  19,  // A3/PD2
  20,  // A4/PD3
  1,   // A5/PD5
  2,   // A6/PD6
  0    // A7/PD4 
};

/* V007 ADC

Ai0 PA2   x
Ai1 PA1 D17
Ai2 PC4 D15
Ai3 PD2 D19
Ai4 PD3 D20
Ai5 PD5 D1
Ai6 PD6 D2
Ai7 PD4 D0

*/


