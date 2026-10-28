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

CH32V002 comes in same packages as CH32V003, with addition of QFN12:
  CH32V002F4P6 - TSSOP20 - PC0-PC7, PD0-PD7, PA1/PA2, A0-A7      - PD1=SWIO, PD7=RST, PD6=RX, PD5=TX
  CH32V002F4U6 - QFN20   - PC0-PC7, PD0-PD7, PA1/PA2, A0-A7      - PD1=SWIO, PD7=RST, PD6=RX, PD5=TX
  CH32V002A4M6 - SOP16   - PC0-PC7, PD1/4-7, PA1/2, A0-2/5-7     - PD1=SWIO, PD7=RST, PD6=RX, PD5=TX
  CH32V002D4U6 - QFN12   - PC0/3/4/6/7, PD0/1/4/7, PA1/2, A0-2/7 - PD1=SWIO, PD7=RST
  CH32V002J4M6 - SOP8    - PC1/2/4, PD1, PA1/2, A0-2/5/6         - PD1=SWIO=PD4=PD5=TX, PA1=PD6=A1=A6=RX
The CH32V002D4U6 (QFN12 package) has 11 I/O pins. Unfortunately I2C/UART uses the SWIO pin (PD1). This means after programming SWIO may become disabled.
There is no full set of SPI pins on the D4U6. 

V002 QFN12->DIP14 DEV-BOARD PINOUT
               +-----v------+
     D0~/A1 1 -+PA1      PD7+- 14  D10~/RST
      D1/A0 2 -+PA2      PD4+- 13  D9/A7
  TX/SDA/D7 3 -+PD0      PD1+- 12  SWIO/D8/SCL/RX
        3V3 4 -+VDD  /\  PC7+- 11  D6
         D2 5 -+PC0  \/  PC6+- 10  D5
        D3~ 6 -+PC3      PC4+- 9   D4~/A2
        3V3 7 -+VDD+     VSS+- 8   GND
               +------------+        

*/
// Digital PinName array. On QFN12 the SWIO pin is used for UART/I2C.
// Leading pinout is TSSOP20 (18x gpio), same as V003 TSSOP20
const PinName digitalPin[] = {
  PA_1,   // D0/A1
  PA_2,   // D1/A0
  PC_0,   // D2
  PC_3,   // D3
  PC_4,   // D4/A2
  PC_6,   // D5
  PC_7,   // D6
  PD_0,   // D7/SDA/TX
  PD_1,   // D8/SWIO/SCL/TX
  PD_4,   // D9/A3
  PD_7    // D10/RST
};

// Analog (Ax) pin number array
const uint32_t analogInputPin[] = {
  1,    // A0/PA2
  0,    // A1/PA1
  4,    // A2/PC4
  9    // A3/PD4 
};



