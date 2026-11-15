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
  CH32V002A4M6 - SOP16   - PC0-4/6/7, PD1/4-7, PA1/2, A0-2       - PD1=SWIO, PD7=RST, PD6=RX, PD5=TX
  CH32V002D4U6 - QFN12   - PC0/3/4/6/7, PD0/1/4/7, PA1/2, A0-2/7 - PD1=SWIO, PD7=RST
  CH32V002J4M6 - SOP8    - PC1/2/4, PD1, PA1/2, A0-2/5/6         - PD1=SWIO=PD4=PD5=TX/RX', PA1=PD6=A1=A6=RX/TX'
The largest package TSSOP20 has 20 pins. The smallest is SOP8 with 8 pins.

CH32V002J4M6/CH32V003J4M6 SOP8 PINOUT
                  +-----v-----+
  TX'/A6/RX/D8~ 1-+PD6     PD1+-8 ~D5/SWIO
         A1/D0~   +PA1   PD5/4+   ~D7/A5/TX ~D6/A7/RX'
           GND  2-+VSS     PC4+-7 ~D4/A2
                  +           +
         A0/D1~ 3-+PA2     PC2+-6 ~D3/SCL
                  +           +
           VCC  4-+VDD     PC1+-5 ~D2/SDA
                  +-----------+

CH32V002 vs CH32V003

V002: RV32EmC, 16K Flash, 4KB SRAM, 3328B bootloader, 256B user mem, 12-bit ADC, System Clock Monitor, 2-5V, QFN12
V003: RV32EC,  16K Flash, 2KB SRAM, 1920B bootloader, 64B user mem, 10-bit ADC, OPA, 3v#-5V
V004: RV32EmC, 32K Flash, 6KB SRAM, 3328B bootloader, 256B user mem, 12-bit ADC, System Clock Monitor, 2-5V
V005: RV32EmC, 32K Flash, 6KB SRAM, 3328B bootloader, 256B user mem, 12-bit ADC, System Clock Monitor, 2-5V, QFN12, QSOP24, UART2
V006: RV32EmC, 62K Flash, 8KB SRAM, 3328B bootloader, 256B user mem, 12-bit ADC, 8-ch touch. System Clock Monitor, 2-5V, OPA, UART2, 16-bit simple timer, QFN32
*/

// Digital PinName array. On SOP8 some GPIOs are bound to the same pin: PD1=PD5=PD4, PA1=PD6
// For CH32V002J4M6 (SOP8) the pinout is the same as CH32V003J4M6 (SOP8), using the same pin numbering.
// CH32V002J4M6 pins:  1:PA1=PD6/TX', 3:PA2, 5:PC1/SDA, 6:PC2/SCL, 7: PC4, 8: PD1=PD4=PD5=SWIO/RX'
const PinName digitalPin[] = {
  PA_1,   // D0/A1
  PA_2,   // D1/A0
  PC_1,   // D2/SDA
  PC_2,   // D3/SCL
  PC_4,   // D4/A2
  PD_1,   // D5/SWIO
  PD_4,   // D6/A7
  PD_5,   // D7/A5/RX'
  PD_6    // D8/A6/TX'
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
