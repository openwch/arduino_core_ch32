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
  CH32V002J4M6 - SOP8    - PC1/2/4, PD1, PA1/2, A0-2/5/6         - PD1=SWIO=PD4=PD5=TX, PA1=PD6=A1=A6=RX
The largest package has 20 pins. For compatibility of Arduino pin numbers the TSSOP20 pinout is used.

CH32V003/V002A4M6 SOP16 PINOUT
              +-----v-----+
RX'/SDA D3  1-+PC1     PC0+-16 D2~/TX'
    SCL D4  2-+PC2     VDD+-15 VCC
        D5~ 3-+PC3     VSS+-14 GND
        D6~ 4-+PC4     PA2+-13 D1/A0
        D8  5-+PC6     PA1+-12 D0~/A1
     SS/D9  6-+PC7     PD7+-11 RST
      SWIO  7-+PD1     PD6+-10 RX/A6/MOSI
SCK/A7/D14~ 8-+PD4     PD5+-9  TX/A5/MISO
              +-----------+
TX'/RX' - when I2C is enabled TX/RX=PD5/PD6, when not enabled TX/RX=PC0/PC1

CH32V002 vs CH32V003

V002: RV32EmC, 16K Flash, 4KB SRAM, 3328B bootloader, 256B user mem, 12-bit ADC, System Clock Monitor, 2-5V, QFN12
V003: RV32EC,  16K Flash, 2KB SRAM, 1920B bootloader, 64B user mem, 10-bit ADC, OPA, 3v#-5V
V004: RV32EmC, 32K Flash, 6KB SRAM, 3328B bootloader, 256B user mem, 12-bit ADC, System Clock Monitor, 2-5V
V005: RV32EmC, 32K Flash, 6KB SRAM, 3328B bootloader, 256B user mem, 12-bit ADC, System Clock Monitor, 2-5V, QFN12, QSOP24, UART2
V006: RV32EmC, 62K Flash, 8KB SRAM, 3328B bootloader, 256B user mem, 12-bit ADC, 8-ch touch. System Clock Monitor, 2-5V, OPA, UART2, 16-bit simple timer, QFN32
*/

// Digital PinName array. On smaller packages some GPIOs are bound to the same pin.
// Leading pinout is TSSOP20 (18x gpio), same as V003 TSSOP20
// For SOP16 (A4M6) we keep the same pin numbering as on TSSOP20
const PinName digitalPin[] = {
  PA_1,   // D0/A1
  PA_2,   // D1/A0
  PC_0,   // D2
  PC_1,   // D3
  PC_2,   // D4
  PC_3,   // D5
  PC_4,   // D6/A2
  NC,     // D7        No PC_5 on A4M6
  PC_6,   // D8
  PC_7,   // D9 (recommended software CS/SS)
  NC,     // D10       No PD_0 on A4M6
  PD_1,   // D11/SWIO
  NC,     // D12/A3    No PD_2 on A4M6
  NC,     // D13/A4    No PD_3 on A4M6
  PD_4,   // D14/A7/SCK
  PD_5,   // D15/A5/TX/MISO
  PD_6,   // D16/A6/RX/MOSI
  PD_7    // D17/RST
};

// Analog (Ax) pin number array
const uint32_t analogInputPin[] = {
  1,    // A0/PA2
  0,    // A1/PA1
  6,    // A2/PC4
  12,   // A3/PD2    No PD2 on A4M6
  13,   // A4/PD3    No PD3 on A4M6
  15,   // A5/PD5
  16,   // A6/PD6
  14    // A7/PD4 
};
