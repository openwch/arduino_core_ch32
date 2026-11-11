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
  PA_0,   //D0       UD-/SWDIO/TXD_3/SCL_1/RXD_2
  PA_1,   //D1       UD+/SWCLK/TXD_2/SDA_1/RXD_3
  PA_2,   //D2       CMP_N/TXD_1/SDA_2/PWM2/TMR_1/RXD/SCS_/KEYSCAN0/CAP_IN1_1/CAP_IN2
  PA_3,   //D3       /CMP_P0/TXD/SCL_2/SCK_/PWM3/RXD_1/KEYSCAN1
  PA_4,   //D4       X25MO/PWM4/TMR_2/SCS/CAP_IN1_2/CAP_IN2_3
  PA_5,   //D5       SCL_3/SCK
  PA_6,   //D6       SDA_3/MISO/RXD_4
  PA_7,   //D7       CMP_P1/TXD_4/MOSI/PWM1/TMR/RST_/CAP_IN1/CAP_IN2_1
  PA_8,   //D8       TXD_5/SCL/PWM5/RST/KEYSCAN2
  PA_9,   //D9       SDA/TMR_3/RXD_5/CAP_IN1_3/CAP_IN2_2
  PA_10,  //D10      TXD_7/RXD_6/KEYSCAN3
  PA_11,  //D11      TXD_6/RXD_7/KEYSCAN4
};
