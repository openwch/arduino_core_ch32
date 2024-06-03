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
  PA_0,  //D0/A0
  PA_1,  //D1/A1
  PA_2,  //D2/A2
  PA_3,  //D3/A3
  PA_4,  //D4/A4   
  PA_5,  //D5/A5    

  PA_10,  //D6      USART1_RX
  PA_9,   //D7      USART1_TX
  PB_4,   //D8   
  PB_6,   //D9       
  PB_7,   //D10       
  PB_8,   //D11                 
  PB_9,   //D12                 
  PA_8,   //D13                  
  PB_1,   //D14                    
  PB_0,   //D15                     
  PB_12,  //D16    SPI2_NSS
  PB_15,  //D17    SPI2_MOSI  
  PB_14,  //D18    SPI2_MISO
  PB_13,  //D19    SPI2_SCK
  PB_11,  //D20    I2C2_SDA
  PB_10   //D21    I2C2_SCL
};

// Analog (Ax) pin number array
const uint32_t analogInputPin[] = {
  0,  // A0,  PA0
  1,  // A1,  PA1
  2,  // A2,  PA2
  3,  // A3,  PA3
  4,  // A4,  PA4
  5   // A5,  PA5
};



