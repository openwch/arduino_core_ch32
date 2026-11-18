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

#include "Arduino.h"
#include "PeripheralPins.h"

/* =====
 * Notes:
 * - The pins mentioned Px_y_ALTz are alternative possibilities which use other
 *   HW peripheral instances. You can use them the same way as any other "normal"
 *   pin (i.e. analogWrite(PA7_ALT1, 128);).
 *
 * - Commented lines are alternative possibilities which are not used per default.
 *   If you change them, you will have to know what you do
 * =====
 */

//*** ADC ***
#ifdef ADC_MODULE_ENABLED
WEAK const PinMap PinMap_ADC[] = {
  {PA_2,      ADC1, CH_PIN_DATA_EXT(CH_MODE_INPUT, CH_CNF_INPUT_ANALOG, 0, AFIO_NONE, 0)}, // ADC1_IN0
  {PA_1,      ADC1, CH_PIN_DATA_EXT(CH_MODE_INPUT, CH_CNF_INPUT_ANALOG, 0, AFIO_NONE, 1)}, // ADC1_IN1
  {PC_4,      ADC1, CH_PIN_DATA_EXT(CH_MODE_INPUT, CH_CNF_INPUT_ANALOG, 0, AFIO_NONE, 2)}, // ADC1_IN2
  {PD_5,      ADC1, CH_PIN_DATA_EXT(CH_MODE_INPUT, CH_CNF_INPUT_ANALOG, 0, AFIO_NONE, 5)}, // ADC1_IN5
  {PD_6,      ADC1, CH_PIN_DATA_EXT(CH_MODE_INPUT, CH_CNF_INPUT_ANALOG, 0, AFIO_NONE, 6)}, // ADC1_IN6
  {PD_4,      ADC1, CH_PIN_DATA_EXT(CH_MODE_INPUT, CH_CNF_INPUT_ANALOG, 0, AFIO_NONE, 7)}, // ADC1_IN7
  {NC,        NP,   0}
};
#endif

//*** No DAC ***



//*** I2C ***
#ifdef I2C_MODULE_ENABLED
WEAK const PinMap PinMap_I2C_SDA[] = {
  {PC_1, I2C1, CH_PIN_DATA(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFOD, NOPULL, AFIO_NONE)},
  {NC,   NP,   0}
};
#endif

#ifdef I2C_MODULE_ENABLED
WEAK const PinMap PinMap_I2C_SCL[] = {
  {PC_2, I2C1, CH_PIN_DATA(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFOD, NOPULL, AFIO_NONE)},
  {NC,   NP,   0}
};
#endif

//*** TIM ***
#ifdef TIM_MODULE_ENABLED

 const PinMap PinMap_TIM[] WEAK = {
// Pins get remapped in groups as below

// NOTE: PD_1, PD_4, and PD_5 are on the same pin as SWIO

// WARNING cannot use CHx and CHxN simultaneously (they are the same channel), such as with PA_1 and PA_2

// WARNING using one timer in a group means only timers in that group can be used because all pins in that group are
// remapped together. So for ex. you can't use PA_1 and PD_4_ALT3 together.

// Hint - you can save space by commenting out groups and pins you don't use
// Hint - you can save space by defining your own PinMap_TIM[] in your code which will override this WEAK definition

// Timer1 Groups
  {PA_1,       TIM1, CH_PIN_DATA_EXT(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, NOPULL, AFIO_Remap_TIM1_DISABLE, 2)}, // TIM1_CH2
  {PA_2,       TIM1, CH_PIN_DATA_EXT(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, NOPULL, AFIO_Remap_TIM1_DISABLE, 2) | CH_PIN_INV_BIT}, // TIM1_CH2N
  {PD_1,       TIM1, CH_PIN_DATA_EXT(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, NOPULL, AFIO_Remap_TIM1_DISABLE, 3) | CH_PIN_INV_BIT}, // TIM1_CH3N
  {PC_4,       TIM1, CH_PIN_DATA_EXT(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, NOPULL, AFIO_Remap_TIM1_DISABLE, 4)}, // TIM1_CH4


  {PC_4_ALT1,  TIM1, CH_PIN_DATA_EXT(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, NOPULL, AFIO_PartialRemap1_TIM1_ENABLE, 2) | CH_PIN_INV_BIT}, // TIM1_CH2N
  {PD_1_ALT1,  TIM1, CH_PIN_DATA_EXT(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, NOPULL, AFIO_PartialRemap1_TIM1_ENABLE, 3) | CH_PIN_INV_BIT}, // TIM1_CH3N

// Remap2 is the same as Default except T1ETR moves from default PC5 to remap PD4. So this is commented out to save space.
//   {PA_1_ALT2,  TIM1, CH_PIN_DATA_EXT(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, NOPULL, AFIO_PartialRemap2_TIM1_ENABLE, 2)}, // TIM1_CH2
//   {PA_2_ALT2,  TIM1, CH_PIN_DATA_EXT(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, NOPULL, AFIO_PartialRemap2_TIM1_ENABLE, 2) | CH_PIN_INV_BIT}, // TIM1_CH2N
//   {PD_1_ALT2,  TIM1, CH_PIN_DATA_EXT(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, NOPULL, AFIO_PartialRemap2_TIM1_ENABLE, 3) | CH_PIN_INV_BIT}, // TIM1_CH3N
//   {PC_4_ALT2,  TIM1, CH_PIN_DATA_EXT(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, NOPULL, AFIO_PartialRemap2_TIM1_ENABLE, 4)}, // TIM1_CH4

  {PC_4_ALT3,  TIM1, CH_PIN_DATA_EXT(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, NOPULL, AFIO_FullRemap_TIM1_ENABLE, 1)}, // TIM1_CH1
  {PD_4_ALT3,  TIM1, CH_PIN_DATA_EXT(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, NOPULL, AFIO_FullRemap_TIM1_ENABLE, 4)}, // TIM1_CH4

// Timer2 Groups
  {PD_4,       TIM2, CH_PIN_DATA_EXT(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, NOPULL, AFIO_Remap_TIM2_DISABLE, 1)}, // TIM2_CH1

  {PC_2,  	   TIM2, CH_PIN_DATA_EXT(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, NOPULL, AFIO_PartialRemap1_TIM2_ENABLE, 2)}, // TIM2_CH2
  {PC_1,  	   TIM2, CH_PIN_DATA_EXT(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, NOPULL, AFIO_PartialRemap1_TIM2_ENABLE, 4)}, // TIM2_CH4

  {PC_1_ALT2,  TIM2, CH_PIN_DATA_EXT(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, NOPULL, AFIO_PartialRemap2_TIM2_ENABLE, 1)}, // TIM2_CH1

  {PC_1_ALT3,  TIM2, CH_PIN_DATA_EXT(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, NOPULL, AFIO_FullRemap_TIM2_ENABLE, 1)}, // TIM2_CH1
  {PD_6,  	   TIM2, CH_PIN_DATA_EXT(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, NOPULL, AFIO_FullRemap_TIM2_ENABLE, 3)}, // TIM2_CH3
  {PD_5,  	   TIM2, CH_PIN_DATA_EXT(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, NOPULL, AFIO_FullRemap_TIM2_ENABLE, 4)}, // TIM2_CH4

  {NC,         NP,   0}
};
#endif



//*** UART ***
#ifdef UART_MODULE_ENABLED
WEAK const PinMap PinMap_UART_TX[] = {
  {PD_5, USART1, CH_PIN_DATA(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, 0, AFIO_Remap_USART1_DISABLE)}, 
  {PD_6, USART1, CH_PIN_DATA(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, 0, AFIO_PartialRemap2_USART1_ENABLE)}, 
  {NC,   NP,     0}
};
#endif

#ifdef UART_MODULE_ENABLED
WEAK const PinMap PinMap_UART_RX[] = {
  {PD_6, USART1, CH_PIN_DATA(CH_MODE_INPUT, CH_CNF_INPUT_PUPD, PULLUP, AFIO_Remap_USART1_DISABLE)},
  {PD_5, USART1, CH_PIN_DATA(CH_MODE_INPUT, CH_CNF_INPUT_PUPD, PULLUP, AFIO_PartialRemap2_USART1_ENABLE)},
  {NC,    NP,     0}
};
#endif

#ifdef UART_MODULE_ENABLED
WEAK const PinMap PinMap_UART_RTS[] = {
  {PC_2, USART1, CH_PIN_DATA(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, 0, AFIO_Remap_USART1_DISABLE)}, 
  {NC, USART1, CH_PIN_DATA(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, 0, AFIO_PartialRemap2_USART1_ENABLE)}, 
  {NC,    NP,     0}
};
#endif

#ifdef UART_MODULE_ENABLED
WEAK const PinMap PinMap_UART_CTS[] = {
  {NC, USART1, CH_PIN_DATA(CH_MODE_INPUT, CH_CNF_INPUT_PUPD, PULLUP, AFIO_Remap_USART1_DISABLE)}, 
  {NC, USART1, CH_PIN_DATA(CH_MODE_INPUT, CH_CNF_INPUT_PUPD, PULLUP, AFIO_PartialRemap2_USART1_ENABLE)}, 
  {NC,    NP,     0}
};
#endif
