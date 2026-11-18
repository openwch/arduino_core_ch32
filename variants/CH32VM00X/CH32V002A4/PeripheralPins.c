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
  // {PD_2,      ADC1, CH_PIN_DATA_EXT(CH_MODE_INPUT, CH_CNF_INPUT_ANALOG, 0, AFIO_NONE, 3)}, // ADC1_IN3 - no PD2/A3 on A4M6
  // {PD_3,      ADC1, CH_PIN_DATA_EXT(CH_MODE_INPUT, CH_CNF_INPUT_ANALOG, 0, AFIO_NONE, 4)}, // ADC1_IN4 - no PD3/A4 on A4M6
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
WEAK const PinMap PinMap_TIM[] = {
  {PD_4,       TIM2, CH_PIN_DATA_EXT(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, NOPULL, AFIO_Remap_TIM2_DISABLE, 1)}, // TIM2_CH1
  {PD_3,       TIM2, CH_PIN_DATA_EXT(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, NOPULL, AFIO_Remap_TIM2_DISABLE, 2)}, // TIM2_CH2
  {PC_0,       TIM2, CH_PIN_DATA_EXT(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, NOPULL, AFIO_Remap_TIM2_DISABLE, 3)}, // TIM2_CH3
  {PD_7,       TIM2, CH_PIN_DATA_EXT(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, NOPULL, AFIO_Remap_TIM2_DISABLE, 4)}, // TIM2_CH4

  {PD_2,       TIM1, CH_PIN_DATA_EXT(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, NOPULL, AFIO_Remap_TIM1_DISABLE, 1)}, // TIM1_CH1
  {PA_1,       TIM1, CH_PIN_DATA_EXT(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, NOPULL, AFIO_Remap_TIM1_DISABLE, 2)}, // TIM1_CH2
  {PC_3,       TIM1, CH_PIN_DATA_EXT(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, NOPULL, AFIO_Remap_TIM1_DISABLE, 3)}, // TIM1_CH3
  {PC_4,       TIM1, CH_PIN_DATA_EXT(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, NOPULL, AFIO_Remap_TIM1_DISABLE, 4)}, // TIM1_CH4
  {PD_0,       TIM1, CH_PIN_DATA_EXT(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, NOPULL, AFIO_Remap_TIM1_DISABLE, 1)}, // TIM1_CH1N
  {PA_2,       TIM1, CH_PIN_DATA_EXT(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, NOPULL, AFIO_Remap_TIM1_DISABLE, 2)}, // TIM1_CH2N
  {PD_1,       TIM1, CH_PIN_DATA_EXT(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, NOPULL, AFIO_Remap_TIM1_DISABLE, 3)}, // TIM1_CH3N
  {NC,         NP,   0}
};
#endif

//*** UART ***
#ifdef UART_MODULE_ENABLED
WEAK const PinMap PinMap_UART_TX[] = {
  {PD_5, USART1, CH_PIN_DATA(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, 0, AFIO_Remap_USART1_DISABLE)},
  {PD_6, USART1, CH_PIN_DATA(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, 0, AFIO_PartialRemap1_USART1_ENABLE)}, 
  {PC_0, USART1, CH_PIN_DATA(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, 0, AFIO_PartialRemap3_USART1_ENABLE)}, 
  {NC,   NP,     0}
};
#endif

#ifdef UART_MODULE_ENABLED
WEAK const PinMap PinMap_UART_RX[] = {
  {PD_6, USART1, CH_PIN_DATA(CH_MODE_INPUT, CH_CNF_INPUT_PUPD, PULLUP, AFIO_Remap_USART1_DISABLE)},
  {PD_5, USART1, CH_PIN_DATA(CH_MODE_INPUT, CH_CNF_INPUT_PUPD, PULLUP, AFIO_PartialRemap1_USART1_ENABLE)},
  {PC_1, USART1, CH_PIN_DATA(CH_MODE_INPUT, CH_CNF_INPUT_PUPD, PULLUP, AFIO_PartialRemap3_USART1_ENABLE)},
  {NC,    NP,     0}
};
#endif

#ifdef UART_MODULE_ENABLED
WEAK const PinMap PinMap_UART_RTS[] = {
  {PC_2, USART1, CH_PIN_DATA(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, 0, AFIO_NONE)}, // PC2 is RTS only with the default TX/RX mapping.
  {PC_7, USART1, CH_PIN_DATA(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, 0, AFIO_NONE)}, // PC7 is RTS with remap 1 or remap 3, AFIO_NONE avoids changing TX/RX remap
  {NC,    NP,     0}
};
#endif

#ifdef UART_MODULE_ENABLED
WEAK const PinMap PinMap_UART_CTS[] = {
//  {PD_3, USART1, CH_PIN_DATA(CH_MODE_INPUT, CH_CNF_INPUT_PUPD, PULLUP, AFIO_Remap_USART1_DISABLE)}, // no PD3 pin on V002J4
  {PC_6, USART1, CH_PIN_DATA(CH_MODE_INPUT, CH_CNF_INPUT_PUPD, PULLUP, AFIO_NONE)},  // PC6 is CTS with remap 1 or remap 3, AFIO_NONE avoids changing TX/RX remap
  {NC,    NP,     0}
};
#endif

/*

CH32V002A4M6 (SOP16) is missing PC5 ==> use SPI_4 for MISO=PD5/MOSI=PD6/SCK=PD4. Missing: SS=PD3
See also table 7-12 on page 74 of CH32V00X Reference Manual v1.4

CH32V002A4M6 SOP16   A[1,2]  - C[0,1,2,3,4,6,7]   - D[1,4,5,6,7]

V002 SPI Alternate pins
      def  _1  _2  _3  _4  _5  _6
SS    PC1 PC0 PC4   x PD3 PC1 PC4
SCK   PC5 PC5 PD2   x PD4 PA1   x
MOSI  PC6 PC6 PD3 PC0 PD6 PA2   x
MISO  PC7 PC7   x   x PD5   x PC7

(x=no port B on CH32V002)
*/

//*** SPI ***
#ifdef SPI_MODULE_ENABLED
WEAK const PinMap PinMap_SPI_MOSI[] = {
  // {PC_6, SPI1, CH_PIN_DATA(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, 0, AFIO_NONE)}, // don't use default SPI (SCK missing on A4M6)
  {PD_6, SPI1, CH_PIN_DATA(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, 0, AFIO_PartialRemap4_SPI1_ENABLE)},
  {NC,   NP,   0}
};
#endif

#ifdef SPI_MODULE_ENABLED
WEAK const PinMap PinMap_SPI_MISO[] = {
  // {PC_7, SPI1, CH_PIN_DATA(CH_MODE_INPUT, CH_CNF_INPUT_FLOAT, 0, AFIO_NONE)}, // don't use default SPI (SCK missing on A4M6)
  {PD_5, SPI1, CH_PIN_DATA(CH_MODE_INPUT, CH_CNF_INPUT_FLOAT, 0, AFIO_PartialRemap4_SPI1_ENABLE)},
  {NC,   NP,   0}
};
#endif

#ifdef SPI_MODULE_ENABLED
WEAK const PinMap PinMap_SPI_SCLK[] = {
  // {PC_5, SPI1, CH_PIN_DATA(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, 0, AFIO_NONE)}, // don't use default SPI (PC5/SCK missing on A4M6)
  {PD_4, SPI1, CH_PIN_DATA(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, 0, AFIO_PartialRemap4_SPI1_ENABLE)},
  {NC,   NP,   0}
};
#endif

#ifdef SPI_MODULE_ENABLED
WEAK const PinMap PinMap_SPI_SSEL[] = {
  // {PC_1,  SPI1, CH_PIN_DATA(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, 0, AFIO_NONE)}, // don't use default SPI (SCK missing on A4M6)
  {NC,  SPI1, CH_PIN_DATA(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, 0, AFIO_PartialRemap4_SPI1_ENABLE)},   // use software CS (PD3/SS missing on A4M6)
  {NC,    NP,   0}
};
#endif
