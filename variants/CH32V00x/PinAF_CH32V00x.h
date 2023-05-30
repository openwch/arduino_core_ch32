/*
 *******************************************************************************
 * Copyright (c) 2017, STMicroelectronics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************
 Modified 1 may 2023 by TempersLee
 */

#ifndef _PINAF_CH32V20X_H
#define _PINAF_CH32V20X_H

#ifdef __cplusplus
extern "C" {
#endif

enum {
  AFIO_NONE,
    // ENABLE:
    /* GPIO_Remap_define */
    /* PCFR1 */
  AFIO_Remap_SPI1_ENABLE,
  AFIO_Remap_SPI1_DISABLE,
  
  AFIO_PartialRemap_I2C1_ENABLE,
  AFIO_FullRemap_I2C1_ENABLE,
  AFIO_Remap_I2C1_DISABLE,

  AFIO_PartialRemap1_USART1_ENABLE,
  AFIO_PartialRemap2_USART1_ENABLE,
  AFIO_FullRemap_USART1_ENABLE,
  AFIO_Remap_USART1_DISABLE,

  AFIO_PartialRemap1_TIM1_ENABLE,
  AFIO_PartialRemap2_TIM1_ENABLE,
  AFIO_FullRemap_TIM1_ENABLE,
  AFIO_Remap_TIM1_DISABLE,

  AFIO_PartialRemap1_TIM2_ENABLE,
  AFIO_PartialRemap2_TIM2_ENABLE,
  AFIO_FullRemap_TIM2_ENABLE,
  AFIO_Remap_TIM2_DISABLE,

  AFIO_Remap_PA1_2_ENABLE,
  AFIO_Remap_PA1_2_DISABLE,

  AFIO_Remap_ADC1_ETRGINJ_ENBALE,
  AFIO_Remap_ADC1_ETRGINJ_DISABLE,
  AFIO_Remap_ADC1_ETRGREG_ENABLE,
  AFIO_Remap_ADC1_ETRGREG_DISABLE,

  AFIO_Remap_LSI_CAL_ENABLE,
  AFIO_Remap_LSI_CAL_DISABLE,

  AFIO_Remap_SDI_Disable_ENABLE,
  AFIO_Remap_SDI_Disable_DISABLE

};

static inline void pinV32_DisconnectDebug(PinName pin)
{
  /** Enable this flag gives the possibility to use debug pins without any risk
    * to lose traces
    */
#ifndef CH32V_LOCK_DEBUG
  // Enable AFIO clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  // Disconnect JTAG-DP + SW-DP signals.
  // Warning: Need to reconnect under reset
  if ((pin == PD_1) ) 
  {
     GPIO_PinRemapConfig(GPIO_Remap_SDI_Disable, ENABLE);  // JTAG-DP Disabled and SW-DP Disabled
  }
#else
  (void)(pin);
#endif 
}

static inline void pin_SetV32AFPin(uint32_t afnum)
{
  // Enable AFIO clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  switch (afnum) {
    case AFIO_Remap_SPI1_ENABLE:
      GPIO_PinRemapConfig(GPIO_Remap_SPI1,ENABLE);
      break;
    case AFIO_Remap_SPI1_DISABLE:
      GPIO_PinRemapConfig(GPIO_Remap_SPI1,DISABLE);
      break;
    case AFIO_PartialRemap_I2C1_ENABLE:
      GPIO_PinRemapConfig(GPIO_PartialRemap_I2C1,ENABLE);
      break;      
    case AFIO_FullRemap_I2C1_ENABLE:
      GPIO_PinRemapConfig(GPIO_FullRemap_I2C1,ENABLE);
      break;
    case AFIO_Remap_I2C1_DISABLE:
      GPIO_PinRemapConfig(GPIO_FullRemap_I2C1,DISABLE);
      break;
    case AFIO_PartialRemap1_USART1_ENABLE:
      GPIO_PinRemapConfig(GPIO_PartialRemap1_USART1,ENABLE);
      break;
    case AFIO_PartialRemap2_USART1_ENABLE:
      GPIO_PinRemapConfig(GPIO_PartialRemap2_USART1,ENABLE);
      break;
    case AFIO_FullRemap_USART1_ENABLE:
      GPIO_PinRemapConfig(GPIO_FullRemap_USART1,ENABLE);
      break;         
    case AFIO_Remap_USART1_DISABLE:
      GPIO_PinRemapConfig(GPIO_FullRemap_USART1,DISABLE);
      break;         
    case AFIO_PartialRemap1_TIM1_ENABLE:
      GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM1,ENABLE);
      break;         
    case AFIO_PartialRemap2_TIM1_ENABLE:
      GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM1,ENABLE);
      break;  
    case AFIO_FullRemap_TIM1_ENABLE:
      GPIO_PinRemapConfig(GPIO_FullRemap_TIM1,ENABLE);
      break;  
    case AFIO_Remap_TIM1_DISABLE:
      GPIO_PinRemapConfig(GPIO_FullRemap_TIM1,DISABLE);
      break;  
    case AFIO_PartialRemap1_TIM2_ENABLE:
      GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2,ENABLE);
      break; 
    case AFIO_PartialRemap2_TIM2_ENABLE:
      GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2,ENABLE);
      break; 
    case AFIO_FullRemap_TIM2_ENABLE:
      GPIO_PinRemapConfig(GPIO_FullRemap_TIM2,ENABLE);
      break; 
    case AFIO_Remap_TIM2_DISABLE:
      GPIO_PinRemapConfig(GPIO_FullRemap_TIM2,DISABLE);
      break; 
    case AFIO_Remap_PA1_2_ENABLE:
      GPIO_PinRemapConfig(GPIO_Remap_PA1_2,ENABLE);
      break;
    case AFIO_Remap_PA1_2_DISABLE:
      GPIO_PinRemapConfig(GPIO_Remap_PA1_2,DISABLE);
      break;
    case AFIO_Remap_ADC1_ETRGINJ_ENBALE:
      GPIO_PinRemapConfig(GPIO_Remap_ADC1_ETRGINJ,ENABLE);
      break;
    case AFIO_Remap_ADC1_ETRGINJ_DISABLE:
      GPIO_PinRemapConfig(GPIO_Remap_ADC1_ETRGINJ,DISABLE);
      break;
    case AFIO_Remap_ADC1_ETRGREG_ENABLE:
      GPIO_PinRemapConfig(GPIO_Remap_ADC1_ETRGREG,ENABLE);
      break;
    case AFIO_Remap_ADC1_ETRGREG_DISABLE:
      GPIO_PinRemapConfig(GPIO_Remap_ADC1_ETRGREG,DISABLE);
      break;
    case AFIO_Remap_LSI_CAL_ENABLE:
      GPIO_PinRemapConfig(GPIO_Remap_LSI_CAL,ENABLE);
      break;
    case AFIO_Remap_LSI_CAL_DISABLE:
      GPIO_PinRemapConfig(GPIO_Remap_LSI_CAL,DISABLE);
      break;
    case AFIO_Remap_SDI_Disable_ENABLE:
      GPIO_PinRemapConfig(GPIO_Remap_SDI_Disable,ENABLE);
      break;
    case AFIO_Remap_SDI_Disable_DISABLE:
      GPIO_PinRemapConfig(GPIO_Remap_SDI_Disable,DISABLE);
      break;
      
    default:
    case AFIO_NONE:
      break;
  }
}

#ifdef __cplusplus
}
#endif

#endif /* _PINAF_CH32V20X_H */
