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
#include "PortNames.h"

GPIO_TypeDef *GPIOPort[MAX_NB_PORT] = {
    GPIOA
#if defined GPIOB_BASE    
  , GPIOB
#endif 
#if defined GPIOC_BASE
  , GPIOC
#endif
#if defined GPIOD_BASE
  , GPIOD
#endif
#if defined GPIOE_BASE
  , GPIOE
#endif
#if defined GPIOF_BASE
  , GPIOF
#endif
#if defined GPIOG_BASE
  , GPIOG
#endif
#if defined GPIOH_BASE
  , GPIOH
#endif
};

/* Enable GPIO clock and return GPIO base address */
GPIO_TypeDef *set_GPIO_Port_Clock(uint32_t port_idx)
{
  GPIO_TypeDef *gpioPort = 0;
  switch (port_idx) {
    case PortA:
      gpioPort = GPIOA;
      #if defined(CH32L10x) || defined(CH32VM00X)
      RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA, ENABLE);
      #else
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
      #endif
      break;
#if defined GPIOB_BASE      
    case PortB:
      gpioPort = GPIOB;
      #if defined(CH32L10x) || defined(CH32VM00X)
      RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOB, ENABLE);      
      #else
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
      #endif
      break;
#endif       
#if defined GPIOC_BASE
    case PortC:
      gpioPort = GPIOC;
      #if defined(CH32L10x) || defined(CH32VM00X)
      RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC, ENABLE);
      #else
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
      #endif
      break;
#endif
#if defined GPIOD_BASE
    case PortD:
      gpioPort = GPIOD;
      #if defined(CH32L10x) || defined(CH32VM00X)
      RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOD, ENABLE);          
      #else
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
      #endif
      break;
#endif
#if defined GPIOE_BASE
    case PortE:
      gpioPort = GPIOE;
      #if defined(CH32L10x) 
      RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOE, ENABLE);
      #else
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
      #endif      
      break;
#endif
#if defined RCC_APB2Periph_GPIOF
    case PortF:
      gpioPort = GPIOF;
      #if defined(CH32L10x) 
      RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOF, ENABLE);      
      #else 
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
      #endif    
      break;
#endif
#if defined RCC_APB2Periph_GPIOG
    case PortG:
      gpioPort = GPIOG;
      #if defined(CH32L10x)      
      RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOG, ENABLE);
      #else
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
      #endif
      break;
#endif
#if defined RCC_APB2Periph_GPIOH
    case PortH:
      gpioPort = GPIOH;
      #if defined(CH32L10x)
      RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOH, ENABLE);
      #else
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOH, ENABLE);      
      #endif
      break;
#endif
    default:
      // wrong port number
      //TBD: error management
      gpioPort = 0;
      break;
  }
  return gpioPort;
}
