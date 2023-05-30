/**
  ******************************************************************************
  * @file    interrupt.h
  * @author  WI6LABS
  * @version V1.0.0
  * @date    01-August-2016
  * @brief   Header for interrupt module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
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
  *
  ******************************************************************************
  Modified 1 may 2023 by TempersLee
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __INTERRUPT_H
#define __INTERRUPT_H

/* Includes ------------------------------------------------------------------*/
#include "ch32_def.h"
#include "Arduino.h"

#if !defined(EXTI_MODULE_DISABLED)


  #ifndef EXTI_IRQ_PRIO
      #define EXTI_IRQ_PRIO       (0xFF)
  #endif /* EXTI_IRQ_PRIO */


  /* Exported functions ------------------------------------------------------- */
  void ch32_interrupt_disable(GPIO_TypeDef *port, uint16_t pin);
  void ch32_interrupt_enable(GPIO_TypeDef *port, GPIOMode_TypeDef io_mode,uint16_t pin, void (*callback)(void), EXTIMode_TypeDef it_mode, EXTITrigger_TypeDef trigger_mode);


#endif


#endif /* __INTERRUPT_H */


