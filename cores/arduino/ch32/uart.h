/**
  ******************************************************************************
  * @file    uart.h
  * @author  WI6LABS, fpistm
  * @brief   Header for uart module
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
#ifndef __UART_H
#define __UART_H

/* Includes ------------------------------------------------------------------*/
#include "ch32_def.h"
#include "PinNames.h"
#include "variant.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(UART_MODULE_ENABLED) 

typedef struct __UART_HandleTypeDef
{
  USART_TypeDef                 *Instance;       
  USART_InitTypeDef              init;       
}UART_HandleTypeDef;


  #ifndef UART_IRQ_PRIO
  #define UART_IRQ_PRIO       (0x80)
  #endif
  #ifndef UART_IRQ_SUBPRIO
  #define UART_IRQ_SUBPRIO    (0x00)
  #endif



  typedef struct serial_s serial_t;
  struct serial_s 
  {
    USART_TypeDef       *uart;  
    UART_HandleTypeDef   handle;   
    PinName pin_tx;
    PinName pin_rx;
    PinName pin_rts;
    PinName pin_cts;
    IRQn_Type irq;
    uint8_t index;
  };




  #define TX_TIMEOUT  1000


  /* Exported macro ------------------------------------------------------------*/
  /* Exported functions ------------------------------------------------------- */
  void uart_init(serial_t *obj, uint32_t baudrate, uint32_t databits, uint32_t parity, uint32_t stopbits);
  void uart_deinit(serial_t *obj);

  int uart_getc(serial_t *obj, unsigned char *c);

  uint8_t serial_tx_active(serial_t *obj);
  uint8_t serial_rx_active(serial_t *obj);

  size_t uart_debug_write(uint8_t *data, uint32_t size);

#else

  #define serial_t void*

#endif /* UART_MODULE_ENABLED  */

#ifdef __cplusplus
}
#endif

#endif /* __UART_H */

