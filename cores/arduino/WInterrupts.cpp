/*
  Copyright (c) 2011-2012 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "WInterrupts.h"
#include "Arduino.h"
#include "PinAF_ch32yyxx.h"

#include "interrupt.h"


#if !defined(EXTI_MODULE_DISABLED)

void attachInterrupt(uint32_t pin, GPIOMode_TypeDef io_mode,  void (*callback)(void), EXTIMode_TypeDef it_mode, EXTITrigger_TypeDef trigger_mode)
{
  PinName p = digitalPinToPinName(pin);
  GPIO_TypeDef* port = set_GPIO_Port_Clock(CH_PORT(p));
  if (!port) return ;
  pinV32_DisconnectDebug(p);

  ch32_interrupt_enable(port, io_mode, CH_GPIO_PIN(p), callback, it_mode, trigger_mode);
}


void detachInterrupt(uint32_t pin)
{
  PinName p = digitalPinToPinName(pin);
  GPIO_TypeDef* port = get_GPIO_Port(CH_PORT(p));
  if (!port)
	  return;
  ch32_interrupt_disable(port, CH_GPIO_PIN(p));
}

#endif