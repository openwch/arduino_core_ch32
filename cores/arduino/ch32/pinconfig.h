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
#ifndef _PINCONFIG_H
#define _PINCONFIG_H

#include "PinAF_ch32yyxx.h"
#include "ch32yyxx_gpio.h"

#define GPIO_PULLUP      0x00000001
#define GPIO_PULLDOWN    0x00000002

static inline void pin_DisconnectDebug(PinName pin)
{
  pinV32_DisconnectDebug(pin);
}




static inline void pin_PullConfig(GPIO_TypeDef *gpio, uint32_t pin, uint32_t pull_config)
{
    GPIO_InitTypeDef init={0} ;
    switch (pull_config)
    {
    case GPIO_PULLUP:
      init.GPIO_Pin = pin;
      init.GPIO_Speed = 0 ;  //unchange speed 
      init.GPIO_Mode = GPIO_Mode_IPU;
      GPIO_Init(gpio, &init);
      break;
    case GPIO_PULLDOWN:
      init.GPIO_Pin = pin;
      init.GPIO_Speed = 0 ;  //unchange speed 
      init.GPIO_Mode = GPIO_Mode_IPD;
      GPIO_Init(gpio, &init);
      break;  

    default:
      init.GPIO_Pin = pin;
      init.GPIO_Speed = 0 ;  //unchange speed 
      init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
      GPIO_Init(gpio, &init);
      break;
    }

}

static inline void pin_SetAFPin(GPIO_TypeDef *gpio, PinName pin, uint32_t afnum)
{

  (void)(gpio);
  (void)(pin);
  pin_SetV32AFPin(afnum);
}

#endif
