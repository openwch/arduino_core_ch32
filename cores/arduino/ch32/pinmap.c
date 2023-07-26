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
#include "pinmap.h"
#include "pinconfig.h"
#include "ch32yyxx_gpio.h"
#include "system_ch32yyxx.h"

typedef struct {
  PinName pin;
  uint32_t LL_AnalogSwitch;
} PinAnalogSwitch;


#if defined(CH32V10x) || defined(CH32V20x) || defined(CH32V30x) || defined(CH32V30x_C)
const uint32_t pin_map[16] = {
  GPIO_Pin_0,
  GPIO_Pin_1,
  GPIO_Pin_2,
  GPIO_Pin_3,
  GPIO_Pin_4,
  GPIO_Pin_5,
  GPIO_Pin_6,
  GPIO_Pin_7
  , GPIO_Pin_8
  , GPIO_Pin_9
  , GPIO_Pin_10
  , GPIO_Pin_11
  , GPIO_Pin_12
  , GPIO_Pin_13
  , GPIO_Pin_14
  , GPIO_Pin_15

};
#elif defined(CH32V00x)
const uint32_t pin_map[8] = {
  GPIO_Pin_0,
  GPIO_Pin_1,
  GPIO_Pin_2,
  GPIO_Pin_3,
  GPIO_Pin_4,
  GPIO_Pin_5,
  GPIO_Pin_6,
  GPIO_Pin_7
};

#elif defined(CH32X035)
const uint32_t pin_map[24] = {
  GPIO_Pin_0,
  GPIO_Pin_1,
  GPIO_Pin_2,
  GPIO_Pin_3,
  GPIO_Pin_4,
  GPIO_Pin_5,
  GPIO_Pin_6,
  GPIO_Pin_7,
  GPIO_Pin_8,
  GPIO_Pin_9,
  GPIO_Pin_10,
  GPIO_Pin_11,
  GPIO_Pin_12,
  GPIO_Pin_13,
  GPIO_Pin_14,
  GPIO_Pin_15,
  GPIO_Pin_16,
  GPIO_Pin_17,
  GPIO_Pin_18,
  GPIO_Pin_19,
  GPIO_Pin_20,
  GPIO_Pin_21,
  GPIO_Pin_22,
  GPIO_Pin_23
};
#endif




bool pin_in_pinmap(PinName pin, const PinMap *map)
{
  if (pin != (PinName)NC) {
    while (map->pin != NC) {
      if (map->pin == pin) {
        return true;
      }
      map++;
    }
  }
  return false;
}

/**
 * Configure pin (mode, speed, output type and pull-up/pull-down)
 */
void pin_function(PinName pin, int function)
{
  /* Get the pin information */
   GPIO_InitTypeDef GPIO_InitStructure = {0};
   
  uint32_t mode  = CH_PIN_MODE(function);
  uint32_t cnf   = CH_PIN_CNF(function); 
  uint32_t pupd  = CH_PIN_PUPD(function);
  uint32_t afnum = CH_PIN_AFNUM(function);
  uint32_t port = CH_PORT(pin);
  uint32_t ch_pinx  = CH_MAP_GPIO_PIN(pin);

  if (pin == (PinName)NC) {
    Error_Handler();
  }

  /* Enable GPIO clock */
  GPIO_TypeDef *gpio = set_GPIO_Port_Clock(port);
  /* if have afnum, initial */
  if(afnum) pin_SetV32AFPin(afnum);

  if(mode) //output with speed seting
  {
      GPIO_InitStructure.GPIO_Speed =  mode;
      switch (cnf)
      {
          case CH_CNF_OUTPUT_PP:
               GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
               break;
          case CH_CNF_OUTPUT_OD:
               GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
               break;
          case CH_CNF_OUTPUT_AFPP:
               GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
               break;
          case CH_CNF_OUTPUT_AFOD:
               GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
               break;           
          default:
               GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
               break;
      }
  }
  else //input
  {
      switch (cnf)
      {
          case CH_CNF_INPUT_ANALOG:
               GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; 
               break;
          case CH_CNF_INPUT_FLOAT:
               GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
               break;
          case CH_CNF_INPUT_PUPD:
               if(pupd == PULLUP) GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
               else if(pupd == PILLDOWN) GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;   
               else GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 
               break;      
          default:
               GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
               break;
      }

  }

  GPIO_InitStructure.GPIO_Pin = (uint16_t) ch_pinx;
  GPIO_Init(gpio, &GPIO_InitStructure);
}




void pinmap_pinout(PinName pin, const PinMap *map)
{
  if (pin == NC) {
    return;
  }

  while (map->pin != NC) {
    if (map->pin == pin) {
      pin_function(pin, map->function);
      return;
    }
    map++;
  }
  Error_Handler();
}




void *pinmap_find_peripheral(PinName pin, const PinMap *map)
{
  while (map->pin != NC) {
    if (map->pin == pin) {
      return map->peripheral;
    }
    map++;
  }
  return NP;
}


void *pinmap_peripheral(PinName pin, const PinMap *map)
{
  void *peripheral = NP;

  if (pin != (PinName)NC) {
    peripheral = pinmap_find_peripheral(pin, map);
  }
  return peripheral;
}



PinName pinmap_find_pin(void *peripheral, const PinMap *map)
{
  while (map->peripheral != NP) {
    if (map->peripheral == peripheral) {
      return map->pin;
    }
    map++;
  }
  return NC;
}



PinName pinmap_pin(void *peripheral, const PinMap *map)
{
  PinName pin = NC;

  if (peripheral != NP) {
    pin = pinmap_find_pin(peripheral, map);
  }
  return pin;
}


uint32_t pinmap_find_function(PinName pin, const PinMap *map)
{
  while (map->pin != NC) {
    if (map->pin == pin) {
      return map->function;
    }
    map++;
  }
  return (uint32_t)NC;
}


uint32_t pinmap_function(PinName pin, const PinMap *map)
{
  uint32_t function = (uint32_t)NC;

  if (pin != (PinName)NC) {
    function = pinmap_find_function(pin, map);
  }
  return function;
}


// Merge peripherals
void *pinmap_merge_peripheral(void *a, void *b)
{
  // both are the same (inc both NP)
  if (a == b) {
    return a;
  }

  // one (or both) is not set
  if (a == NP) {
    return b;
  }
  if (b == NP) {
    return a;
  }

  // mismatch error case
  return NP;
}

