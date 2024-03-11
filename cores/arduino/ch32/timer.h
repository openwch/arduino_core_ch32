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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIMER_H
#define __TIMER_H

/* Includes ------------------------------------------------------------------*/
#include "ch32_def.h"
#include "PinNames.h"
#include "variant.h"


#ifdef __cplusplus
extern "C" {
#endif

#if defined(TIM_MODULE_ENABLED) 

/* Exported constants --------------------------------------------------------*/
#ifndef TIM_IRQ_PRIO
#define TIM_IRQ_PRIO       0x80
#endif /* TIM_IRQ_PRIO */


#ifndef TIM_IRQ_SUBPRIO
#define TIM_IRQ_SUBPRIO    0x40
#endif




typedef enum {
#if defined(TIM1_BASE)
  TIMER1_INDEX,
#endif
#if defined(TIM2_BASE)
  TIMER2_INDEX,
#endif
#if defined(TIM3_BASE)
  TIMER3_INDEX,
#endif
#if defined(TIM4_BASE)
  TIMER4_INDEX,
#endif
#if defined(TIM5_BASE)
  TIMER5_INDEX,
#endif
#if defined(TIM6_BASE)
  TIMER6_INDEX,
#endif
#if defined(TIM7_BASE)
  TIMER7_INDEX,
#endif
#if defined(TIM8_BASE)
  TIMER8_INDEX,
#endif
#if defined(TIM9_BASE)
  TIMER9_INDEX,
#endif
#if defined(TIM10_BASE)
  TIMER10_INDEX,
#endif
  TIMER_NUM,
  UNKNOWN_TIMER = 0XFFFF
} timer_index_t;


typedef enum
{
  HAL_TIM_ACTIVE_CHANNEL_1        = 0x01U,    /*!< The active channel is 1     */
  HAL_TIM_ACTIVE_CHANNEL_2        = 0x02U,    /*!< The active channel is 2     */
  HAL_TIM_ACTIVE_CHANNEL_3        = 0x04U,    /*!< The active channel is 3     */
  HAL_TIM_ACTIVE_CHANNEL_4        = 0x08U,    /*!< The active channel is 4     */
  HAL_TIM_ACTIVE_CHANNEL_5        = 0x10U,    /*!< The active channel is 5     */
  HAL_TIM_ACTIVE_CHANNEL_6        = 0x20U,    /*!< The active channel is 6     */
  HAL_TIM_ACTIVE_CHANNEL_CLEARED  = 0x00U     /*!< All active channels cleared */
} TIM_ActiveChannel;


typedef struct 
{
  TIM_TypeDef                          *Instance;         
  TIM_TimeBaseInitTypeDef               Init;     
  TIM_ActiveChannel                     Channel;   //ÊÇ·ñÐèÒª£¿
  /*    //Not yet considered          
  TIM_OCInitTypeDef                     OC_Init;
  TIM_ICInitTypeDef                     IC_Init;
  */
}TIM_HandleTypeDef;


// This structure is used to be able to get HardwareTimer instance (C++ class)
// from handler (C structure) specially for interrupt management
typedef struct  {
  // Those 2 first fields must remain in this order at the beginning of the structure
  void    *__this;
  TIM_HandleTypeDef handle;
  uint32_t preemptPriority;
  uint32_t subPriority;
} timerObj_t;

/* Exported functions ------------------------------------------------------- */
timerObj_t *get_timer_obj(TIM_HandleTypeDef *htim);

void enableTimerClock(TIM_HandleTypeDef *htim);
void disableTimerClock(TIM_HandleTypeDef *htim);

uint32_t getTimerIrq(TIM_TypeDef *tim);
uint8_t getTimerClkSrc(TIM_TypeDef *tim);

IRQn_Type getTimerUpIrq(TIM_TypeDef *tim);
IRQn_Type getTimerCCIrq(TIM_TypeDef *tim);

uint32_t getTimerChannel(PinName pin);

#endif /* TIM_MODULE_ENABLED && TIM_MODULE_ONLY */

#ifdef __cplusplus
}
#endif

#endif /* __TIMER_H */


