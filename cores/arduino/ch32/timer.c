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
modified by TempersLee 
 */
#include "core_debug.h"
#include "timer.h"
#include "board.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(TIM_MODULE_ENABLED) 

/* Private Functions */
/* Aim of the function is to get _timerObj pointer using htim pointer */
/* Highly inspired from magical linux kernel's "container_of" */
/* (which was not directly used since not compatible with IAR toolchain) */
timerObj_t *get_timer_obj(TIM_HandleTypeDef *htim)
{
  timerObj_t *obj;
  obj = (timerObj_t *)((char *)htim - offsetof(timerObj_t, handle));
  return (obj);
}

/**
  * @brief  TIMER Initialization - clock init and nvic init
  * @param  htim_base: TIM handle
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim_base)
{
  timerObj_t *obj = get_timer_obj(htim_base);
  enableTimerClock(htim_base);

  // configure Update interrupt
  NVIC_SetPriority(getTimerUpIrq(htim_base->Instance), obj->preemptPriority | obj->subPriority);
  NVIC_EnableIRQ(getTimerUpIrq(htim_base->Instance));

  if (getTimerCCIrq(htim_base->Instance) != getTimerUpIrq(htim_base->Instance)) {
    // configure Capture Compare interrupt
    NVIC_SetPriority(getTimerCCIrq(htim_base->Instance), obj->preemptPriority | obj->subPriority);
    NVIC_EnableIRQ(getTimerCCIrq(htim_base->Instance));
  }
}

/**
  * @brief  TIMER Deinitialization - clock and nvic
  * @param  htim_base: TIM handle
  * @retval None
  */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim_base)
{
  disableTimerClock(htim_base);
  NVIC_DisableIRQ(getTimerUpIrq(htim_base->Instance));
  NVIC_DisableIRQ(getTimerCCIrq(htim_base->Instance));
}

/**
  * @brief  Initializes the TIM Output Compare MSP.
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *htim)
{
  timerObj_t *obj = get_timer_obj(htim);
  enableTimerClock(htim);

  // configure Update interrupt
  NVIC_SetPriority(getTimerUpIrq(htim->Instance), obj->preemptPriority | obj->subPriority);
  NVIC_EnableIRQ(getTimerUpIrq(htim->Instance));

  if (getTimerCCIrq(htim->Instance) != getTimerUpIrq(htim->Instance)) {
    // configure Capture Compare interrupt
    NVIC_SetPriority(getTimerCCIrq(htim->Instance), obj->preemptPriority | obj->subPriority);
    NVIC_EnableIRQ(getTimerCCIrq(htim->Instance));
  }
}

/**
  * @brief  DeInitialize TIM Output Compare MSP.
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_OC_MspDeInit(TIM_HandleTypeDef *htim)
{
  disableTimerClock(htim);
  NVIC_DisableIRQ(getTimerUpIrq(htim->Instance));
  NVIC_DisableIRQ(getTimerCCIrq(htim->Instance));
}

/**
  * @brief  Initializes the TIM Input Capture MSP.
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
  enableTimerClock(htim);
}

/**
  * @brief  DeInitialize TIM Input Capture MSP.
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef *htim)
{
  disableTimerClock(htim);
}

/* Exported functions */
/**
  * @brief  Enable the timer clock
  * @param  htim: TIM handle
  * @retval None
  */
void enableTimerClock(TIM_HandleTypeDef *htim)
{
  // Enable TIM clock
#if defined(TIM1_BASE)
  if (htim->Instance == TIM1) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  }
#endif
#if defined(TIM2_BASE)
  if (htim->Instance == TIM2) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  }
#endif
#if defined(TIM3_BASE)
  if (htim->Instance == TIM3) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  }
#endif
#if defined(TIM4_BASE)
  if (htim->Instance == TIM4) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  }
#endif
#if defined(TIM5_BASE)
  if (htim->Instance == TIM5) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
  }
#endif
#if defined(TIM6_BASE)
  if (htim->Instance == TIM6) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);    
  }
#endif
#if defined(TIM7_BASE)
  if (htim->Instance == TIM7) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);    
  }
#endif
#if defined(TIM8_BASE)
  if (htim->Instance == TIM8) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);    
  }
#endif
#if defined(TIM9_BASE)
  if (htim->Instance == TIM9) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);    
  }
#endif
#if defined(TIM10_BASE)
  if (htim->Instance == TIM10) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);    
  }
#endif

}

/**
  * @brief  Disable the timer clock
  * @param  htim: TIM handle
  * @retval None
  */
void disableTimerClock(TIM_HandleTypeDef *htim)
{
  // Enable TIM clock
#if defined(TIM1_BASE)
  if (htim->Instance == TIM1) {
     RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, DISABLE);
  }
#endif
#if defined(TIM2_BASE)
  if (htim->Instance == TIM2) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, DISABLE);
  }
#endif
#if defined(TIM3_BASE)
  if (htim->Instance == TIM3) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, DISABLE);
  }
#endif
#if defined(TIM4_BASE)
  if (htim->Instance == TIM4) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, DISABLE);
  }
#endif
#if defined(TIM5_BASE)
  if (htim->Instance == TIM5) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, DISABLE);
  }
#endif
#if defined(TIM6_BASE)
  if (htim->Instance == TIM6) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, DISABLE);
  }
#endif
#if defined(TIM7_BASE)
  if (htim->Instance == TIM7) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, DISABLE);
  }
#endif
#if defined(TIM8_BASE)
  if (htim->Instance == TIM8) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, DISABLE);
  }
#endif
#if defined(TIM9_BASE)
  if (htim->Instance == TIM9) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, DISABLE);
  }
#endif
#if defined(TIM10_BASE)
  if (htim->Instance == TIM10) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, DISABLE);
  }
#endif
}

/**
  * @brief  This function return IRQ number corresponding to update interrupt event of timer instance.
  * @param  tim: timer instance
  * @retval IRQ number
  */
IRQn_Type getTimerUpIrq(TIM_TypeDef *tim)
{
  IRQn_Type IRQn = NonMaskableInt_IRQn;

  if (tim != (TIM_TypeDef *)NC) {
    /* Get IRQn depending on TIM instance */
    switch ((uint32_t)tim) {
#if defined(TIM1_BASE)
      case (uint32_t)TIM1_BASE:
        IRQn = TIM1_UP_IRQn;
        break;
#endif
#if defined(TIM2_BASE)
      case (uint32_t)TIM2_BASE:
        IRQn = TIM2_IRQn;
        break;
#endif
#if defined(TIM3_BASE)
      case (uint32_t)TIM3_BASE:
        IRQn = TIM3_IRQn;
        break;
#endif
#if defined(TIM4_BASE)
      case (uint32_t)TIM4_BASE:
        IRQn = TIM4_IRQn;
        break;
#endif
#if defined(TIM5_BASE) && !defined(CH32V10x) && !defined(CH32V20x)
      case (uint32_t)TIM5_BASE:
        IRQn = TIM5_IRQn;
        break;
#endif
#if defined(TIM6_BASE) && !defined(CH32V10x) && !defined(CH32V20x)
      case (uint32_t)TIM6_BASE:
        IRQn = TIM6_IRQn;
        break;
#endif
#if defined(TIM7_BASE) && !defined(CH32V10x) && !defined(CH32V20x)
      case (uint32_t)TIM7_BASE:
        IRQn = TIM7_IRQn;
        break;
#endif
#if defined(TIM8_BASE) && !defined(CH32V10x) && !defined(CH32V20x)
      case (uint32_t)TIM8_BASE:
        IRQn = TIM8_UP_IRQn;
        break;
#endif
#if defined(TIM9_BASE) && !defined(CH32V10x) && !defined(CH32V20x)
      case (uint32_t)TIM9_BASE:
        IRQn = TIM9_UP_IRQn;
        break;
#endif
#if defined(TIM10_BASE) && !defined(CH32V10x) && !defined(CH32V20x)
      case (uint32_t)TIM10_BASE:
        IRQn = TIM10_UP_IRQn;
        break;
#endif
      default:
        _Error_Handler("TIM: Unknown timer IRQn", (int)tim);
        break;
    }
  }
  return IRQn;
}

/**
  * @brief  This function return IRQ number corresponding to Capture or Compare interrupt event of timer instance.
  * @param  tim: timer instance
  * @retval IRQ number
  */
IRQn_Type getTimerCCIrq(TIM_TypeDef *tim)
{
  IRQn_Type IRQn = NonMaskableInt_IRQn;

  if (tim != (TIM_TypeDef *)NC) {
    /* Get IRQn depending on TIM instance */
    switch ((uint32_t)tim) {
#if defined(TIM1_BASE)
      case (uint32_t)TIM1_BASE:
        IRQn = TIM1_CC_IRQn;
        break;
#endif
#if defined(TIM2_BASE)
      case (uint32_t)TIM2_BASE:
        IRQn = TIM2_IRQn;
        break;
#endif
#if defined(TIM3_BASE)
      case (uint32_t)TIM3_BASE:
        IRQn = TIM3_IRQn;
        break;
#endif
#if defined(TIM4_BASE)
      case (uint32_t)TIM4_BASE:
        IRQn = TIM4_IRQn;
        break;
#endif
#if defined(TIM5_BASE) && !defined(CH32V20x) && !defined(CH32V10x)  
      case (uint32_t)TIM5_BASE:
        IRQn = TIM5_IRQn;
        break;
#endif
#if defined(TIM6_BASE) && !defined(CH32V20x) && !defined(CH32V10x)  
      case (uint32_t)TIM6_BASE:
        IRQn = TIM6_IRQn;
        break;
#endif
#if defined(TIM7_BASE) && !defined(CH32V20x) && !defined(CH32V10x)  
      case (uint32_t)TIM7_BASE:
        IRQn = TIM7_IRQn;
        break;
#endif
#if defined(TIM8_BASE) && !defined(CH32V20x) && !defined(CH32V10x)  
      case (uint32_t)TIM8_BASE:
        IRQn = TIM8_CC_IRQn;
        break;
#endif
#if defined(TIM9_BASE) && !defined(CH32V20x) && !defined(CH32V10x)  
      case (uint32_t)TIM9_BASE:
        IRQn = TIM9_CC_IRQn;
        break;
#endif
#if defined(TIM10_BASE) && !defined(CH32V20x) && !defined(CH32V10x)  
      case (uint32_t)TIM10_BASE:
        IRQn = TIM10_CC_IRQn;
        break;
#endif
      default:
        _Error_Handler("TIM: Unknown timer IRQn", (int)tim);
        break;
    }
  }
  return IRQn;
}

/**
  * @brief  This function return the timer clock source.
  * @param  tim: timer instance
  * @retval 1 = PCLK1 or 2 = PCLK2
  */
uint8_t getTimerClkSrc(TIM_TypeDef *tim)
{
  uint8_t clkSrc = 0;

  if (tim != (TIM_TypeDef *)NC)
  {
    /* Get source clock depending on TIM instance */
    switch ((uint32_t)tim) {
#if defined(TIM2_BASE)
      case (uint32_t)TIM2:
#endif
#if defined(TIM3_BASE)
      case (uint32_t)TIM3:
#endif
#if defined(TIM4_BASE)
      case (uint32_t)TIM4:
#endif
#if defined(TIM5_BASE)
      case (uint32_t)TIM5:
#endif
#if defined(TIM6_BASE)
      case (uint32_t)TIM6:
#endif
#if defined(TIM7_BASE)
      case (uint32_t)TIM7:
#endif
        clkSrc = 1;
        break;
#if defined(TIM1_BASE)
      case (uint32_t)TIM1:
#endif
#if defined(TIM8_BASE)
      case (uint32_t)TIM8:
#endif
#if defined(TIM9_BASE)
      case (uint32_t)TIM9:
#endif
#if defined(TIM10_BASE)
      case (uint32_t)TIM10:
#endif
        clkSrc = 2;
        break;
      default:
        _Error_Handler("TIM: Unknown timer instance", (int)tim);
        break;
    }
  }
  return clkSrc;
}

/**
  * @brief  Return HAL timer channel linked to a PinName
  * @param  pin: PinName
  * @retval Valid HAL channel
  */
uint32_t getTimerChannel(PinName pin)
{
  uint32_t function = pinmap_function(pin, PinMap_TIM);
  uint32_t channel = 0;
  switch (CH_PIN_CHANNEL(function)) {
    case 1:
      channel = TIM_Channel_1; 
      break;
    case 2:
      channel = TIM_Channel_2;
      break;
    case 3:
      channel = TIM_Channel_3;
      break;
    case 4:
      channel = TIM_Channel_4;
      break;
    default:
      _Error_Handler("TIM: Unknown timer channel", (int)(CH_PIN_CHANNEL(function)));
      break;
  }
  return channel;
}

#endif /* TIM_MODULE_ENABLED */

#ifdef __cplusplus
}
#endif


