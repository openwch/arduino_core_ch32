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
#include "backup.h"
#include "clock.h"
#include "core_riscv_ch32yyxx.h"
#include "ch32yyxx_rcc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TICK_FREQ_1KHz    1L
// #define TICK_FREQ_100Hz   10L
// #define TICK_FREQ_10Hz    100L 


__IO uint64_t msTick=0;
WEAK uint64_t GetTick(void)
{
  return msTick;
}


void osSystickHandler() __attribute__((weak, alias("noOsSystickHandler")));
void noOsSystickHandler()
{

}

/**
  * @brief  Function called wto read the current millisecond
  * @param  None
  * @retval None
  */
uint32_t getCurrentMillis(void)
{
  return GetTick();
}



#if defined(CH32V20x) || defined(CH32V30x) || defined(CH32V30x_C) || defined(CH32V00x) || defined(CH32X035) || defined(CH32L10x) || defined(CH32VM00X)

/* SysTick ticks -> microseconds as (ticks * k) >> 20, k = (1000 << 20) / ticks
   per ms. 32-bit only: the 64-bit multiply and divide used before are library
   calls on chips without a hardware divider (about 20 us on the V003/V006).
   k is computed once; ticks < ticks per ms, so ticks * k < 2^30. */
static uint32_t us_tms, us_k;

static inline uint32_t ticks_to_us(uint32_t ticks, uint32_t tms)
{
  if (tms != us_tms) {
    us_k = 1048576000u / tms;
    us_tms = tms;
  }
  return (ticks * us_k) >> 20;
}

uint32_t getCurrentMicros(void)
{
  /* low words of CNT/CMP (64-bit registers on V20x/V30x/X035/L10x) */
  __IO uint32_t *cnt = (__IO uint32_t *)&SysTick->CNT;
  uint32_t tms = *(__IO uint32_t *)&SysTick->CMP + 1;
  uint32_t m, u, wrapped;

  /* SysTick restarts at CMP and sets SR bit 0; SysTick_Handler then adds the
     millisecond and clears the flag. Read the count until no handler ran in
     between; if the flag is still set, the restart happened but the handler
     has not run yet (a few cycles, or interrupts off): count that ms here.
     Without this, micros() stepped back by 1 ms and delay() ended early (#65). */
  do {
    m = (uint32_t)GetTick();
    u = *cnt;
    wrapped = SysTick->SR & 1;
  } while (m != (uint32_t)GetTick());
  if (wrapped) {
    u = *cnt;      /* read after the restart */
    m++;
  }
  return m * 1000 + ticks_to_us(u, tms);
}


/*********************************************************************
 * @fn      SysTick_Handler
 *
 * @brief   This function handles systick interrupt.
 *
 * @return  none
 */
void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void __attribute__((weak)) SysTick_Handler(void)
{
  msTick+=TICK_FREQ_1KHz;
  osSystickHandler();
  SysTick->SR = 0;
}

#endif





// for 10x serils (Qingke V3A) 
#if defined (CH32V10x)

#define SYSTICK_CNTL    (0xE000F004)   
#define SYSTICK_CNTH    (0xE000F008)
#define SYSTICK_CMPL    (0xE000F00C)
#define SYSTICK_CMPH    (0xE000F010)

/* Same as above: 32-bit only, (ticks * k) >> 20 with k computed once */
static uint32_t us_tms, us_k;

static inline uint32_t ticks_to_us(uint32_t ticks, uint32_t tms)
{
  if (tms != us_tms) {
    us_k = 1048576000u / tms;
    us_tms = tms;
  }
  return (ticks * us_k) >> 20;
}

uint32_t getCurrentMicros(void)
{
  /* SysTick restarts every ms: the low words are enough */
  uint32_t m0 = GetTick();
  uint32_t u0 = *((__IO uint32_t *)SYSTICK_CNTL);
  uint32_t m1 = GetTick();
  uint32_t u1 = *((__IO uint32_t *)SYSTICK_CNTL); //may be a interruption
  uint32_t tms = *((__IO uint32_t *)SYSTICK_CMPL) + 1;

  if (m1 != m0) {
    return m1 * 1000 + ticks_to_us(u1, tms);  // fix #65 - micros() is non-monotonic
  } else {
    return m0 * 1000 + ticks_to_us(u0, tms);  // fix #65 - micros() is non-monotonic
  }
}



/*********************************************************************
 * @fn      SysTick_Handler
 *
 * @brief   This function handles systick interrupt.
 *
 * @return  none
 */
void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void __attribute__((weak)) SysTick_Handler(void)
{
  SysTick->CTLR=0;
  msTick+=TICK_FREQ_1KHz;
  SysTick->CNTL0=0;SysTick->CNTL1=0;SysTick->CNTL2=0;SysTick->CNTL3=0;
  SysTick->CNTH0=0;SysTick->CNTH1=0;SysTick->CNTH2=0;SysTick->CNTH3=0;
  SysTick->CTLR=0x1;
  osSystickHandler();
}

#endif




#ifdef __cplusplus
}
#endif


