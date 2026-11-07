/*
  Copyright (c) 2011 Arduino.  All right reserved.
  Copyright (c) 2013 by Paul Stoffregen <paul@pjrc.com> (delayMicroseconds)

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

#ifndef _WIRING_TIME_H_
#define _WIRING_TIME_H_

#include "clock.h"


#ifdef __cplusplus
extern "C" {
#endif
/**
 * \brief Returns the number of milliseconds since the Arduino board began running the current program.
 *
 * This number will overflow (go back to zero), after approximately 50 days.
 *
 * \return Number of milliseconds since the program started (uint32_t)
 */
extern uint32_t millis(void) ;

/**
 * \brief Returns the number of microseconds since the Arduino board began running the current program.
 *
 * This number will overflow (go back to zero), after approximately 70 minutes. On 16 MHz Arduino boards
 * (e.g. Duemilanove and Nano), this function has a resolution of four microseconds (i.e. the value returned is
 * always a multiple of four). On 8 MHz Arduino boards (e.g. the LilyPad), this function has a resolution
 * of eight microseconds.
 *
 * \note There are 1,000 microseconds in a millisecond and 1,000,000 microseconds in a second.
 */
extern uint32_t micros(void) ;

/**
 * \brief Pauses the program for the amount of time (in milliseconds) specified as parameter.
 * (There are 1000 milliseconds in a second.)
 *
 * \param ms the number of milliseconds to pause (uint32_t)
 */
extern void delay(uint32_t ms) ;

/**
 * \brief Pauses the program for the amount of time (in microseconds) specified as parameter.
 *
 * \param us the number of microseconds to pause (uint32_t)
 */

/*
 * SysTick restarts every millisecond (CMP = ticks per ms - 1), so every count
 * fits in 32 bits. Only 32-bit math here: the 64-bit multiply and divide
 * this used to do are library calls on chips without a hardware divider and
 * took 35-55 us on the V003/V006 before the wait even started.
 * tickPerMs / 1000 is done as (tickPerMs * 1049) >> 20 (exact for whole-MHz
 * clocks, no division).
 */
#ifndef CH32V10x
static inline void delayMicroseconds(uint32_t) __attribute__((always_inline, unused));
static inline void delayMicroseconds(uint32_t us)
{
  /* low word of CNT/CMP (64-bit registers on V20x/V30x/X035/L10x) */
  __IO uint32_t *cnt = (__IO uint32_t *)&SysTick->CNT;
  uint32_t oldTicks = *cnt;
  uint32_t tickPerMs = *(__IO uint32_t *)&SysTick->CMP + 1;
#if defined(F_CPU) && (F_CPU % 1000000 == 0)
  /* SysTick runs at HCLK = F_CPU: a constant, so no multiply call on rv32ec */
  uint32_t nbTicks = (us - ((us > 0) ? 1 : 0)) * (F_CPU / 1000000);
#else
  uint32_t nbTicks = (us - ((us > 0) ? 1 : 0)) * ((tickPerMs * 1049) >> 20);
#endif
  uint32_t elapsedTicks = 0;
  while (elapsedTicks < nbTicks) {
    uint32_t currentTicks = *cnt;
    elapsedTicks += (oldTicks <= currentTicks) ? currentTicks - oldTicks :
                     tickPerMs - oldTicks + currentTicks;
    oldTicks = currentTicks;
  }
}
#else
#define SYSTICK_CNTL    (0xE000F004)   
#define SYSTICK_CNTH    (0xE000F008)
#define SYSTICK_CMPL    (0xE000F00C)
#define SYSTICK_CMPH    (0xE000F010)

static inline void delayMicroseconds(uint32_t) __attribute__((always_inline, unused));
static inline void delayMicroseconds(uint32_t us)
{
  /* V10x: SysTick at HCLK/8, restarts every ms; the low words are enough */
  uint32_t oldTicks = *((__IO uint32_t *)SYSTICK_CNTL);
  uint32_t tickPerMs = *((__IO uint32_t *)SYSTICK_CMPL) + 1;
  uint32_t nbTicks = (us - ((us > 0) ? 1 : 0)) * ((tickPerMs * 1049) >> 20);
  uint32_t elapsedTicks = 0;
  while (elapsedTicks < nbTicks) {
    uint32_t currentTicks = *((__IO uint32_t *)SYSTICK_CNTL);
    elapsedTicks += (oldTicks <= currentTicks) ? currentTicks - oldTicks :
                     tickPerMs - oldTicks + currentTicks;
    oldTicks = currentTicks;
  }
}

#endif



#ifdef __cplusplus
}
#endif

#endif /* _WIRING_TIME_H_ */
