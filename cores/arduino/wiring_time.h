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

#ifndef CH32V10x
static inline void delayMicroseconds(uint32_t) __attribute__((always_inline, unused));
static inline void delayMicroseconds(uint32_t us)
{
  __IO uint64_t currentTicks = SysTick->CNT;
  /* Number of ticks per millisecond */
  uint64_t tickPerMs = SysTick->CMP + 1;
  /* Number of ticks to count */
  uint64_t nbTicks = ((us - ((us > 0) ? 1 : 0)) * tickPerMs) / 1000;
  /* Number of elapsed ticks */
  uint64_t elapsedTicks = 0;
  __IO uint64_t oldTicks = currentTicks;
  do {
    currentTicks = SysTick->CNT;
    // elapsedTicks += (oldTicks < currentTicks) ? tickPerMs + oldTicks - currentTicks :
    //                 oldTicks - currentTicks;
    
    //increment
    elapsedTicks += (oldTicks <= currentTicks) ? currentTicks - oldTicks :
                     tickPerMs - oldTicks + currentTicks;

    oldTicks = currentTicks;
  } while (nbTicks > elapsedTicks);  
}
#else
#define SYSTICK_CNTL    (0xE000F004)   
#define SYSTICK_CNTH    (0xE000F008)
#define SYSTICK_CMPL    (0xE000F00C)
#define SYSTICK_CMPH    (0xE000F010)

static inline void delayMicroseconds(uint32_t) __attribute__((always_inline, unused));
static inline void delayMicroseconds(uint32_t us)
{
  __IO uint64_t currentTicks = *((__IO uint32_t *)SYSTICK_CNTH);
                currentTicks = (currentTicks << 32) +  *((__IO uint32_t *)SYSTICK_CNTL); 
  /* Number of ticks per millisecond */
  uint64_t tickPerMs = *((__IO uint32_t *)SYSTICK_CMPH);
           tickPerMs = (tickPerMs << 32) + *((__IO uint32_t *)SYSTICK_CMPL) + 1;        
  /* Number of ticks to count */
  uint64_t nbTicks = ((us - ((us > 0) ? 1 : 0)) * tickPerMs) / 1000;
  /* Number of elapsed ticks */
  uint64_t elapsedTicks = 0;
  __IO uint64_t oldTicks = currentTicks;
  do {
    currentTicks = *((__IO uint32_t *)SYSTICK_CNTH);
    currentTicks = (currentTicks << 32) +  *((__IO uint32_t *)SYSTICK_CNTL); 
    //increment
    elapsedTicks += (oldTicks <= currentTicks) ? currentTicks - oldTicks :
                     tickPerMs - oldTicks + currentTicks;

    oldTicks = currentTicks;
  } while (nbTicks > elapsedTicks);  
}

#endif



#ifdef __cplusplus
}
#endif

#endif /* _WIRING_TIME_H_ */
