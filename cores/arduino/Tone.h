/*
  Copyright (c) 2011 Arduino.  All right reserved.

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

#ifndef _WIRING_TONE_
#define _WIRING_TONE_

#ifdef __cplusplus
  #include "variant.h"

  // Diagnose use of the no-op fallback at the call site, not in every core build.
  #if !defined(TIM_MODULE_ENABLED) || !defined(TIMER_TONE) || defined(TIM_MODULE_ONLY)
    #define CH32_TONE_UNAVAILABLE __attribute__((deprecated("Tone requires TIM_MODULE_ENABLED and TIMER_TONE, without TIM_MODULE_ONLY")))
  #else
    #define CH32_TONE_UNAVAILABLE
  #endif
  /*
  * \brief Generate a tone to a pin.
  *
  * \param _pin
  * \param frequency Tone frequency (in hertz)
  * \param duration  Tone duration (in milliseconds)
  */
  CH32_TONE_UNAVAILABLE
  extern void tone(uint8_t _pin, unsigned int frequency, unsigned long duration = 0);

  /*
  * \brief Stop tone generation on pin.
  *
  * \param _pin
  */
  CH32_TONE_UNAVAILABLE
  extern void noTone(uint8_t _pin, bool destruct = false);

  #undef CH32_TONE_UNAVAILABLE

#endif

#endif /* _WIRING_TONE_ */
