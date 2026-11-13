/*
 *******************************************************************************
 * Copyright (c) 2017, STMicroelectronics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
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
 *******************************************************************************
 Modified 22 July 2023 by TempersLee
 */

#ifndef _PINAF_CH32V4X7_H_
#define _PINAF_CH32V4X7_H_

#include <stdint.h>
#include "PinNames.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Remap selectors used in PeripheralPins.c (CH_PIN_DATA AFNUM field).
 * AFIO_NONE leaves the reset (default) mapping untouched.
 */
enum {
  AFIO_NONE,
  AFIO_Remap_I2C1_ENABLE,     /* I2C1 SCL/SDA -> PB8/PB9 */
  AFIO_Remap_I2C1_DISABLE,    /* I2C1 SCL/SDA -> PB6/PB7 */
  AFIO_Remap_SPI3_ENABLE,     /* SPI3 -> PA4/PC10/PC11/PC12 */
  AFIO_Remap_SPI3_DISABLE,    /* SPI3 -> PA15/PB3/PB4/PB5 */
};

static inline void pinV32_DisconnectDebug(PinName pin)
{
  (void)pin;  /* Keep the SDI programming pins (PA13/PA14) connected. */
}

static inline void pin_SetV32AFPin(uint32_t afnum)
{
  // Enable AFIO clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  switch (afnum) {
    case AFIO_Remap_I2C1_ENABLE:
      GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);
      break;
    case AFIO_Remap_I2C1_DISABLE:
      GPIO_PinRemapConfig(GPIO_Remap_I2C1, DISABLE);
      break;
    case AFIO_Remap_SPI3_ENABLE:
      GPIO_PinRemapConfig(GPIO_Remap_SPI3, ENABLE);
      break;
    case AFIO_Remap_SPI3_DISABLE:
      GPIO_PinRemapConfig(GPIO_Remap_SPI3, DISABLE);
      break;
    default:
    case AFIO_NONE:
      break;
  }
}

#ifdef __cplusplus
}
#endif

#endif