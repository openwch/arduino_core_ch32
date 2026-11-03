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

extern "C" {
#include "stdlib.h"
#include "stdint.h"
}
#include "WMath.h"

// Same generator and sequence as newlib's rand()/srand(). newlib's own rand()
// (GCC 15) allocates its state with malloc and pulls in assert/fprintf, about
// 3 KB of flash, which no longer fits next to a sketch on the 16 KB V003.
static uint64_t _rand_next = 1;

static long _rand(void)
{
  _rand_next = _rand_next * 6364136223846793005ULL + 1;
  return (long)((_rand_next >> 32) & RAND_MAX);
}

extern void randomSeed(uint32_t dwSeed)
{
  if (dwSeed != 0) {
    _rand_next = dwSeed;
  }
}

extern long random(long howbig)
{
  if (howbig == 0) {
    return 0 ;
  }

  return _rand() % howbig;
}

extern long random(long howsmall, long howbig)
{
  if (howsmall >= howbig) {
    return howsmall;
  }

  long diff = howbig - howsmall;

  return random(diff) + howsmall;
}

extern long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

extern uint16_t makeWord(uint16_t w)
{
  return w ;
}

extern uint16_t makeWord(uint8_t h, uint8_t l)
{
  return (h << 8) | l ;
}
