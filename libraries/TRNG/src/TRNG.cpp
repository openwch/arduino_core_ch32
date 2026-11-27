#include "TRNG.h"

TRNGClass TRNG;

static void rngClock(FunctionalState state)
{
#if defined(CH32V4x7)
  RCC_HBPeriphClockCmd(RCC_HBPeriph_RNG, state);
#else
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_RNG, state);
#endif
}

void TRNGClass::begin()
{
  rngClock(ENABLE);
  RNG_Cmd(ENABLE);
}

void TRNGClass::end()
{
  RNG_Cmd(DISABLE);
  rngClock(DISABLE);
}

uint32_t TRNGClass::read()
{
  uint32_t t0 = millis();
  while (RNG_GetFlagStatus(RNG_FLAG_DRDY) == RESET) {
    if (RNG_GetFlagStatus(RNG_FLAG_CECS | RNG_FLAG_SECS) != RESET || millis() - t0 > 10) {
      return 0;
    }
  }
  return RNG_GetRandomNumber();
}

long TRNGClass::random(long howbig)
{
  if (howbig <= 0) {
    return 0;
  }
  return read() % (uint32_t)howbig;
}

long TRNGClass::random(long howsmall, long howbig)
{
  if (howsmall >= howbig) {
    return howsmall;
  }
  return random(howbig - howsmall) + howsmall;
}
