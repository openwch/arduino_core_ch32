// The hardware random number generator of the CH32V30x and CH32V4x7.
#pragma once

#include <Arduino.h>

#if !defined(CH32V30x) && !defined(CH32V4x7)
#error "TRNG needs a chip with a hardware RNG (CH32V30x or CH32V4x7)"
#endif

class TRNGClass {
public:
  void begin();
  void end();
  // 32 random bits; 0 if the RNG reports a clock or seed error (then call begin() again)
  uint32_t read();
  // like Arduino's random(): [0, howbig) and [howsmall, howbig)
  long random(long howbig);
  long random(long howsmall, long howbig);
};

extern TRNGClass TRNG;
