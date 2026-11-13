#ifdef CH32V10x
#include "ch32v10x_crc.c"
#endif

#ifdef CH32V20x
#include "ch32v20x_crc.c"
#endif

#if defined(CH32V30x) || defined(CH32V30x_C)
#include "ch32v30x_crc.c"
#endif

#if defined(CH32L10x)
#include "ch32l103_crc.c"
#endif
#if defined(CH32V4x7)
#include "ch32v4x7_crc.c"
#endif
