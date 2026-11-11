#ifdef CH32V00x
#include "ch32v00x_gpio.c"
#endif

#ifdef CH32VM00X
#include "ch32v00X_gpio.c"
#endif

#ifdef CH32X035
#include "ch32x035_gpio.c"
#endif

#ifdef CH32V10x
#include "ch32v10x_gpio.c"
#endif

#ifdef CH32V20x
#include "ch32v20x_gpio.c"
#endif

#if defined(CH32V30x) || defined(CH32V30x_C)
#include "ch32v30x_gpio.c"
#endif

#if defined(CH32L10x)
#include "ch32l103_gpio.c"
#endif

#if defined(CH573)
#include "CH573_gpio.c"
#endif

#if defined(CH572)
#include "CH572_gpio.c"
#endif

#if defined(CH585)
#include "CH585_gpio.c"
#endif
