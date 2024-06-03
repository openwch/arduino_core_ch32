#ifdef CH32V00x
#include "ch32v00x_adc.c"

#elif defined(CH32VM00X) 
#include "ch32v00X_adc.c"

#elif defined(CH32X035)
#include "ch32x035_adc.c"
#include "ch32x035_awu.c"

#elif defined(CH32V10x)
#include "ch32v10x_adc.c"

#elif defined(CH32V20x)
#include "ch32v20x_adc.c"

#elif defined(CH32V30x ) || defined (CH32V30x_C)
#include "ch32v30x_adc.c"

#elif defined(CH32L10x)
#include "ch32l103_adc.c"
#endif


