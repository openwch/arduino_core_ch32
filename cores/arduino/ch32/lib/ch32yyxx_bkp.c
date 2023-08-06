#ifdef CH32V00x  
//nothing
#elif defined(CH32V10x)
#include "ch32v10x_bkp.c"

#elif defined(CH32V20x)
#include "ch32v20x_bkp.c"

#elif defined(CH32V30x) || defined(CH32V30x_C)
#include "ch32v30x_bkp.c"
#endif