#ifdef CH32V00x
#include "system_ch32v00x.c"    
#endif

#ifdef CH32VM00X
#include "system_ch32v00X.c"
#endif

#ifdef CH32X035
#include "system_ch32x035.c"    
#endif

#ifdef CH32V10x
#include "system_ch32v10x.c"
#endif

#ifdef CH32V20x
#include "system_ch32v20x.c"    
#endif

#if defined(CH32V30x) || defined(CH32V30x_C)
#include "system_ch32v30x.c"
#endif

#if defined(CH32L10x)
#include "system_ch32l103.c"
#endif

#if defined(CH573)
#include "CH573_sys.c"
#include "CH573.c"
#endif

#if defined(CH572)
#include "CH572_sys.c"
#include "CH572.c"
#endif

#if defined(CH585)
#include "CH585_sys.c"
#include "CH585.c"
#endif
