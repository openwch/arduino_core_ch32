#ifndef _CH32YYXX_DEBUG_H_
#define _CH32YYXX_DEBUG_H_

#ifndef DEBUG
#define CH32_SDK_DEBUG_DEFAULT      /* DEBUG is not set by the user: debug.h sets its own */
#endif
#include "debug.h"
#ifdef CH32_SDK_DEBUG_DEFAULT
#undef DEBUG                        /* #125: keep the SDK's default out of sketches and libraries */
#undef CH32_SDK_DEBUG_DEFAULT
#endif


#endif /*   _CH32YYXX_ADC_H_ */