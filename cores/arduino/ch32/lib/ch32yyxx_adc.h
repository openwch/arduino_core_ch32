#ifndef _CH32YYXX_ADC_H_
#define _CH32YYXX_ADC_H_

#ifdef CH32V00x
#include "ch32v00x_adc.h"

#elif defined(CH32VM00X)
#include "ch32v00X_adc.h"

#elif defined(CH32X035)
#include "ch32x035_adc.h"
#include "ch32x035_awu.h"

#elif defined(CH32V10x)
#include "ch32v10x_adc.h"

#elif defined(CH32V20x)
#include "ch32v20x_adc.h"

#elif defined(CH32V30x)  || defined(CH32V30x_C)
#include "ch32v30x_adc.h"

#elif defined(CH32L10x)
#include "ch32l103_adc.h"

#endif
#if defined(CH32V4x7)
#include "ch32v4x7_adc.h"
#endif

#endif /*   _CH32YYXX_ADC_H_ */