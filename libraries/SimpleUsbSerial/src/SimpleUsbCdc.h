#ifndef __CH573_USB_CDC_H__
#define __CH573_USB_CDC_H__

// clang-format off
#include <stdint.h>
#if defined (CH573)
#include "CH573SFR.h"
#include "core_riscv.h"
#elif defined (CH572)
#include "CH572SFR.h"
#include "core_riscv.h"
#elif defined (CH585)
#include "CH585SFR.h"
#include "core_riscv.h"
#elif defined (CH32X035)
#include "ch32x035_rcc.h"
#elif defined (CH32V30x)
#include "ch32v30x_rcc.h"
// HSE might be needed for high speed USB
#if !defined (SYSCLK_FREQ_144MHz_HSE)
#error "High speed USB requires HSE. Please use 144M External as Clock Source"
#endif
#else
#error "Unsupported platform"
#endif
// clang-format on

#ifdef __cplusplus
extern "C" {
#endif

void USBInit(void);

#ifdef __cplusplus
} // extern "C"
#endif

#endif