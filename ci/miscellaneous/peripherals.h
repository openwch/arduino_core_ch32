#pragma once

// Load the board's pin definitions once, then override its peripheral defaults.
// This file is force-included only by the miscellaneous CI configurations.
#include "variant.h"

#undef ADC_MODULE_ENABLED
#undef DAC_MODULE_ENABLED
#undef UART_MODULE_ENABLED
#undef SPI_MODULE_ENABLED
#undef I2C_MODULE_ENABLED
#undef TIM_MODULE_ENABLED

#if defined(CI_ENABLE_ADC)
#define ADC_MODULE_ENABLED
#endif
#if defined(CI_ENABLE_DAC)
#define DAC_MODULE_ENABLED
#endif
#if defined(CI_ENABLE_SPI)
#define SPI_MODULE_ENABLED
#endif
#if defined(CI_ENABLE_UART)
#define UART_MODULE_ENABLED
#endif
#if defined(CI_ENABLE_I2C)
#define I2C_MODULE_ENABLED
#endif
#if defined(CI_ENABLE_TIM)
#define TIM_MODULE_ENABLED
#endif
