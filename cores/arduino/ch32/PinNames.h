#ifndef _PINNAMES_H
#define _PINNAMES_H

#include "PinNamesTypes.h"
#include "PortNames.h"

#ifdef __cplusplus
extern "C" {
#endif

// Alternative possibilities which use other HW peripheral instances
#define ALT0                        0x000
#define ALT1                        0x100
#define ALT2                        0x200
#define ALT3                        0x300
#define ALT4                        0x400
#define ALT5                        0x500
#define ALT6                        0x600
#define ALT7                        0x700
// ALTX mask
#define ALTX_MASK                   0x700

// Specific pinmap definition
// Analog internal
#define PNAME_ANALOG_INTERNAL_BASE  0x1000
// Dual pad pin
// Direct channels are connected to analog I/Os
// (PY_C) to optimize ADC performance.
#define PDUAL                       0x2000
// Remap pin
#define PREMAP                      0x3000
// PinName mask
#define PNAME_MASK                  0xFF

typedef enum {
  // Not connected
  NC = 0xFFFFFFFF,

  // Pin name definition
  PA_0  = (PortA << 4) + 0x00,
  PA_1  = (PortA << 4) + 0x01,
  PA_2  = (PortA << 4) + 0x02,
  PA_3  = (PortA << 4) + 0x03,
  PA_4  = (PortA << 4) + 0x04,
  PA_5  = (PortA << 4) + 0x05,
  PA_6  = (PortA << 4) + 0x06,
  PA_7  = (PortA << 4) + 0x07,
  PA_8  = (PortA << 4) + 0x08,
  PA_9  = (PortA << 4) + 0x09,
  PA_10 = (PortA << 4) + 0x0A,
  PA_11 = (PortA << 4) + 0x0B,
  PA_12 = (PortA << 4) + 0x0C,
  PA_13 = (PortA << 4) + 0x0D,
  PA_14 = (PortA << 4) + 0x0E,
  PA_15 = (PortA << 4) + 0x0F,
#if defined GPIOB_BASE
  PB_0  = (PortB << 4) + 0x00,
  PB_1  = (PortB << 4) + 0x01,
  PB_2  = (PortB << 4) + 0x02,
  PB_3  = (PortB << 4) + 0x03,
  PB_4  = (PortB << 4) + 0x04,
  PB_5  = (PortB << 4) + 0x05,
  PB_6  = (PortB << 4) + 0x06,
  PB_7  = (PortB << 4) + 0x07,
  PB_8  = (PortB << 4) + 0x08,
  PB_9  = (PortB << 4) + 0x09,
  PB_10 = (PortB << 4) + 0x0A,
  PB_11 = (PortB << 4) + 0x0B,
  PB_12 = (PortB << 4) + 0x0C,
  PB_13 = (PortB << 4) + 0x0D,
  PB_14 = (PortB << 4) + 0x0E,
  PB_15 = (PortB << 4) + 0x0F,
#endif  
#if defined GPIOC_BASE
  PC_0  = (PortC << 4) + 0x00,
  PC_1  = (PortC << 4) + 0x01,
  PC_2  = (PortC << 4) + 0x02,
  PC_3  = (PortC << 4) + 0x03,
  PC_4  = (PortC << 4) + 0x04,
  PC_5  = (PortC << 4) + 0x05,
  PC_6  = (PortC << 4) + 0x06,
  PC_7  = (PortC << 4) + 0x07,
  PC_8  = (PortC << 4) + 0x08,
  PC_9  = (PortC << 4) + 0x09,
  PC_10 = (PortC << 4) + 0x0A,
  PC_11 = (PortC << 4) + 0x0B,
  PC_12 = (PortC << 4) + 0x0C,
  PC_13 = (PortC << 4) + 0x0D,
  PC_14 = (PortC << 4) + 0x0E,
  PC_15 = (PortC << 4) + 0x0F,
#endif
#if defined GPIOD_BASE
  PD_0  = (PortD << 4) + 0x00,
  PD_1  = (PortD << 4) + 0x01,
  PD_2  = (PortD << 4) + 0x02,
  PD_3  = (PortD << 4) + 0x03,
  PD_4  = (PortD << 4) + 0x04,
  PD_5  = (PortD << 4) + 0x05,
  PD_6  = (PortD << 4) + 0x06,
  PD_7  = (PortD << 4) + 0x07,
  PD_8  = (PortD << 4) + 0x08,
  PD_9  = (PortD << 4) + 0x09,
  PD_10 = (PortD << 4) + 0x0A,
  PD_11 = (PortD << 4) + 0x0B,
  PD_12 = (PortD << 4) + 0x0C,
  PD_13 = (PortD << 4) + 0x0D,
  PD_14 = (PortD << 4) + 0x0E,
  PD_15 = (PortD << 4) + 0x0F,
#endif
#if defined GPIOE_BASE
  PE_0  = (PortE << 4) + 0x00,
  PE_1  = (PortE << 4) + 0x01,
  PE_2  = (PortE << 4) + 0x02,
  PE_3  = (PortE << 4) + 0x03,
  PE_4  = (PortE << 4) + 0x04,
  PE_5  = (PortE << 4) + 0x05,
  PE_6  = (PortE << 4) + 0x06,
  PE_7  = (PortE << 4) + 0x07,
  PE_8  = (PortE << 4) + 0x08,
  PE_9  = (PortE << 4) + 0x09,
  PE_10 = (PortE << 4) + 0x0A,
  PE_11 = (PortE << 4) + 0x0B,
  PE_12 = (PortE << 4) + 0x0C,
  PE_13 = (PortE << 4) + 0x0D,
  PE_14 = (PortE << 4) + 0x0E,
  PE_15 = (PortE << 4) + 0x0F,
#endif
#if defined GPIOF_BASE
  PF_0  = (PortF << 4) + 0x00,
  PF_1  = (PortF << 4) + 0x01,
  PF_2  = (PortF << 4) + 0x02,
  PF_3  = (PortF << 4) + 0x03,
  PF_4  = (PortF << 4) + 0x04,
  PF_5  = (PortF << 4) + 0x05,
  PF_6  = (PortF << 4) + 0x06,
  PF_7  = (PortF << 4) + 0x07,
  PF_8  = (PortF << 4) + 0x08,
  PF_9  = (PortF << 4) + 0x09,
  PF_10 = (PortF << 4) + 0x0A,
  PF_11 = (PortF << 4) + 0x0B,
  PF_12 = (PortF << 4) + 0x0C,
  PF_13 = (PortF << 4) + 0x0D,
  PF_14 = (PortF << 4) + 0x0E,
  PF_15 = (PortF << 4) + 0x0F,
#endif
#if defined GPIOG_BASE
  PG_0  = (PortG << 4) + 0x00,
  PG_1  = (PortG << 4) + 0x01,
  PG_2  = (PortG << 4) + 0x02,
  PG_3  = (PortG << 4) + 0x03,
  PG_4  = (PortG << 4) + 0x04,
  PG_5  = (PortG << 4) + 0x05,
  PG_6  = (PortG << 4) + 0x06,
  PG_7  = (PortG << 4) + 0x07,
  PG_8  = (PortG << 4) + 0x08,
  PG_9  = (PortG << 4) + 0x09,
  PG_10 = (PortG << 4) + 0x0A,
  PG_11 = (PortG << 4) + 0x0B,
  PG_12 = (PortG << 4) + 0x0C,
  PG_13 = (PortG << 4) + 0x0D,
  PG_14 = (PortG << 4) + 0x0E,
  PG_15 = (PortG << 4) + 0x0F,
#endif
#if defined GPIOH_BASE
  PH_0  = (PortH << 4) + 0x00,
  PH_1  = (PortH << 4) + 0x01,
  PH_2  = (PortH << 4) + 0x02,
  PH_3  = (PortH << 4) + 0x03,
  PH_4  = (PortH << 4) + 0x04,
  PH_5  = (PortH << 4) + 0x05,
  PH_6  = (PortH << 4) + 0x06,
  PH_7  = (PortH << 4) + 0x07,
  PH_8  = (PortH << 4) + 0x08,
  PH_9  = (PortH << 4) + 0x09,
  PH_10 = (PortH << 4) + 0x0A,
  PH_11 = (PortH << 4) + 0x0B,
  PH_12 = (PortH << 4) + 0x0C,
  PH_13 = (PortH << 4) + 0x0D,
  PH_14 = (PortH << 4) + 0x0E,
  PH_15 = (PortH << 4) + 0x0F,
#endif

  // Specific pin name
  PADC_BASE = PNAME_ANALOG_INTERNAL_BASE,
#if defined(ADC_Channel_TempSensor) 
  PADC_TEMP,
#endif
#ifdef ADC_Channel_Vrefint
  PADC_VREF,
#endif
#ifdef ADC_Channel_Vbat
  PADC_VBAT,
#endif
  ANA_START,
#ifdef SYSCFG_PMCSETR_ANA0_SEL_Pos
  ANA_0,
#endif
#ifdef SYSCFG_PMCSETR_ANA1_SEL_Pos
  ANA_1,
#endif
  // Specific pin name define in the variant
#if __has_include("PinNamesVar.h")
#include "PinNamesVar.h"
#endif
  P_END = NC
} PinName;

#ifdef __cplusplus
}
#endif

#endif
