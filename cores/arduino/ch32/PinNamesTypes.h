/**
 *******************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * This software component is licensed by WCH under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */
#ifndef _PINNAMESTYPES_H
#define _PINNAMESTYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  WCH PIN data as used in pin_function is coded on 32 bits as below
 *  [1:0]  MODE Input / Output / Output Speed
 *         00:input
 *         01:output,10MHz
 *         10:output,2MHz
 *         11:output,50MHz
 * 
 *  [3:2]  CNF under input
 *         00:Analog input
 *         01:float input
 *         10:input with PU/PD
 *         11:reserved
 *         CNF under output
 *         00:output PP
 *         01:output OD
 *         10:output AF_PP
 *         11:output AF_OD
 * 
 *  [5:4]  input PUPD config
 *         00:No Pull
 *         01:Pull-up
 *         10:Pull-Down
 *         11:reserved
 *  [7:6]  reserved
 * 
 *  [14:8] AF_NUM 0-0x7F
 * 
 *  [19:15] Channel(Analog/Timer) 0-31
 * 
 *  [31:20] Reserved      
 * 
 * 
 *   
 * */


#define CH_PIN_MODE_MASK 0x03
#define CH_PIN_MODE_SHIFT 0
#define CH_PIN_MODE_BITS (CH_PIN_MODE_MASK << CH_PIN_MODE_SHIFT)

#define CH_PIN_CNF_MASK 0x03
#define CH_PIN_CNF_SHIFT 2
#define CH_PIN_CNF_BITS (CH_PIN_CNF_MASK << CH_PIN_CNF_SHIFT)

#define CH_PIN_PUPD_MASK 0x03
#define CH_PIN_PUPD_SHIFT 4
#define CH_PIN_PUPD_BITS (CH_PIN_PUPD_MASK << CH_PIN_PUPD_SHIFT)

#define CH_PIN_AFNUM_MASK 0x7F
#define CH_PIN_AFNUM_SHIFT 8
#define CH_PIN_AFNUM_BITS (CH_PIN_AFNUM_MASK << CH_PIN_AFNUM_SHIFT)


#define CH_PIN_CHAN_MASK 0x1F
#define CH_PIN_CHAN_SHIFT 15
#define CH_PIN_CHANNEL_BIT (CH_PIN_CHAN_MASK << CH_PIN_CHAN_SHIFT)

/*
#define CH_PIN_FUNCTION_MASK 0x07
#define CH_PIN_FUNCTION_SHIFT 0
#define CH_PIN_FUNCTION_BITS (CH_PIN_FUNCTION_MASK << CH_PIN_FUNCTION_SHIFT)

#define CH_PIN_OD_MASK 0x01
#define CH_PIN_OD_SHIFT 3
#define CH_PIN_OD_BITS (CH_PIN_OD_MASK << CH_PIN_OD_SHIFT)


#define CH_PIN_SPEED_MASK 0x03
#define CH_PIN_SPEED_SHIFT 6
#define CH_PIN_SPEED_BITS (CH_PIN_SPEED_MASK << CH_PIN_SPEED_SHIFT)
*/
#define CH_PIN_INV_MASK 0x01
#define CH_PIN_INV_SHIFT 20
#define CH_PIN_INV_BIT (CH_PIN_INV_MASK << CH_PIN_INV_SHIFT)

#define CH_PIN_AN_CTRL_MASK 0x01
#define CH_PIN_AN_CTRL_SHIFT 21
#define CH_PIN_ANALOG_CONTROL_BIT (CH_PIN_AN_CTRL_MASK << CH_PIN_AN_CTRL_SHIFT)

/*
#define CH_PIN_AN_CHAN_BANK_B_MASK 0x01
#define CH_PIN_AN_CHAN_BANK_B_SHIFT 22
#define CH_PIN_ANALOG_CHAN_BANK_B_BIT (CH_PIN_AN_CHAN_BANK_B_MASK << CH_PIN_AN_CHAN_BANK_B_SHIFT)
*/

#define CH_PIN_MODE(x)                   (((x) >> CH_PIN_MODE_SHIFT) & CH_PIN_MODE_MASK)
#define CH_PIN_CNF(x)                    (((x) >> CH_PIN_CNF_SHIFT) & CH_PIN_CNF_MASK)
#define CH_PIN_PUPD(x)                   (((x) >> CH_PIN_PUPD_SHIFT) & CH_PIN_PUPD_MASK)
#define CH_PIN_AFNUM(x)                  (((x) >> CH_PIN_AFNUM_SHIFT) & CH_PIN_AFNUM_MASK) 
#define CH_PIN_CHANNEL(X)                (((X) >> CH_PIN_CHAN_SHIFT) & CH_PIN_CHAN_MASK)

/*
#define CH_PIN_FUNCTION(X)               (((X) >> CH_PIN_FUNCTION_SHIFT) & CH_PIN_FUNCTION_MASK)
#define CH_PIN_OD(X)                     (((X) >> CH_PIN_OD_SHIFT) & CH_PIN_OD_MASK)
#define CH_PIN_PUPD(X)                   (((X) >> CH_PIN_PUPD_SHIFT) & CH_PIN_PUPD_MASK)
#define CH_PIN_SPEED(X)                  (((X) >> CH_PIN_SPEED_SHIFT) & CH_PIN_SPEED_MASK)
#define CH_PIN_AFNUM(X)                  (((X) >> CH_PIN_AFNUM_SHIFT) & CH_PIN_AFNUM_MASK)
#define CH_PIN_CHANNEL(X)                (((X) >> CH_PIN_CHAN_SHIFT) & CH_PIN_CHAN_MASK)
*/
#define CH_PIN_INVERTED(X)               (((X) >> CH_PIN_INV_SHIFT) & CH_PIN_INV_MASK)
#define CH_PIN_ANALOG_CONTROL(X)         (((X) >> CH_PIN_AN_CTRL_SHIFT) & CH_PIN_AN_CTRL_MASK)
/*
#define CH_PIN_ANALOG_CHANNEL_BANK_B(X)  (((X) >> CH_PIN_AN_CHAN_BANK_B_SHIFT) & CH_PIN_AN_CHAN_BANK_B_MASK)
#define CH_PIN_MODE(X)                   ((CH_PIN_OD((X)) << 4) | \
                                           (CH_PIN_FUNCTION((X)) & (~CH_PIN_OD_BITS)))
*/


#define CH_PIN_DEFINE(MODE, CNF, PUPD, AFNUM) ((int)(MODE) |\
                             ((CNF & CH_PIN_CNF_MASK) << CH_PIN_CNF_SHIFT)       |\
                             ((PUPD & CH_PIN_PUPD_MASK) << CH_PIN_PUPD_SHIFT)    |\
                             ((AFNUM & CH_PIN_AFNUM_MASK) << CH_PIN_AFNUM_SHIFT))


#define CH_PIN_DEFINE_EXT(MODE, CNF, PUPD, AFNUM, CHAN)  ((int)(MODE) |\
                             ((CNF & CH_PIN_CNF_MASK) << CH_PIN_CNF_SHIFT)       |\
                             ((PUPD & CH_PIN_PUPD_MASK) << CH_PIN_PUPD_SHIFT)    |\
                             ((AFNUM & CH_PIN_AFNUM_MASK) << CH_PIN_AFNUM_SHIFT) |\
                             ((CHAN & CH_PIN_CHAN_MASK) << CH_PIN_CHAN_SHIFT))




/*
#define CH_PIN_DEFINE(FUNC_OD, PUPD, AFNUM)  ((int)(FUNC_OD) |\
                          ((PUPD  & CH_PIN_PUPD_MASK) << CH_PIN_PUPD_SHIFT) |\
                          ((AFNUM & CH_PIN_AFNUM_MASK) << CH_PIN_AFNUM_SHIFT))

#define CH_PIN_DEFINE_EXT(FUNC_OD, PUPD, AFNUM, CHAN, INV) \
                                            ((int)(FUNC_OD) |\
                       ((PUPD   & CH_PIN_PUPD_MASK) << CH_PIN_PUPD_SHIFT) |\
                       ((AFNUM  & CH_PIN_AFNUM_MASK) << CH_PIN_AFNUM_SHIFT) |\
                       ((CHAN   & CH_PIN_CHAN_MASK) << CH_PIN_CHAN_SHIFT) |\
                       ((INV    & CH_PIN_INV_MASK) << CH_PIN_INV_SHIFT))
*/


/*
 * MACROS to support the legacy definition of PIN formats
 * The CH_MODE_ defines contain the function and the Push-pull/OpenDrain
 * configuration (legacy inheritance).
 */
#define CH_PIN_DATA(MODE, CNF, PUPD, AFNUM) \
            CH_PIN_DEFINE(MODE, CNF, PUPD, AFNUM)
#define CH_PIN_DATA_EXT(MODE, CNF, PUPD, AFNUM, CHAN) \
            CH_PIN_DEFINE_EXT(MODE, CNF, PUPD, AFNUM, CHAN)


/*
typedef enum {
  CH_PIN_INPUT = 0,
  CH_PIN_OUTPUT = 1,
  CH_PIN_ALTERNATE = 2,
  CH_PIN_ANALOG = 3,
} CHPinFunction;

#define CH_MODE_INPUT                      (CH_PIN_INPUT)
#define CH_MODE_OUTPUT_PP                  (CH_PIN_OUTPUT)
#define CH_MODE_OUTPUT_OD                  (CH_PIN_OUTPUT | CH_PIN_OD_BITS)
#define CH_MODE_AF_PP                      (CH_PIN_ALTERNATE)
#define CH_MODE_AF_OD                      (CH_PIN_ALTERNATE | CH_PIN_OD_BITS)
#define CH_MODE_ANALOG                     (CH_PIN_ANALOG)
#define CH_MODE_ANALOG_ADC_CONTROL         (CH_PIN_ANALOG | CH_PIN_ANALOG_CONTROL_BIT)
#define CH_MODE_ANALOG_ADC_CHANNEL_BANK_B  (CH_PIN_ANALOG | CH_PIN_ANALOG_CHAN_BANK_B_BIT)
*/

/* MODE */
#define CH_MODE_INPUT        (0)
#define CH_MODE_OUTPUT_10MHz (1)
#define CH_MODE_OUTPUT_2MHz  (2)
#define CH_MODE_OUTPUT_50MHz (3)
/* CNF */
#define CH_CNF_INPUT_ANALOG  (0)
#define CH_CNF_INPUT_FLOAT   (1)
#define CH_CNF_INPUT_PUPD    (2)
#define CH_CNF_OUTPUT_PP     (0)
#define CH_CNF_OUTPUT_OD     (1)
#define CH_CNF_OUTPUT_AFPP   (2)
#define CH_CNF_OUTPUT_AFOD   (3)

/* PUPD */
#define NOPULL               (0)
#define PULLUP               (1)
#define PILLDOWN             (2)


#if !defined(CH32X035)
// High nibble = port number (FirstPort <= PortName <= LastPort)
// Low nibble  = pin number
#define CH_PORT(X) (((uint32_t)(X) >> 4) & 0xF)
#define CH_PIN(X)  ((uint32_t)(X) & 0xF)

// Check PinName is valid: FirstPort <= PortName <= LastPort
// As FirstPort is equal to 0 and CH_PORT cast as an unsigned
// (CH_PORT(X) >= FirstPort)  is always true
//#define CH_VALID_PINNAME(X) ((CH_PORT(X) >= FirstPort) && (CH_PORT(X) <= LastPort))

#define CH_VALID_PINNAME(X) (CH_PORT(X) <= LastPort)
#define CH_GPIO_PIN(X) ((uint16_t)(1<<CH_PIN(X)))

#else

// High nibble = port number (FirstPort <= PortName <= LastPort)
// Low nibble  = pin number
#define CH_PORT(X) (((uint32_t)(X) >> 5) & 0x7)
#define CH_PIN(X)  ((uint32_t)(X) & 0x1F)

// Check PinName is valid: FirstPort <= PortName <= LastPort
// As FirstPort is equal to 0 and CH_PORT cast as an unsigned
// (CH_PORT(X) >= FirstPort)  is always true
//#define CH_VALID_PINNAME(X) ((CH_PORT(X) >= FirstPort) && (CH_PORT(X) <= LastPort))
#define CH_VALID_PINNAME(X) (CH_PORT(X) <= LastPort)
#define CH_GPIO_PIN(X) ((uint32_t)(1<<CH_PIN(X)))

#endif

#ifdef __cplusplus
}
#endif

#endif

