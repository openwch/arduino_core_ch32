/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32x035_gpio.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/12/26
 * Description        : This file contains all the functions prototypes for the
 *                      GPIO firmware library.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32X035_GPIO_H
#define __CH32X035_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ch32x035.h"

/* Output Maximum frequency selection */
typedef enum
{
    GPIO_Speed_50MHz = 1,
} GPIOSpeed_TypeDef;

/* Configuration Mode enumeration */
typedef enum
{
    GPIO_Mode_AIN = 0x0,
    GPIO_Mode_IN_FLOATING = 0x04,
    GPIO_Mode_IPD = 0x28,   /* Only PA0--PA15 and PC16--PC17 support input pull-down */
    GPIO_Mode_IPU = 0x48,
    GPIO_Mode_Out_PP = 0x10,
    GPIO_Mode_AF_PP = 0x18
} GPIOMode_TypeDef;

/* GPIO Init structure definition */
typedef struct
{
    uint32_t GPIO_Pin; /* Specifies the GPIO pins to be configured.
                          This parameter can be any value of @ref GPIO_pins_define */

    GPIOSpeed_TypeDef GPIO_Speed; /* Specifies the speed for the selected pins.
                                     This parameter can be a value of @ref GPIOSpeed_TypeDef */

    GPIOMode_TypeDef GPIO_Mode; /* Specifies the operating mode for the selected pins.
                                   This parameter can be a value of @ref GPIOMode_TypeDef */
} GPIO_InitTypeDef;

/* Bit_SET and Bit_RESET enumeration */
typedef enum
{
    Bit_RESET = 0,
    Bit_SET
} BitAction;

/* GPIO_pins_define */
#define GPIO_Pin_0                      ((uint32_t)0x000001) /* Pin 0 selected */
#define GPIO_Pin_1                      ((uint32_t)0x000002) /* Pin 1 selected */
#define GPIO_Pin_2                      ((uint32_t)0x000004) /* Pin 2 selected */
#define GPIO_Pin_3                      ((uint32_t)0x000008) /* Pin 3 selected */
#define GPIO_Pin_4                      ((uint32_t)0x000010) /* Pin 4 selected */
#define GPIO_Pin_5                      ((uint32_t)0x000020) /* Pin 5 selected */
#define GPIO_Pin_6                      ((uint32_t)0x000040) /* Pin 6 selected */
#define GPIO_Pin_7                      ((uint32_t)0x000080) /* Pin 7 selected */
#define GPIO_Pin_8                      ((uint32_t)0x000100) /* Pin 8 selected */
#define GPIO_Pin_9                      ((uint32_t)0x000200) /* Pin 9 selected */
#define GPIO_Pin_10                     ((uint32_t)0x000400) /* Pin 10 selected */
#define GPIO_Pin_11                     ((uint32_t)0x000800) /* Pin 11 selected */
#define GPIO_Pin_12                     ((uint32_t)0x001000) /* Pin 12 selected */
#define GPIO_Pin_13                     ((uint32_t)0x002000) /* Pin 13 selected */
#define GPIO_Pin_14                     ((uint32_t)0x004000) /* Pin 14 selected */
#define GPIO_Pin_15                     ((uint32_t)0x008000) /* Pin 15 selected */
#define GPIO_Pin_16                     ((uint32_t)0x010000) /* Pin 16 selected */
#define GPIO_Pin_17                     ((uint32_t)0x020000) /* Pin 17 selected */
#define GPIO_Pin_18                     ((uint32_t)0x040000) /* Pin 18 selected */
#define GPIO_Pin_19                     ((uint32_t)0x080000) /* Pin 19 selected */
#define GPIO_Pin_20                     ((uint32_t)0x100000) /* Pin 20 selected */
#define GPIO_Pin_21                     ((uint32_t)0x200000) /* Pin 21 selected */
#define GPIO_Pin_22                     ((uint32_t)0x400000) /* Pin 22 selected */
#define GPIO_Pin_23                     ((uint32_t)0x800000) /* Pin 23 selected */
#define GPIO_Pin_All                    ((uint32_t)0xFFFFFF) /* All pins selected */

/* GPIO_Remap_define */
/* PCFR1 */
#define GPIO_PartialRemap1_SPI1         ((uint32_t)0x00100001) /* SPI1 Partial1 Alternate Function mapping */
#define GPIO_PartialRemap2_SPI1         ((uint32_t)0x00100002) /* SPI1 Partial2 Alternate Function mapping */
#define GPIO_FullRemap_SPI1             ((uint32_t)0x00100003) /* SPI1 Full Alternate Function mapping */
#define GPIO_PartialRemap1_I2C1         ((uint32_t)0x08020004) /* I2C1 Partial1 Alternate Function mapping */
#define GPIO_PartialRemap2_I2C1         ((uint32_t)0x08020008) /* I2C1 Partial2 Alternate Function mapping */
#define GPIO_PartialRemap3_I2C1         ((uint32_t)0x0802000C) /* I2C1 Partial3 Alternate Function mapping */
#define GPIO_PartialRemap4_I2C1         ((uint32_t)0x08020010) /* I2C1 Partial4 Alternate Function mapping */
#define GPIO_FullRemap_I2C1             ((uint32_t)0x08020014) /* I2C1 Full Alternate Function mapping */
#define GPIO_PartialRemap1_USART1       ((uint32_t)0x00150020) /* USART1 Partial1 Alternate Function mapping */
#define GPIO_PartialRemap2_USART1       ((uint32_t)0x00150040) /* USART1 Partial2 Alternate Function mapping */
#define GPIO_FullRemap_USART1           ((uint32_t)0x00150060) /* USART1 Full Alternate Function mapping */
#define GPIO_PartialRemap1_USART2       ((uint32_t)0x08070080) /* USART2 Partial1 Alternate Function mapping */
#define GPIO_PartialRemap2_USART2       ((uint32_t)0x08070100) /* USART2 Partial2 Alternate Function mapping */
#define GPIO_PartialRemap3_USART2       ((uint32_t)0x08070180) /* USART2 Partial3 Alternate Function mapping */
#define GPIO_FullRemap_USART2           ((uint32_t)0x08070200) /* USART2 Full Alternate Function mapping */
#define GPIO_PartialRemap1_USART3       ((uint32_t)0x001A0400) /* USART3 Partial1 Alternate Function mapping */
#define GPIO_PartialRemap2_USART3       ((uint32_t)0x001A0800) /* USART3 Partial2 Alternate Function mapping */
#define GPIO_FullRemap_USART3           ((uint32_t)0x001A0C00) /* USART3 Full Alternate Function mapping */
#define GPIO_PartialRemap1_USART4       ((uint32_t)0x080C1000) /* USART4 Partial1 Alternate Function mapping */
#define GPIO_PartialRemap2_USART4       ((uint32_t)0x080C2000) /* USART4 Partial2 Alternate Function mapping */
#define GPIO_PartialRemap3_USART4       ((uint32_t)0x080C3000) /* USART4 Partial3 Alternate Function mapping */
#define GPIO_PartialRemap4_USART4       ((uint32_t)0x080C4000) /* USART4 Partial4 Alternate Function mapping */
#define GPIO_FullRemap_USART4           ((uint32_t)0x080C7000) /* USART4 Full Alternate Function mapping */
#define GPIO_PartialRemap1_TIM1         ((uint32_t)0x084F0001) /* TIM1 Partial1 Alternate Function mapping */
#define GPIO_PartialRemap2_TIM1         ((uint32_t)0x084F0002) /* TIM1 Partial2 Alternate Function mapping */
#define GPIO_PartialRemap3_TIM1         ((uint32_t)0x084F0003) /* TIM1 Partial3 Alternate Function mapping */
#define GPIO_FullRemap_TIM1             ((uint32_t)0x084F0004) /* TIM1 Full Alternate Function mapping */
#define GPIO_PartialRemap1_TIM2         ((uint32_t)0x08220004) /* TIM2 Partial1 Alternate Function mapping */
#define GPIO_PartialRemap2_TIM2         ((uint32_t)0x08220008) /* TIM2 Partial2 Alternate Function mapping */
#define GPIO_PartialRemap3_TIM2         ((uint32_t)0x0822000C) /* TIM2 Partial3 Alternate Function mapping */
#define GPIO_PartialRemap4_TIM2         ((uint32_t)0x08220010) /* TIM2 Partial4 Alternate Function mapping */
#define GPIO_PartialRemap5_TIM2         ((uint32_t)0x08220014) /* TIM2 Partial5 Alternate Function mapping */
#define GPIO_FullRemap_TIM2             ((uint32_t)0x08220018) /* TIM2 Full Alternate Function mapping */
#define GPIO_PartialRemap1_TIM3         ((uint32_t)0x00350020) /* TIM3 Partial1 Alternate Function mapping */
#define GPIO_PartialRemap2_TIM3         ((uint32_t)0x00350040) /* TIM3 Partial2 Alternate Function mapping */
#define GPIO_FullRemap_TIM3             ((uint32_t)0x00350060) /* TIM3 Full Alternate Function mapping */
#define GPIO_Remap_PIOC                 ((uint32_t)0x00200080) /* PIOC Alternate Function mapping */
#define GPIO_Remap_SWJ_Disable          ((uint32_t)0x08300400) /* SDI Disabled (SDI) */

/* GPIO_Port_Sources */
#define GPIO_PortSourceGPIOA            ((uint8_t)0x00)
#define GPIO_PortSourceGPIOB            ((uint8_t)0x01)
#define GPIO_PortSourceGPIOC            ((uint8_t)0x02)

/* GPIO_Pin_sources */
#define GPIO_PinSource0                 ((uint8_t)0x00)
#define GPIO_PinSource1                 ((uint8_t)0x01)
#define GPIO_PinSource2                 ((uint8_t)0x02)
#define GPIO_PinSource3                 ((uint8_t)0x03)
#define GPIO_PinSource4                 ((uint8_t)0x04)
#define GPIO_PinSource5                 ((uint8_t)0x05)
#define GPIO_PinSource6                 ((uint8_t)0x06)
#define GPIO_PinSource7                 ((uint8_t)0x07)
#define GPIO_PinSource8                 ((uint8_t)0x08)
#define GPIO_PinSource9                 ((uint8_t)0x09)
#define GPIO_PinSource10                ((uint8_t)0x0A)
#define GPIO_PinSource11                ((uint8_t)0x0B)
#define GPIO_PinSource12                ((uint8_t)0x0C)
#define GPIO_PinSource13                ((uint8_t)0x0D)
#define GPIO_PinSource14                ((uint8_t)0x0E)
#define GPIO_PinSource15                ((uint8_t)0x0F)
#define GPIO_PinSource16                ((uint8_t)0x10)
#define GPIO_PinSource17                ((uint8_t)0x11)
#define GPIO_PinSource18                ((uint8_t)0x12)
#define GPIO_PinSource19                ((uint8_t)0x13)
#define GPIO_PinSource20                ((uint8_t)0x14)
#define GPIO_PinSource21                ((uint8_t)0x15)
#define GPIO_PinSource22                ((uint8_t)0x16)
#define GPIO_PinSource23                ((uint8_t)0x17)


void     GPIO_DeInit(GPIO_TypeDef *GPIOx);
void     GPIO_AFIODeInit(void);
void     GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_InitStruct);
void     GPIO_StructInit(GPIO_InitTypeDef *GPIO_InitStruct);
uint8_t  GPIO_ReadInputDataBit(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin);
uint32_t GPIO_ReadInputData(GPIO_TypeDef *GPIOx);
uint8_t  GPIO_ReadOutputDataBit(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin);
uint32_t GPIO_ReadOutputData(GPIO_TypeDef *GPIOx);
void     GPIO_SetBits(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin);
void     GPIO_ResetBits(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin);
void     GPIO_WriteBit(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin, BitAction BitVal);
void     GPIO_Write(GPIO_TypeDef *GPIOx, uint32_t PortVal);
void     GPIO_PinLockConfig(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin);
void     GPIO_PinRemapConfig(uint32_t GPIO_Remap, FunctionalState NewState);
void     GPIO_EXTILineConfig(uint8_t GPIO_PortSource, uint16_t GPIO_PinSource);
void     GPIO_IPD_Unused(void);


#ifdef __cplusplus
}
#endif

#endif
