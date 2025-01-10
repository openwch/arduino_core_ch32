/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32l103_gpio.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/03/01
 * Description        : This file contains all the functions prototypes for the
 *                      GPIO firmware library.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#ifndef __CH32L103_GPIO_H
#define __CH32L103_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ch32l103.h"

/* Output Maximum frequency selection */
typedef enum
{
    GPIO_Speed_10MHz = 1,
    GPIO_Speed_2MHz,
    GPIO_Speed_50MHz
} GPIOSpeed_TypeDef;

/* Configuration Mode enumeration */
typedef enum
{
    GPIO_Mode_AIN = 0x0,
    GPIO_Mode_IN_FLOATING = 0x04,
    GPIO_Mode_IPD = 0x28,
    GPIO_Mode_IPU = 0x48,
    GPIO_Mode_Out_OD = 0x14,
    GPIO_Mode_Out_PP = 0x10,
    GPIO_Mode_AF_OD = 0x1C,
    GPIO_Mode_AF_PP = 0x18
} GPIOMode_TypeDef;

/* GPIO Init structure definition */
typedef struct
{
    uint16_t GPIO_Pin; /* Specifies the GPIO pins to be configured.
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
#define GPIO_Pin_0                      ((uint16_t)0x0001) /* Pin 0 selected */
#define GPIO_Pin_1                      ((uint16_t)0x0002) /* Pin 1 selected */
#define GPIO_Pin_2                      ((uint16_t)0x0004) /* Pin 2 selected */
#define GPIO_Pin_3                      ((uint16_t)0x0008) /* Pin 3 selected */
#define GPIO_Pin_4                      ((uint16_t)0x0010) /* Pin 4 selected */
#define GPIO_Pin_5                      ((uint16_t)0x0020) /* Pin 5 selected */
#define GPIO_Pin_6                      ((uint16_t)0x0040) /* Pin 6 selected */
#define GPIO_Pin_7                      ((uint16_t)0x0080) /* Pin 7 selected */
#define GPIO_Pin_8                      ((uint16_t)0x0100) /* Pin 8 selected */
#define GPIO_Pin_9                      ((uint16_t)0x0200) /* Pin 9 selected */
#define GPIO_Pin_10                     ((uint16_t)0x0400) /* Pin 10 selected */
#define GPIO_Pin_11                     ((uint16_t)0x0800) /* Pin 11 selected */
#define GPIO_Pin_12                     ((uint16_t)0x1000) /* Pin 12 selected */
#define GPIO_Pin_13                     ((uint16_t)0x2000) /* Pin 13 selected */
#define GPIO_Pin_14                     ((uint16_t)0x4000) /* Pin 14 selected */
#define GPIO_Pin_15                     ((uint16_t)0x8000) /* Pin 15 selected */
#define GPIO_Pin_All                    ((uint16_t)0xFFFF) /* All pins selected */

/* GPIO_Remap_define */
//bit[31:30] = 11b - PCFR1-bit[15-0] and PCFR2-bit[26:16]
/* bit[29:27] = 000b */
#define GPIO_PartialRemap1_SPI1         ((uint32_t)0xC0000001) /* SPI1 Partial1 Alternate Function mapping */
#define GPIO_PartialRemap2_SPI1         ((uint32_t)0xC1000000) /* SPI1 Partial2 Alternate Function mapping */
#define GPIO_FullRemap_SPI1             ((uint32_t)0xC1000001) /* SPI1 Full Alternate Function mapping */
/* bit[29:27] = 001b */
#define GPIO_PartialRemap1_I2C1         ((uint32_t)0xC8800000) /* I2C1 Partial1 Alternate Function mapping */
#define GPIO_FullRemap_I2C1             ((uint32_t)0xC8800002) /* I2C1 Full Alternate Function mapping */
/* bit[29:27] = 010b */
#define GPIO_PartialRemap1_USART1       ((uint32_t)0xD0000004) /* USART1 Partial1 Alternate Function mapping */
#define GPIO_PartialRemap2_USART1       ((uint32_t)0xD0080000) /* USART1 Partial2 Alternate Function mapping */
#define GPIO_PartialRemap3_USART1       ((uint32_t)0xD0080004) /* USART1 Partial3 Alternate Function mapping */
#define GPIO_PartialRemap4_USART1       ((uint32_t)0xD0100000) /* USART1 Partial4 Alternate Function mapping */
#define GPIO_FullRemap_USART1           ((uint32_t)0xD0100004) /* USART1 Full Alternate Function mapping */
/* bit[29:27] = 011b */
#define GPIO_PartialRemap1_USART2       ((uint32_t)0xD8000008) /* USART2 Partial1 Alternate Function mapping */
#define GPIO_PartialRemap2_USART2       ((uint32_t)0xD8040000) /* USART2 Partial2 Alternate Function mapping */
#define GPIO_FullRemap_USART2           ((uint32_t)0xD8040008) /* USART2 Full Alternate Function mapping */
/* bit[29:27] = 100b */
#define GPIO_PartialRemap1_TIM1         ((uint32_t)0xE0000040) /* TIM1 Partial1 Alternate Function mapping */
#define GPIO_PartialRemap2_TIM1         ((uint32_t)0xE0000080) /* TIM1 Partial2 Alternate Function mapping */
#define GPIO_PartialRemap3_TIM1         ((uint32_t)0xE00000C0) /* TIM1 Partial3 Alternate Function mapping */
#define GPIO_PartialRemap4_TIM1         ((uint32_t)0xE0400000) /* TIM1 Partial4 Alternate Function mapping */
#define GPIO_PartialRemap5_TIM1         ((uint32_t)0xE0400040) /* TIM1 Partial5 Alternate Function mapping */
#define GPIO_FullRemap_TIM1             ((uint32_t)0xE04000C0) /* TIM1 Full Alternate Function mapping */
/* bit[29:27] = 101b */
#define GPIO_PartialRemap1_TIM2         ((uint32_t)0xE8000100) /* TIM2 Partial1 Alternate Function mapping */
#define GPIO_PartialRemap2_TIM2         ((uint32_t)0xE8000200) /* TIM2 Partial2 Alternate Function mapping */
#define GPIO_PartialRemap3_TIM2         ((uint32_t)0xE8000300) /* TIM2 Partial3 Alternate Function mapping */
#define GPIO_PartialRemap4_TIM2         ((uint32_t)0xE8200000) /* TIM2 Partial4 Alternate Function mapping */
#define GPIO_PartialRemap5_TIM2         ((uint32_t)0xE8200100) /* TIM2 Partial5 Alternate Function mapping */
#define GPIO_FullRemap_TIM2             ((uint32_t)0xE8200300) /* TIM2 Full Alternate Function mapping */

//bit[31:30] = 00b - PCFR1
#define GPIO_PartialRemap_USART3        ((uint32_t)0x00140020) /* USART3 Partial Alternate Function mapping */
#define GPIO_FullRemap_USART3           ((uint32_t)0x00140030) /* USART3 Full Alternate Function mapping */
#define GPIO_Remap_TIM3                 ((uint32_t)0x00000400) /* TIM3 Alternate Function mapping */
#define GPIO_Remap_TIM4                 ((uint32_t)0x00001000) /* TIM4 Alternate Function mapping */
#define GPIO_Remap1_CAN1                ((uint32_t)0x001D4000) /* CAN1 Alternate Function mapping */
#define GPIO_Remap2_CAN1                ((uint32_t)0x001D6000) /* CAN1 Alternate Function mapping */
#define GPIO_Remap_PD01                 ((uint32_t)0x00008000) /* PD01 Alternate Function mapping */
#define GPIO_Remap_SWJ_Disable          ((uint32_t)0x00300400) /* GPIO_Remap_SWJ_Disable - Full SDI Disabled (SDI) */

//bit[31:30] = 01b - PCFR2
#define GPIO_Remap_USART4               ((uint32_t)0x40000001) /* USART4 Alternate Function mapping */
#define GPIO_Remap_LPTIM                ((uint32_t)0x40000200) /* LPTIM Alternate Function mapping */

/* GPIO_Port_Sources */
#define GPIO_PortSourceGPIOA            ((uint8_t)0x00)
#define GPIO_PortSourceGPIOB            ((uint8_t)0x01)
#define GPIO_PortSourceGPIOC            ((uint8_t)0x02)
#define GPIO_PortSourceGPIOD            ((uint8_t)0x03)

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


void     GPIO_DeInit(GPIO_TypeDef *GPIOx);
void     GPIO_AFIODeInit(void);
void     GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_InitStruct);
void     GPIO_StructInit(GPIO_InitTypeDef *GPIO_InitStruct);
uint8_t  GPIO_ReadInputDataBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadInputData(GPIO_TypeDef *GPIOx);
uint8_t  GPIO_ReadOutputDataBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_ReadOutputData(GPIO_TypeDef *GPIOx);
void     GPIO_SetBits(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void     GPIO_ResetBits(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void     GPIO_WriteBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, BitAction BitVal);
void     GPIO_Write(GPIO_TypeDef *GPIOx, uint16_t PortVal);
void     GPIO_PinLockConfig(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void     GPIO_EventOutputConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource);
void     GPIO_EventOutputCmd(FunctionalState NewState);
void     GPIO_PinRemapConfig(uint32_t GPIO_Remap, FunctionalState NewState);
void     GPIO_EXTILineConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource);
void     GPIO_IPD_Unused(void);

#ifdef __cplusplus
}
#endif

#endif
