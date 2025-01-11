/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32x035_rcc.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/04/06
 * Description        : This file provides all the RCC firmware functions.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32X035_RCC_H
#define __CH32X035_RCC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ch32x035.h"

/* RCC_Exported_Types */
typedef struct
{
    uint32_t SYSCLK_Frequency; /* returns SYSCLK clock frequency expressed in Hz */
    uint32_t HCLK_Frequency;   /* returns HCLK clock frequency expressed in Hz */
    uint32_t PCLK1_Frequency;  /* returns PCLK1 clock frequency expressed in Hz */
    uint32_t PCLK2_Frequency;  /* returns PCLK2 clock frequency expressed in Hz */
} RCC_ClocksTypeDef;

/* AHB_clock_source */
#define RCC_SYSCLK_Div1                  ((uint32_t)0x00000000)
#define RCC_SYSCLK_Div2                  ((uint32_t)0x00000010)
#define RCC_SYSCLK_Div3                  ((uint32_t)0x00000020)
#define RCC_SYSCLK_Div4                  ((uint32_t)0x00000030)
#define RCC_SYSCLK_Div5                  ((uint32_t)0x00000040)
#define RCC_SYSCLK_Div6                  ((uint32_t)0x00000050)
#define RCC_SYSCLK_Div7                  ((uint32_t)0x00000060)
#define RCC_SYSCLK_Div8                  ((uint32_t)0x00000070)
#define RCC_SYSCLK_Div16                 ((uint32_t)0x000000B0)
#define RCC_SYSCLK_Div32                 ((uint32_t)0x000000C0)
#define RCC_SYSCLK_Div64                 ((uint32_t)0x000000D0)
#define RCC_SYSCLK_Div128                ((uint32_t)0x000000E0)
#define RCC_SYSCLK_Div256                ((uint32_t)0x000000F0)

/* AHB_peripheral */
#define RCC_AHBPeriph_DMA1             ((uint32_t)0x00000001)
#define RCC_AHBPeriph_SRAM             ((uint32_t)0x00000004)
#define RCC_AHBPeriph_USBFS            ((uint32_t)0x00001000)
#define RCC_AHBPeriph_IO2W             ((uint32_t)0x00002000)
#define RCC_AHBPeriph_USBPD            ((uint32_t)0x00020000)

/* APB2_peripheral */
#define RCC_APB2Periph_AFIO            ((uint32_t)0x00000001)
#define RCC_APB2Periph_GPIOA           ((uint32_t)0x00000004)
#define RCC_APB2Periph_GPIOB           ((uint32_t)0x00000008)
#define RCC_APB2Periph_GPIOC           ((uint32_t)0x00000010)
#define RCC_APB2Periph_ADC1            ((uint32_t)0x00000200)
#define RCC_APB2Periph_TIM1            ((uint32_t)0x00000800)
#define RCC_APB2Periph_SPI1            ((uint32_t)0x00001000)
#define RCC_APB2Periph_USART1          ((uint32_t)0x00004000)

/* APB1_peripheral */
#define RCC_APB1Periph_TIM2            ((uint32_t)0x00000001)
#define RCC_APB1Periph_TIM3            ((uint32_t)0x00000002)
#define RCC_APB1Periph_WWDG            ((uint32_t)0x00000800)
#define RCC_APB1Periph_USART2          ((uint32_t)0x00020000)
#define RCC_APB1Periph_USART3          ((uint32_t)0x00040000)
#define RCC_APB1Periph_USART4          ((uint32_t)0x00080000)
#define RCC_APB1Periph_I2C1            ((uint32_t)0x00200000)
#define RCC_APB1Periph_PWR             ((uint32_t)0x10000000)

/* Clock_source_to_output_on_MCO_pin */
#define RCC_MCO_NoClock                ((uint8_t)0x00)
#define RCC_MCO_SYSCLK                 ((uint8_t)0x04)
#define RCC_MCO_HSI                    ((uint8_t)0x05)

/* RCC_Flag */
#define RCC_FLAG_HSIRDY                ((uint8_t)0x21)
#define RCC_FLAG_OPARST                ((uint8_t)0x79)
#define RCC_FLAG_PINRST                ((uint8_t)0x7A)
#define RCC_FLAG_PORRST                ((uint8_t)0x7B)
#define RCC_FLAG_SFTRST                ((uint8_t)0x7C)
#define RCC_FLAG_IWDGRST               ((uint8_t)0x7D)
#define RCC_FLAG_WWDGRST               ((uint8_t)0x7E)
#define RCC_FLAG_LPWRRST               ((uint8_t)0x7F)

/* SysTick_clock_source */
#define SysTick_CLKSource_HCLK_Div8    ((uint32_t)0xFFFFFFFB)
#define SysTick_CLKSource_HCLK         ((uint32_t)0x00000004)


void        RCC_DeInit(void);
void        RCC_AdjustHSICalibrationValue(uint8_t HSICalibrationValue);
void        RCC_HSICmd(FunctionalState NewState);
void        RCC_HCLKConfig(uint32_t RCC_SYSCLK);
void        RCC_GetClocksFreq(RCC_ClocksTypeDef *RCC_Clocks);
void        RCC_AHBPeriphClockCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState);
void        RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);
void        RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);
void        RCC_AHBPeriphResetCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState);
void        RCC_APB2PeriphResetCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);
void        RCC_APB1PeriphResetCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);
void        RCC_MCOConfig(uint8_t RCC_MCO);
FlagStatus  RCC_GetFlagStatus(uint8_t RCC_FLAG);
void        RCC_ClearFlag(void);

#ifdef __cplusplus
}
#endif

#endif
