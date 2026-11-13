/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32v4x7_rcc.h
* Author             : WCH
* Version            : V1.0.1
* Date               : 2026/06/26
* Description        : This file provides all the RCC firmware functions.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32V4x7_RCC_H
#define __CH32V4x7_RCC_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "ch32v4x7.h"

/* RCC_Exported_Types */
typedef struct
{
  uint32_t SYSCLK_Frequency;  /* returns SYSCLK clock frequency expressed in Hz */
  uint32_t HCLK_Frequency;    /* returns HCLK clock frequency expressed in Hz */
  uint32_t PCLK1_Frequency;   /* returns PCLK1 clock frequency expressed in Hz */
  uint32_t PCLK2_Frequency;   /* returns PCLK2 clock frequency expressed in Hz */
  uint32_t ADCCLK_Frequency;  /* returns ADCCLK clock frequency expressed in Hz */
}RCC_ClocksTypeDef;

/* HSE_configuration */
#define RCC_HSE_OFF                      ((uint32_t)0x00000000)
#define RCC_HSE_ON                       ((uint32_t)0x00010000)
#define RCC_HSE_Bypass                   ((uint32_t)0x00040000)

#define RCC_PLLSource_HSI                ((uint32_t)0x00000000)
#define RCC_PLLSource_HSE                ((uint32_t)0x00010000)

/* PLL_multiplication_factor */
#define RCC_PLLMul_8                     ((uint32_t)0x00000000)
#define RCC_PLLMul_9                     ((uint32_t)0x00020000)
#define RCC_PLLMul_10                    ((uint32_t)0x00040000)
#define RCC_PLLMul_11                    ((uint32_t)0x00060000)
#define RCC_PLLMul_12                    ((uint32_t)0x00080000)
#define RCC_PLLMul_12_5                  ((uint32_t)0x000A0000)
#define RCC_PLLMul_13                    ((uint32_t)0x000C0000)
#define RCC_PLLMul_13_5                  ((uint32_t)0x000E0000)
#define RCC_PLLMul_14                    ((uint32_t)0x00100000)
#define RCC_PLLMul_14_5                  ((uint32_t)0x00120000)
#define RCC_PLLMul_15                    ((uint32_t)0x00140000)
#define RCC_PLLMul_15_5                  ((uint32_t)0x00160000)
#define RCC_PLLMul_16                    ((uint32_t)0x00180000)
#define RCC_PLLMul_16_5                  ((uint32_t)0x001A0000)
#define RCC_PLLMul_17                    ((uint32_t)0x001C0000)
#define RCC_PLLMul_17_5                  ((uint32_t)0x001E0000)
#define RCC_PLLMul_18                    ((uint32_t)0x00200000)
#define RCC_PLLMul_18_5                  ((uint32_t)0x00220000)
#define RCC_PLLMul_19                    ((uint32_t)0x00240000)
#define RCC_PLLMul_19_5                  ((uint32_t)0x00260000)
#define RCC_PLLMul_20                    ((uint32_t)0x00280000)
#define RCC_PLLMul_21                    ((uint32_t)0x002A0000)
#define RCC_PLLMul_22                    ((uint32_t)0x002C0000)
#define RCC_PLLMul_24                    ((uint32_t)0x002E0000)
#define RCC_PLLMul_26                    ((uint32_t)0x00300000)
#define RCC_PLLMul_28                    ((uint32_t)0x00320000)
#define RCC_PLLMul_30                    ((uint32_t)0x00340000)
#define RCC_PLLMul_32                    ((uint32_t)0x00360000)
#define RCC_PLLMul_34                    ((uint32_t)0x00380000)
#define RCC_PLLMul_36                    ((uint32_t)0x003A0000)
#define RCC_PLLMul_38                    ((uint32_t)0x003C0000)
#define RCC_PLLMul_40                    ((uint32_t)0x003E0000)

/* System_clock_source */
#define RCC_SYSCLKSource_HSI             ((uint32_t)0x00000000)
#define RCC_SYSCLKSource_HSE             ((uint32_t)0x00000001)
#define RCC_SYSCLKSource_PLLCLK          ((uint32_t)0x00000002)

/* HB_clock_source */
#define RCC_SYSCLK_Div1                  ((uint32_t)0x00000000)
#define RCC_SYSCLK_Div2                  ((uint32_t)0x00000080)
#define RCC_SYSCLK_Div4                  ((uint32_t)0x00000090)
#define RCC_SYSCLK_Div8                  ((uint32_t)0x000000A0)
#define RCC_SYSCLK_Div16                 ((uint32_t)0x000000B0)
#define RCC_SYSCLK_Div64                 ((uint32_t)0x000000C0)
#define RCC_SYSCLK_Div128                ((uint32_t)0x000000D0)
#define RCC_SYSCLK_Div256                ((uint32_t)0x000000E0)
#define RCC_SYSCLK_Div512                ((uint32_t)0x000000F0)

/* PB1_PB2_clock_source */
#define RCC_HCLK_Div1                    ((uint32_t)0x00000000)
#define RCC_HCLK_Div2                    ((uint32_t)0x00000400)
#define RCC_HCLK_Div4                    ((uint32_t)0x00000500)
#define RCC_HCLK_Div8                    ((uint32_t)0x00000600)
#define RCC_HCLK_Div16                   ((uint32_t)0x00000700)

/* RCC_Interrupt_source */
#define RCC_IT_LSIRDY                    ((uint8_t)0x01)
#define RCC_IT_LSERDY                    ((uint8_t)0x02)
#define RCC_IT_HSIRDY                    ((uint8_t)0x04)
#define RCC_IT_HSERDY                    ((uint8_t)0x08)
#define RCC_IT_CSS                       ((uint8_t)0x80)

/* I2S2_clock_source */
#define RCC_I2S2CLKSource_SYSCLK         ((uint8_t)0x00)
#define RCC_I2S2CLKSource_USBHSPLL       ((uint8_t)0x01)
#define RCC_I2S2CLKSource_ETHPLL         ((uint8_t)0x02)

/* I2S3_clock_source */
#define RCC_I2S3CLKSource_SYSCLK         ((uint8_t)0x00)
#define RCC_I2S3CLKSource_USBHSPLL       ((uint8_t)0x01)
#define RCC_I2S3CLKSource_ETHPLL         ((uint8_t)0x02)

/* ADC_clock_source */
#define RCC_PCLK2_Div2                   ((uint32_t)0x00000000)
#define RCC_PCLK2_Div4                   ((uint32_t)0x00004000)
#define RCC_PCLK2_Div6                   ((uint32_t)0x00008000)
#define RCC_PCLK2_Div8                   ((uint32_t)0x0000C000)

/* LSE_configuration */
#define RCC_LSE_OFF                      ((uint8_t)0x00)
#define RCC_LSE_ON                       ((uint8_t)0x01)
#define RCC_LSE_Bypass                   ((uint8_t)0x04)

/* RTC_clock_source */
#define RCC_RTCCLKSource_LSE             ((uint32_t)0x00000100)
#define RCC_RTCCLKSource_LSI             ((uint32_t)0x00000200)
#define RCC_RTCCLKSource_HSE_Div128      ((uint32_t)0x00000300)

/* HB_peripheral */
#define RCC_HBPeriph_DMA1               ((uint32_t)0x00000001)
#define RCC_HBPeriph_DMA2               ((uint32_t)0x00000002)
#define RCC_HBPeriph_SRAM               ((uint32_t)0x00000004)
#define RCC_HBPeriph_LTDC               ((uint32_t)0x00000008)
#define RCC_HBPeriph_PSRAM              ((uint32_t)0x00000010)
#define RCC_HBPeriph_I3C                ((uint32_t)0x00000020)
#define RCC_HBPeriph_CRC                ((uint32_t)0x00000040)
#define RCC_HBPeriph_ARGB               ((uint32_t)0x00000080)
#define RCC_HBPeriph_FSMC               ((uint32_t)0x00000100)
#define RCC_HBPeriph_RNG                ((uint32_t)0x00000200)
#define RCC_HBPeriph_SDIO               ((uint32_t)0x00000400)
#define RCC_HBPeriph_USBHS2             ((uint32_t)0x00000800)
#define RCC_HBPeriph_USBHS1             ((uint32_t)0x00001000)
#define RCC_HBPeriph_DVP                ((uint32_t)0x00002000)
#define RCC_HBPeriph_ETH_MAC            ((uint32_t)0x00004000)

/* PB2_peripheral */
#define RCC_PB2Periph_AFIO              ((uint32_t)0x00000001)
#define RCC_PB2Periph_GPIOA             ((uint32_t)0x00000004)
#define RCC_PB2Periph_GPIOB             ((uint32_t)0x00000008)
#define RCC_PB2Periph_GPIOC             ((uint32_t)0x00000010)
#define RCC_PB2Periph_GPIOD             ((uint32_t)0x00000020)
#define RCC_PB2Periph_GPIOE             ((uint32_t)0x00000040)
#define RCC_PB2Periph_ADC1              ((uint32_t)0x00000200)
#define RCC_PB2Periph_ADC2              ((uint32_t)0x00000400)
#define RCC_PB2Periph_TIM1              ((uint32_t)0x00000800)
#define RCC_PB2Periph_SPI1              ((uint32_t)0x00001000)
#define RCC_PB2Periph_TIM8              ((uint32_t)0x00002000)
#define RCC_PB2Periph_USART1            ((uint32_t)0x00004000)

/* PB1_peripheral */
#define RCC_PB1Periph_TIM2              ((uint32_t)0x00000001)
#define RCC_PB1Periph_TIM3              ((uint32_t)0x00000002)
#define RCC_PB1Periph_TIM4              ((uint32_t)0x00000004)
#define RCC_PB1Periph_TIM5              ((uint32_t)0x00000008)
#define RCC_PB1Periph_TIM6              ((uint32_t)0x00000010)
#define RCC_PB1Periph_TIM7              ((uint32_t)0x00000020)
#define RCC_PB1Periph_USART6            ((uint32_t)0x00000040)
#define RCC_PB1Periph_USART7            ((uint32_t)0x00000080)
#define RCC_PB1Periph_USART8            ((uint32_t)0x00000100)
#define RCC_PB1Periph_USART9            ((uint32_t)0x00000200)
#define RCC_PB1Periph_USART10           ((uint32_t)0x00000400)
#define RCC_PB1Periph_WWDG              ((uint32_t)0x00000800)
#define RCC_PB1Periph_SPI2              ((uint32_t)0x00004000)
#define RCC_PB1Periph_SPI3              ((uint32_t)0x00008000)
#define RCC_PB1Periph_USART2            ((uint32_t)0x00020000)
#define RCC_PB1Periph_USART3            ((uint32_t)0x00040000)
#define RCC_PB1Periph_USART4            ((uint32_t)0x00080000)
#define RCC_PB1Periph_USART5            ((uint32_t)0x00100000)
#define RCC_PB1Periph_I2C1              ((uint32_t)0x00200000)
#define RCC_PB1Periph_CAN1              ((uint32_t)0x02000000)
#define RCC_PB1Periph_BKP               ((uint32_t)0x08000000)
#define RCC_PB1Periph_PWR               ((uint32_t)0x10000000)
#define RCC_PB1Periph_DAC               ((uint32_t)0x20000000)

/* Clock_source_to_output_on_MCO_pin */
#define RCC_MCO_NoClock                  ((uint8_t)0x00)
#define RCC_MCO_SYSCLK                   ((uint8_t)0x04)
#define RCC_MCO_HSI                      ((uint8_t)0x05)
#define RCC_MCO_HSE                      ((uint8_t)0x06)
#define RCC_MCO_UTMI                     ((uint8_t)0x07)
#define RCC_MCO_ETHPLL_Div8              ((uint8_t)0x08)
#define RCC_MCO_XT1                      ((uint8_t)0x0A)

/* RCC_Flag */
#define RCC_FLAG_HSIRDY                  ((uint8_t)0x21)
#define RCC_FLAG_HSERDY                  ((uint8_t)0x31)
#define RCC_FLAG_PLLRDY                  ((uint8_t)0x39)
#define RCC_FLAG_LSERDY                  ((uint8_t)0x41)
#define RCC_FLAG_LSIRDY                  ((uint8_t)0x61)
#define RCC_FLAG_PINRST                  ((uint8_t)0x7A)
#define RCC_FLAG_PORRST                  ((uint8_t)0x7B)
#define RCC_FLAG_SFTRST                  ((uint8_t)0x7C)
#define RCC_FLAG_IWDGRST                 ((uint8_t)0x7D)
#define RCC_FLAG_WWDGRST                 ((uint8_t)0x7E)
#define RCC_FLAG_LPWRRST                 ((uint8_t)0x7F)

/* SysTick_clock_source */
#define SysTick_CLKSource_HCLK_Div8      ((uint32_t)0xFFFFFFFB)
#define SysTick_CLKSource_HCLK           ((uint32_t)0x00000004)

/* RNG_clock_source */
#define RCC_RNGCLKSource_SYSCLK          ((uint32_t)0x00)
#define RCC_RNGCLKSource_USBHSPLL        ((uint32_t)0x01)
#define RCC_RNGCLKSource_ETHPLL          ((uint32_t)0x02)

/* USBHSPLL_clock_source */
#define RCC_USBHSPLLCLKSource_HSE        ((uint32_t)0x00)
#define RCC_USBHSPLLCLKSource_HSI        ((uint32_t)0x01)
#define RCC_USBHSPLLCLKSource_ETHCLK_20  ((uint32_t)0x02)

/* USBHSPLLCKREF_clock_select */
#define RCC_USBHSPLLCKREFCLK_25M          ((uint32_t)0x00)
#define RCC_USBHSPLLCKREFCLK_20M          ((uint32_t)0x01)
#define RCC_USBHSPLLCKREFCLK_24M          ((uint32_t)0x02)
#define RCC_USBHSPLLCKREFCLK_32M          ((uint32_t)0x03)

/* ADC_clock_Duty_Cycle */
#define RCC_DutyCycle_50                  ((uint32_t)0x00000000)
#define RCC_DutyCycle_62_5                ((uint32_t)0x20000000)
#define RCC_DutyCycle_75                  ((uint32_t)0x40000000)

/* ADC_clock_source */
#define RCC_ADCCLKSource_HCLK            ((uint8_t)0x00)
#define RCC_ADCCLKSource_USBHSPLL        ((uint8_t)0x01)

#define  RCC_ADC_DUTY_SEL                ((uint32_t)0x60000000)

/* SYSPLL_clock_source */
#define RCC_SYSPLLClockSource_PLL              ((uint8_t)0x06)
#define RCC_SYSPLLClockSource_USBHSPLL         ((uint8_t)0x00)
#define RCC_SYSPLLClockSource_ETHPLL           ((uint8_t)0x01)
#define RCC_SYSPLLClockSource_USBHS_Mul2_Div3  ((uint8_t)0x02)
#define RCC_SYSPLLClockSource_ETH_Mul2_Div3    ((uint8_t)0x03)

/* LTDC_clock_source */
#define RCC_LTDCClockSource_ETHPLL       ((uint8_t)0x01)
#define RCC_LTDCClockSource_USBHSPLL     ((uint8_t)0x00)

/* LTDC_division_factor */
#define RCC_LTDCClockSource_Div1         ((uint8_t)0x00)
#define RCC_LTDCClockSource_Div2         ((uint8_t)0x01)
#define RCC_LTDCClockSource_Div3         ((uint8_t)0x02)
#define RCC_LTDCClockSource_Div4         ((uint8_t)0x03)
#define RCC_LTDCClockSource_Div5         ((uint8_t)0x04)
#define RCC_LTDCClockSource_Div6         ((uint8_t)0x05)
#define RCC_LTDCClockSource_Div7         ((uint8_t)0x06)
#define RCC_LTDCClockSource_Div8         ((uint8_t)0x07)
#define RCC_LTDCClockSource_Div9         ((uint8_t)0x08)
#define RCC_LTDCClockSource_Div10        ((uint8_t)0x09)
#define RCC_LTDCClockSource_Div11        ((uint8_t)0x0A)
#define RCC_LTDCClockSource_Div12        ((uint8_t)0x0B)
#define RCC_LTDCClockSource_Div13        ((uint8_t)0x0C)
#define RCC_LTDCClockSource_Div14        ((uint8_t)0x0D)
#define RCC_LTDCClockSource_Div15        ((uint8_t)0x0E)
#define RCC_LTDCClockSource_Div16        ((uint8_t)0x0F)
#define RCC_LTDCClockSource_Div17        ((uint8_t)0x10)
#define RCC_LTDCClockSource_Div18        ((uint8_t)0x11)
#define RCC_LTDCClockSource_Div19        ((uint8_t)0x12)
#define RCC_LTDCClockSource_Div20        ((uint8_t)0x13)
#define RCC_LTDCClockSource_Div21        ((uint8_t)0x14)
#define RCC_LTDCClockSource_Div22        ((uint8_t)0x15)
#define RCC_LTDCClockSource_Div23        ((uint8_t)0x16)
#define RCC_LTDCClockSource_Div24        ((uint8_t)0x17)
#define RCC_LTDCClockSource_Div25        ((uint8_t)0x18)
#define RCC_LTDCClockSource_Div26        ((uint8_t)0x19)
#define RCC_LTDCClockSource_Div27        ((uint8_t)0x1A)
#define RCC_LTDCClockSource_Div28        ((uint8_t)0x1B)
#define RCC_LTDCClockSource_Div29        ((uint8_t)0x1C)
#define RCC_LTDCClockSource_Div30        ((uint8_t)0x1D)
#define RCC_LTDCClockSource_Div31        ((uint8_t)0x1E)
#define RCC_LTDCClockSource_Div32        ((uint8_t)0x1F)
#define RCC_LTDCClockSource_Div33        ((uint8_t)0x20)
#define RCC_LTDCClockSource_Div34        ((uint8_t)0x21)
#define RCC_LTDCClockSource_Div35        ((uint8_t)0x22)
#define RCC_LTDCClockSource_Div36        ((uint8_t)0x23)
#define RCC_LTDCClockSource_Div37        ((uint8_t)0x24)
#define RCC_LTDCClockSource_Div38        ((uint8_t)0x25)
#define RCC_LTDCClockSource_Div39        ((uint8_t)0x26)
#define RCC_LTDCClockSource_Div40        ((uint8_t)0x27)
#define RCC_LTDCClockSource_Div41        ((uint8_t)0x28)
#define RCC_LTDCClockSource_Div42        ((uint8_t)0x29)
#define RCC_LTDCClockSource_Div43        ((uint8_t)0x2A)
#define RCC_LTDCClockSource_Div44        ((uint8_t)0x2B)
#define RCC_LTDCClockSource_Div45        ((uint8_t)0x2C)
#define RCC_LTDCClockSource_Div46        ((uint8_t)0x2D)
#define RCC_LTDCClockSource_Div47        ((uint8_t)0x2E)
#define RCC_LTDCClockSource_Div48        ((uint8_t)0x2F)
#define RCC_LTDCClockSource_Div49        ((uint8_t)0x30)
#define RCC_LTDCClockSource_Div50        ((uint8_t)0x31)
#define RCC_LTDCClockSource_Div51        ((uint8_t)0x32)
#define RCC_LTDCClockSource_Div52        ((uint8_t)0x33)
#define RCC_LTDCClockSource_Div53        ((uint8_t)0x34)
#define RCC_LTDCClockSource_Div54        ((uint8_t)0x35)
#define RCC_LTDCClockSource_Div55        ((uint8_t)0x36)
#define RCC_LTDCClockSource_Div56        ((uint8_t)0x37)
#define RCC_LTDCClockSource_Div57        ((uint8_t)0x38)
#define RCC_LTDCClockSource_Div58        ((uint8_t)0x39)
#define RCC_LTDCClockSource_Div59        ((uint8_t)0x3A)
#define RCC_LTDCClockSource_Div60        ((uint8_t)0x3B)
#define RCC_LTDCClockSource_Div61        ((uint8_t)0x3C)
#define RCC_LTDCClockSource_Div62        ((uint8_t)0x3D)
#define RCC_LTDCClockSource_Div63        ((uint8_t)0x3E)
#define RCC_LTDCClockSource_Div64        ((uint8_t)0x3F)

#define RCC_CLKDIV1                      ((uint32_t)0x00000000)
#define RCC_CLKDIV2                      ((uint32_t)0x00000001)
#define RCC_CLKDIV3                      ((uint32_t)0x00000002)
#define RCC_CLKDIV4                      ((uint32_t)0x00000003)
#define RCC_CLKDIV5                      ((uint32_t)0x00000004)
#define RCC_CLKDIV6                      ((uint32_t)0x00000005)
#define RCC_CLKDIV7                      ((uint32_t)0x00000006)
#define RCC_CLKDIV8                      ((uint32_t)0x00000007)
#define RCC_CLKDIV9                      ((uint32_t)0x00000008)
#define RCC_CLKDIV10                     ((uint32_t)0x00000009)
#define RCC_CLKDIV11                     ((uint32_t)0x0000000A)
#define RCC_CLKDIV13                     ((uint32_t)0x0000000B)
#define RCC_CLKDIV14                     ((uint32_t)0x0000000C)
#define RCC_CLKDIV15                     ((uint32_t)0x0000000D)
#define RCC_CLKDIV16                     ((uint32_t)0x0000000E)
#define RCC_CLKDIV17                     ((uint32_t)0x0000000F)
#define RCC_CLKDIV18                     ((uint32_t)0x00000011)
#define RCC_CLKDIV19                     ((uint32_t)0x00000012)
#define RCC_CLKDIV20                     ((uint32_t)0x00000013)
#define RCC_CLKDIV21                     ((uint32_t)0x00000014)
#define RCC_CLKDIV22                     ((uint32_t)0x00000015)
#define RCC_CLKDIV23                     ((uint32_t)0x00000016)
#define RCC_CLKDIV24                     ((uint32_t)0x00000017)
#define RCC_CLKDIV25                     ((uint32_t)0x00000018)
#define RCC_CLKDIV26                     ((uint32_t)0x00000019)
#define RCC_CLKDIV27                     ((uint32_t)0x0000001A)
#define RCC_CLKDIV28                     ((uint32_t)0x0000001B)
#define RCC_CLKDIV29                     ((uint32_t)0x0000001C)
#define RCC_CLKDIV30                     ((uint32_t)0x0000001D)
#define RCC_CLKDIV31                     ((uint32_t)0x0000001E)
#define RCC_CLKDIV32                     ((uint32_t)0x0000001F)

void RCC_DeInit(void);
void RCC_HSEConfig(uint32_t RCC_HSE);
ErrorStatus RCC_WaitForHSEStartUp(void);
void RCC_AdjustHSICalibrationValue(uint8_t HSICalibrationValue);
void RCC_HSICmd(FunctionalState NewState);
void RCC_PLLConfig(uint32_t RCC_PLLSource, uint32_t RCC_PLLMul);
void RCC_PLLCmd(FunctionalState NewState);
void RCC_SYSCLKConfig(uint32_t RCC_SYSCLKSource);
uint8_t RCC_GetSYSCLKSource(void);
void RCC_HCLKConfig(uint32_t RCC_SYSCLK);
void RCC_PCLK1Config(uint32_t RCC_HCLK);
void RCC_PCLK2Config(uint32_t RCC_HCLK);
void RCC_ITConfig(uint8_t RCC_IT, FunctionalState NewState);
void RCC_ADCCLKConfig(uint32_t RCC_PCLK2);
void RCC_LSEConfig(uint8_t RCC_LSE);
void RCC_LSICmd(FunctionalState NewState);
void RCC_RTCCLKConfig(uint32_t RCC_RTCCLKSource);
void RCC_RTCCLKCmd(FunctionalState NewState);
void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks);
void RCC_HBPeriphClockCmd(uint32_t RCC_HBPeriph, FunctionalState NewState);
void RCC_PB2PeriphClockCmd(uint32_t RCC_PB2Periph, FunctionalState NewState);
void RCC_PB1PeriphClockCmd(uint32_t RCC_PB1Periph, FunctionalState NewState); 
void RCC_PB2PeriphResetCmd(uint32_t RCC_PB2Periph, FunctionalState NewState);
void RCC_PB1PeriphResetCmd(uint32_t RCC_PB1Periph, FunctionalState NewState);
void RCC_BackupResetCmd(FunctionalState NewState);
void RCC_ClockSecuritySystemCmd(FunctionalState NewState);
void RCC_MCOConfig(uint8_t RCC_MCO);
FlagStatus RCC_GetFlagStatus(uint8_t RCC_FLAG);
void RCC_ClearFlag(void);
ITStatus RCC_GetITStatus(uint8_t RCC_IT);
void RCC_ClearITPendingBit(uint8_t RCC_IT);
void RCC_I2S2CLKConfig(uint8_t RCC_I2S2CLKSource);
void RCC_I2S3CLKConfig(uint8_t RCC_I2S3CLKSource);
void RCC_HBPeriphResetCmd(uint32_t RCC_HBPeriph, FunctionalState NewState);
void RCC_RNGCLKConfig(uint8_t RCC_RNGCLKSource);
void RCC_USBHSPLLCLKConfig(uint32_t RCC_USBHSPLLCLKSource);
void RCC_USBHSPLLReferConfig(uint32_t RCC_USBHSPLLCKREFCLKSource);
void RCC_UTMI2cmd(FunctionalState NewState);
void RCC_UTMI1cmd(FunctionalState NewState);
void RCC_ADCClockSourceUSBHSConfig(uint32_t RCC_CLKDIV);
void RCC_SYSPLLConfig(uint32_t RCC_SYSPLLClockSource);
void RCC_SYSPLLGATEcmd(FunctionalState NewState);
void RCC_ADCCLKSourceConfig(uint32_t RCC_ADCCLKSource);
void RCC_ADCCLKDutyCycleConfig(uint32_t RCC_DutyCycle);
void RCC_LTDCCLKConfig(uint32_t RCC_LTDCClockSource);
void RCC_LTDCClockSourceDivConfig(uint32_t RCC_LTDCClockSource_Div);

#ifdef __cplusplus
}
#endif

#endif 





