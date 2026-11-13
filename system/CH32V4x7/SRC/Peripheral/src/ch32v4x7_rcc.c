/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32v4x7_rcc.c
* Author             : WCH
* Version            : V1.0.1
* Date               : 2026/06/26
* Description        : This file provides all the RCC firmware functions.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32v4x7_rcc.h"

/* RCC registers bit address in the alias region */
#define RCC_OFFSET                 (RCC_BASE - PERIPH_BASE)

/* BDCTLR Register */
#define BDCTLR_OFFSET              (RCC_OFFSET + 0x20)

/* RCC registers bit mask */

/* CTLR register bit mask */
#define CTLR_HSEBYP_Reset          ((uint32_t)0xFFFBFFFF)
#define CTLR_HSEBYP_Set            ((uint32_t)0x00040000)
#define CTLR_HSEON_Reset           ((uint32_t)0xFFFEFFFF)
#define CTLR_HSEON_Set             ((uint32_t)0x00010000)
#define CTLR_HSITRIM_Mask          ((uint32_t)0xFFFFFF07)

#define CFGR0_PLL_Mask             ((uint32_t)0xFFC0FFFF)
#define CFGR0_PLLMull_Mask         ((uint32_t)0x003E0000)
#define CFGR0_PLLSRC_Mask          ((uint32_t)0x00010000)
#define CFGR0_PLLXTPRE_Mask        ((uint32_t)0x00020000)
#define CFGR0_SWS_Mask             ((uint32_t)0x0000000C)
#define CFGR0_SW_Mask              ((uint32_t)0xFFFFFFFC)
#define CFGR0_HPRE_Reset_Mask      ((uint32_t)0xFFFFFF0F)
#define CFGR0_HPRE_Set_Mask        ((uint32_t)0x000000F0)
#define CFGR0_PPRE1_Reset_Mask     ((uint32_t)0xFFFFF8FF)
#define CFGR0_PPRE1_Set_Mask       ((uint32_t)0x00000700)
#define CFGR0_PPRE2_Reset_Mask     ((uint32_t)0xFFFFC7FF)
#define CFGR0_PPRE2_Set_Mask       ((uint32_t)0x00003800)
#define CFGR0_ADCPRE_Reset_Mask    ((uint32_t)0xFFFF3FFF)
#define CFGR0_ADCPRE_Set_Mask      ((uint32_t)0x0000C000)
#define CFGR3_ADCPRE_Reset_Mask    ((uint32_t)0xFFFFFFE0)

/* RSTSCKR register bit mask */
#define RSTSCKR_RMVF_Set           ((uint32_t)0x01000000)

/* RCC Flag Mask */
#define FLAG_Mask                  ((uint8_t)0x1F)

/* INTR register byte 2 (Bits[15:8]) base address */
#define INTR_BYTE2_ADDRESS         ((uint32_t)0x40021009)

/* INTR register byte 3 (Bits[23:16]) base address */
#define INTR_BYTE3_ADDRESS         ((uint32_t)0x4002100A)

/* CFGR0 register byte 4 (Bits[31:24]) base address */
#define CFGR0_BYTE4_ADDRESS        ((uint32_t)0x40021007)

/* BDCTLR register base address */
#define BDCTLR_ADDRESS             (PERIPH_BASE + BDCTLR_OFFSET)

static __I uint8_t HBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
static __I uint8_t PBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4};
static __I uint8_t PLLMULTB[32] = {8,9,10,11,12,25,13,27,14,29,15,31,16,33,17,35,18,37,19,39,20,21,22,24,26,28,30,32,34,36,38,40};
static __I uint8_t ADCPrescTable[4] = {2, 4, 6, 8};

/*********************************************************************
 * @fn      RCC_DeInit
 *
 * @brief   Resets the RCC clock configuration to the default reset state.
 *          Note-
 *          HSE can not be stopped if it is used directly or through the PLL as system clock.
 * @return  none
 */
void RCC_DeInit(void)
{
  RCC->CTLR |= (uint32_t)0x00000001;
  RCC->CFGR0 &= (uint32_t)0x80FF0000;
  while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x00)
  {
  }

  RCC->CTLR &= (uint32_t)0xFEA6FFFF;
  RCC->CTLR &= (uint32_t)0xFFFBFFFF;

  RCC->CFGR0 &= (uint32_t)0xFFC0FFFF;
  RCC->CFGR2 &= (uint32_t)0xFFFFEFFF;
  RCC->CFGR2 &= (uint32_t)0x03F11800;

  RCC->CFGR0 |= (uint32_t)(0x20000000);
  RCC->CFGR3 = 0x00000000;
  RCC->INTR = 0x008F0000;   
}

/*********************************************************************
 * @fn      RCC_HSEConfig
 *
 * @brief   Configures the External High Speed oscillator (HSE).
 *
 * @param   RCC_HSE -
 *            RCC_HSE_OFF - HSE oscillator OFF.
 *            RCC_HSE_ON - HSE oscillator ON.
 *            RCC_HSE_Bypass - HSE oscillator bypassed with external clock.
 *            Note-
 *            HSE can not be stopped if it is used directly or through the PLL as system clock.
 * @return  none
 */
void RCC_HSEConfig(uint32_t RCC_HSE)
{
    RCC->CTLR &= CTLR_HSEON_Reset;
    RCC->CTLR &= CTLR_HSEBYP_Reset;

    switch(RCC_HSE)
    {
        case RCC_HSE_ON:
            RCC->CTLR |= CTLR_HSEON_Set;
            break;

        case RCC_HSE_Bypass:
            RCC->CTLR |= CTLR_HSEBYP_Set | CTLR_HSEON_Set;
            break;

        default:
            break;
    }
}

/*********************************************************************
 * @fn      RCC_WaitForHSEStartUp
 *
 * @brief   Waits for HSE start-up.
 *
 * @return  READY - HSE oscillator is stable and ready to use.
 *          NoREADY - HSE oscillator not yet ready.
 */
ErrorStatus RCC_WaitForHSEStartUp(void)
{
    __IO uint32_t StartUpCounter = 0;

    ErrorStatus status = NoREADY;
    FlagStatus  HSEStatus = RESET;

    do
    {
        HSEStatus = RCC_GetFlagStatus(RCC_FLAG_HSERDY);
        StartUpCounter++;
    } while((StartUpCounter != HSE_STARTUP_TIMEOUT) && (HSEStatus == RESET));

    if(RCC_GetFlagStatus(RCC_FLAG_HSERDY) != RESET)
    {
        status = READY;
    }
    else
    {
        status = NoREADY;
    }

    return (status);
}

/*********************************************************************
 * @fn      RCC_AdjustHSICalibrationValue
 *
 * @brief   Adjusts the Internal High Speed oscillator (HSI) calibration value.
 *
 * @param   HSICalibrationValue - specifies the calibration trimming value.
 *                    This parameter must be a number between 0 and 0x1F.
 *
 * @return  none
 */
void RCC_AdjustHSICalibrationValue(uint8_t HSICalibrationValue)
{
    uint32_t tmpreg = 0;

    tmpreg = RCC->CTLR;
    tmpreg &= CTLR_HSITRIM_Mask;
    tmpreg |= (uint32_t)HSICalibrationValue << 3;
    RCC->CTLR = tmpreg;
}

/*********************************************************************
 * @fn      RCC_HSICmd
 *
 * @brief   Enables or disables the Internal High Speed oscillator (HSI).
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_HSICmd(FunctionalState NewState)
{
    if(NewState)
    {
        RCC->CTLR |= (1 << 0);
    }
    else
    {
        RCC->CTLR &= ~(1 << 0);
    }
}

/*********************************************************************
 * @fn      RCC_PLLConfig
 *
 * @brief   Configures the PLL clock source and multiplication factor.
 *
 * @param   RCC_PLLSource - specifies the PLL entry clock source.
 *            RCC_PLLSource_HSI - HSI oscillator clock 
 *            RCC_PLLSource_HSE - HSE oscillator clock 
 *          RCC_PLLMul - specifies the PLL multiplication factor.
 *            This parameter can be RCC_PLLMul_x where x:[8,40].
 *            For other CH32V4x7 -
 *              RCC_PLLMul_8
 *              RCC_PLLMul_9
 *              RCC_PLLMul_10
 *              RCC_PLLMul_11
 *              RCC_PLLMul_12
 *              RCC_PLLMul_12_5
 *              RCC_PLLMul_13
 *              RCC_PLLMul_13_5
 *              RCC_PLLMul_14
 *              RCC_PLLMul_14_5
 *              RCC_PLLMul_15
 *              RCC_PLLMul_15_5
 *              RCC_PLLMul_16
 *              RCC_PLLMul_16_5
 *              RCC_PLLMul_17
 *              RCC_PLLMul_17_5
 *              RCC_PLLMul_18
 *              RCC_PLLMul_18_5
 *              RCC_PLLMul_19
 *              RCC_PLLMul_19_5
 *              RCC_PLLMul_20
 *              RCC_PLLMul_21
 *              RCC_PLLMul_22
 *              RCC_PLLMul_24
 *              RCC_PLLMul_26
 *              RCC_PLLMul_28
 *              RCC_PLLMul_30
 *              RCC_PLLMul_32
 *              RCC_PLLMul_34
 *              RCC_PLLMul_36
 *              RCC_PLLMul_38
 *              RCC_PLLMul_40
 * @return  none
 */
void RCC_PLLConfig(uint32_t RCC_PLLSource, uint32_t RCC_PLLMul)
{
    uint32_t tmpreg = 0;
    tmpreg = RCC->CFGR0;
    tmpreg &= CFGR0_PLL_Mask;
    tmpreg |= RCC_PLLSource | RCC_PLLMul;
    RCC->CFGR0 = tmpreg;
}

/*********************************************************************
 * @fn      RCC_PLLCmd
 *
 * @brief   Enables or disables the PLL.
 *          Note-The PLL can not be disabled if it is used as system clock.
 *          
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_PLLCmd(FunctionalState NewState)
{
    if(NewState)
    {
        RCC->CTLR |= (1 << 24);
    }
    else
    {
        RCC->CTLR &= ~(1 << 24);
    }
}

/*********************************************************************
 * @fn      RCC_SYSCLKConfig
 *
 * @brief   Configures the system clock (SYSCLK).
 *
 * @param   RCC_SYSCLKSource - specifies the clock source used as system clock.
 *            RCC_SYSCLKSource_HSI - HSI selected as system clock.
 *            RCC_SYSCLKSource_HSE - HSE selected as system clock.
 *            RCC_SYSCLKSource_PLLCLK - PLL selected as system clock.
 *
 * @return  none
 */
void RCC_SYSCLKConfig(uint32_t RCC_SYSCLKSource)
{
    uint32_t tmpreg = 0;

    tmpreg = RCC->CFGR0;
    tmpreg &= CFGR0_SW_Mask;
    tmpreg |= RCC_SYSCLKSource;
    RCC->CFGR0 = tmpreg;
}

/*********************************************************************
 * @fn      RCC_GetSYSCLKSource
 *
 * @brief   Returns the clock source used as system clock.
 *
 * @return  0x00 - HSI used as system clock.
 *          0x04 - HSE used as system clock.
 *          0x08 - PLL used as system clock.
 */
uint8_t RCC_GetSYSCLKSource(void)
{
    return ((uint8_t)(RCC->CFGR0 & CFGR0_SWS_Mask));
}

/*********************************************************************
 * @fn      RCC_HCLKConfig
 *
 * @brief   Configures the HB clock (HCLK).
 *
 * @param   RCC_SYSCLK - defines the HB clock divider. This clock is derived from
 *        the system clock (SYSCLK).
 *            RCC_SYSCLK_Div1 - HB clock = SYSCLK.
 *            RCC_SYSCLK_Div2 - HB clock = SYSCLK/2.
 *            RCC_SYSCLK_Div4 - HB clock = SYSCLK/4.
 *            RCC_SYSCLK_Div8 - HB clock = SYSCLK/8.
 *            RCC_SYSCLK_Div16 - HB clock = SYSCLK/16.
 *            RCC_SYSCLK_Div64 - HB clock = SYSCLK/64.
 *            RCC_SYSCLK_Div128 - HB clock = SYSCLK/128.
 *            RCC_SYSCLK_Div256 - HB clock = SYSCLK/256.
 *            RCC_SYSCLK_Div512 - HB clock = SYSCLK/512.
 * @return  none
 */
void RCC_HCLKConfig(uint32_t RCC_SYSCLK)
{
    uint32_t tmpreg = 0;

    tmpreg = RCC->CFGR0;
    tmpreg &= CFGR0_HPRE_Reset_Mask;
    tmpreg |= RCC_SYSCLK;
    RCC->CFGR0 = tmpreg;
}

/*********************************************************************
 * @fn      RCC_PCLK1Config
 *
 * @brief   Configures the Low Speed APB clock (PCLK1).
 *
 * @param   RCC_HCLK - defines the PB1 clock divider. This clock is derived from
 *        the HB clock (HCLK).
 *            RCC_HCLK_Div1 - PB1 clock = HCLK.
 *            RCC_HCLK_Div2 - PB1 clock = HCLK/2.
 *            RCC_HCLK_Div4 - PB1 clock = HCLK/4.
 *            RCC_HCLK_Div8 - PB1 clock = HCLK/8.
 *            RCC_HCLK_Div16 - PB1 clock = HCLK/16.
 *
 * @return  none
 */
void RCC_PCLK1Config(uint32_t RCC_HCLK)
{
    uint32_t tmpreg = 0;

    tmpreg = RCC->CFGR0;
    tmpreg &= CFGR0_PPRE1_Reset_Mask;
    tmpreg |= RCC_HCLK;
    RCC->CFGR0 = tmpreg;
}

/*********************************************************************
 * @fn      RCC_PCLK2Config
 *
 * @brief   Configures the High Speed APB clock (PCLK2).
 *
 * @param   RCC_HCLK - defines the PB2 clock divider. This clock is derived from
 *        the HB clock (HCLK).
 *            RCC_HCLK_Div1 - PB2 clock = HCLK.
 *            RCC_HCLK_Div2 - PB2 clock = HCLK/2.
 *            RCC_HCLK_Div4 - PB2 clock = HCLK/4.
 *            RCC_HCLK_Div8 - PB2 clock = HCLK/8.
 *            RCC_HCLK_Div16 - PB2 clock = HCLK/16.
 *
 * @return  none
 */
void RCC_PCLK2Config(uint32_t RCC_HCLK)
{
    uint32_t tmpreg = 0;
    tmpreg = RCC->CFGR0;
    tmpreg &= CFGR0_PPRE2_Reset_Mask;
    tmpreg |= RCC_HCLK << 3;
    RCC->CFGR0 = tmpreg;
}

/*********************************************************************
 * @fn      RCC_ITConfig
 *
 * @brief   Enables or disables the specified RCC interrupts.
 *
 * @param   RCC_IT - specifies the RCC interrupt sources to be enabled or disabled.
 *            RCC_IT_LSIRDY - LSI ready interrupt.
 *            RCC_IT_LSERDY - LSE ready interrupt.
 *            RCC_IT_HSIRDY - HSI ready interrupt.
 *            RCC_IT_HSERDY - HSE ready interrupt.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_ITConfig(uint8_t RCC_IT, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        *(__IO uint8_t *)INTR_BYTE2_ADDRESS |= RCC_IT;
    }
    else
    {
        *(__IO uint8_t *)INTR_BYTE2_ADDRESS &= (uint8_t)~RCC_IT;
    }
}

/*********************************************************************
 * @fn      RCC_ADCCLKConfig
 *
 * @brief   Configures the ADC clock (ADCCLK).
 *
 * @param   RCC_PCLK2 - defines the ADC clock divider. This clock is derived from
 *        the PB2 clock (PCLK2).
 *          RCC_PCLK2_Div2 - ADC clock = PCLK2/2.
 *          RCC_PCLK2_Div4 - ADC clock = PCLK2/4.
 *          RCC_PCLK2_Div6 - ADC clock = PCLK2/6.
 *          RCC_PCLK2_Div8 - ADC clock = PCLK2/8.
 *
 * @return  none
 */
void RCC_ADCCLKConfig(uint32_t RCC_PCLK2)
{
    uint32_t tmpreg = 0;
    tmpreg = RCC->CFGR0;
    tmpreg &= CFGR0_ADCPRE_Reset_Mask;
    tmpreg |= RCC_PCLK2;
    RCC->CFGR0 = tmpreg;
}

/*********************************************************************
 * @fn      RCC_LSEConfig
 *
 * @brief   Configures the External Low Speed oscillator (LSE).
 *
 * @param   RCC_LSE - specifies the new state of the LSE.
 *            RCC_LSE_OFF - LSE oscillator OFF.
 *            RCC_LSE_ON - LSE oscillator ON.
 *            RCC_LSE_Bypass - LSE oscillator bypassed with external clock.
 *
 * @return  none
 */
void RCC_LSEConfig(uint8_t RCC_LSE)
{
    *(__IO uint8_t *)BDCTLR_ADDRESS = RCC_LSE_OFF;
    *(__IO uint8_t *)BDCTLR_ADDRESS = RCC_LSE_OFF;

    switch(RCC_LSE)
    {
        case RCC_LSE_ON:
            *(__IO uint8_t *)BDCTLR_ADDRESS = RCC_LSE_ON;
            break;

        case RCC_LSE_Bypass:
            *(__IO uint8_t *)BDCTLR_ADDRESS = RCC_LSE_Bypass | RCC_LSE_ON;
            break;

        default:
            break;
    }
}

/*********************************************************************
 * @fn      RCC_LSICmd
 *
 * @brief   Enables or disables the Internal Low Speed oscillator (LSI).
 *          Note-
 *          LSI can not be disabled if the IWDG is running.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_LSICmd(FunctionalState NewState)
{
    if(NewState)
    {
        RCC->RSTSCKR |= (1 << 0);
    }
    else
    {
        RCC->RSTSCKR &= ~(1 << 0);
    }
}

/*********************************************************************
 * @fn      RCC_RTCCLKConfig
 *
 * @brief   Once the RTC clock is selected it can't be changed unless the Backup domain is reset.
 *
 * @param   RCC_RTCCLKConfig - specifies the RTC clock source.
 *            RCC_RTCCLKSource_LSE - LSE selected as RTC clock.
 *            RCC_RTCCLKSource_LSI - LSI selected as RTC clock.
 *            RCC_RTCCLKSource_HSE_Div128 - HSE clock divided by 128 selected as RTC clock.
 *         Note-   
 *           Once the RTC clock is selected it can't be changed unless the Backup domain is reset.
 * @return  none
 */
void RCC_RTCCLKConfig(uint32_t RCC_RTCCLKSource)
{
    RCC->BDCTLR |= RCC_RTCCLKSource;
}

/*********************************************************************
 * @fn      RCC_RTCCLKCmd
 *
 * @brief   This function must be used only after the RTC clock was selected
 *        using the RCC_RTCCLKConfig function.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_RTCCLKCmd(FunctionalState NewState)
{
    if(NewState)
    {
        RCC->BDCTLR |= (1 << 15);
    }
    else
    {
        RCC->BDCTLR &= ~(1 << 15);
    }
}

/*********************************************************************
 * @fn      RCC_GetClocksFreq
 *
 * @brief   The result of this function could be not correct when using
 *        fractional value for HSE crystal.
 *
 * @param   RCC_Clocks - pointer to a RCC_ClocksTypeDef structure which will hold
 *        the clocks frequencies.
 *
 * @return  none
 */
void RCC_GetClocksFreq(RCC_ClocksTypeDef *RCC_Clocks)
{
    uint32_t tmp = 0, tmp1 = 0, tmp2 = 0, pllmull = 0, pllsource = 0, presc = 0;
    uint8_t Plldiv2 = 0;
  
    tmp = RCC->CFGR0 & RCC_SWS;
    tmp1 = RCC->CFGR2 & RCC_SYSPLLSRC;
  
    switch (tmp)
    {
      case 0x00:
        RCC_Clocks->SYSCLK_Frequency = HSI_VALUE;
        break;
      case 0x04:  
      RCC_Clocks->SYSCLK_Frequency = HSE_VALUE;
        break;
      case 0x08: 
        pllsource = RCC->CFGR0 & RCC_PLLSRC; 
        presc = ((RCC->CFGR2 & RCC_SYSPLLDIV)>>8) + 1;
        tmp2 = (RCC->CFGR0 & RCC_PLLMULL)>>17;
        pllmull = PLLMULTB[tmp2];
        if((tmp2 > 4) && (tmp2 < 20) && ((tmp2 % 2) != 0))
        {
          Plldiv2 = 1;
        }
  
        switch (tmp1) 
        {
          case RCC_SYSPLLSRC_USBHSPLL:
          RCC_Clocks->SYSCLK_Frequency = 480000000;
          break;
  
          case RCC_SYSPLLSRC_ETHPLL:
          RCC_Clocks->SYSCLK_Frequency = 500000000;
          break;
  
          case RCC_SYSPLLSRC_USBHSPLL_MUL2_DIV3:
          RCC_Clocks->SYSCLK_Frequency = 320000000;
          break;
  
          case RCC_SYSPLLSRC_ETHPLL_MUL2_DIV3:
          RCC_Clocks->SYSCLK_Frequency = 333000000;
          break;
  
          case RCC_SYSPLLSRC_PLL:
  
            if(pllsource == 0x00) //HSI
            {
                RCC_Clocks->SYSCLK_Frequency = HSI_VALUE*pllmull;
            }
            else 
            {
                RCC_Clocks->SYSCLK_Frequency = HSE_VALUE*pllmull;
            }
            RCC_Clocks->SYSCLK_Frequency /= (1 + Plldiv2);
          break;
  
          default:
          break;
        }
        RCC_Clocks->SYSCLK_Frequency /= presc;
        break;
  
        default:
        RCC_Clocks->SYSCLK_Frequency = HSI_VALUE;
        break;
    }
    tmp = HBPrescTable[((RCC->CFGR0 & CFGR0_HPRE_Set_Mask) >> 4)];
    RCC_Clocks->HCLK_Frequency =  RCC_Clocks->SYSCLK_Frequency >> tmp;

    tmp = PBPrescTable[((RCC->CFGR0 & CFGR0_PPRE1_Set_Mask) >> 8)];
    RCC_Clocks->PCLK1_Frequency =  RCC_Clocks->HCLK_Frequency >> tmp;

    tmp = PBPrescTable[((RCC->CFGR0 & CFGR0_PPRE2_Set_Mask) >> 11)];
    RCC_Clocks->PCLK2_Frequency =  RCC_Clocks->HCLK_Frequency >> tmp;

    
    if((RCC->CFGR0 & 0x10000000) == 0x10000000)
    {
        tmp = 480000000/((RCC->CFGR3)+1);
    }
    else
    {
        tmp = RCC_Clocks->HCLK_Frequency;
    }
    tmp1 = tmp >> PBPrescTable[((RCC->CFGR0 & CFGR0_PPRE2_Set_Mask) >> 11)];
    tmp = RCC->CFGR0 & CFGR0_ADCPRE_Set_Mask;
    tmp = tmp >> 14;
    presc = ADCPrescTable[tmp];
    RCC_Clocks->ADCCLK_Frequency = tmp1 / presc;

}

/*********************************************************************
 * @fn      RCC_HBPeriphClockCmd
 *
 * @brief   Enables or disables the HB peripheral clock.
 *
 * @param   RCC_HBPeriph - specifies the HB peripheral to gates its clock.
 *            RCC_HBPeriph_DMA1.
 *            RCC_HBPeriph_DMA2.
 *            RCC_HBPeriph_SRAM.
 *            RCC_HBPeriph_LTDC
 *            RCC_HBPeriph_PSRAM
 *            RCC_HBPeriph_I3C
 *            RCC_HBPeriph_CRC.
 *            RCC_HBPeriph_ARGB
 *            RCC_HBPeriph_FSMC
 *            RCC_HBPeriph_RNG
 *            RCC_HBPeriph_SDIO
 *            RCC_HBPeriph_USBHS1
 *            RCC_HBPeriph_USBHS2
 *            RCC_HBPeriph_DVP
 *            RCC_HBPeriph_ETH_MAC
 *          Note-
 *          SRAM  clock can be disabled only during sleep mode.
 *          NewState: ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_HBPeriphClockCmd(uint32_t RCC_HBPeriph, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        RCC->HBPCENR |= RCC_HBPeriph;
    }
    else
    {
        RCC->HBPCENR &= ~RCC_HBPeriph;
    }
}

/*********************************************************************
 * @fn      RCC_PB2PeriphClockCmd
 *
 * @brief   Enables or disables the High Speed APB (PB2) peripheral clock.
 *
 * @param   RCC_PB2Periph - specifies the PB2 peripheral to gates its clock.
 *            RCC_PB2Periph_AFIO.
 *            RCC_PB2Periph_GPIOA.
 *            RCC_PB2Periph_GPIOB.
 *            RCC_PB2Periph_GPIOC.
 *            RCC_PB2Periph_GPIOD.
 *            RCC_PB2Periph_GPIOE
 *            RCC_PB2Periph_ADC1.
 *            RCC_PB2Periph_ADC2
 *            RCC_PB2Periph_TIM1.
 *            RCC_PB2Periph_SPI1.
 *            RCC_PB2Periph_TIM8
 *            RCC_PB2Periph_USART1.
 *          NewState - ENABLE or DISABLE
 *
 * @return  none
 */
void RCC_PB2PeriphClockCmd(uint32_t RCC_PB2Periph, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        RCC->PB2PCENR |= RCC_PB2Periph;
    }
    else
    {
        RCC->PB2PCENR &= ~RCC_PB2Periph;
    }
}

/*********************************************************************
 * @fn      RCC_PB1PeriphClockCmd
 *
 * @brief   Enables or disables the Low Speed APB (PB1) peripheral clock.
 *
 * @param   RCC_PB1Periph - specifies the PB1 peripheral to gates its clock.
 *            RCC_PB1Periph_TIM2.
 *            RCC_PB1Periph_TIM3.
 *            RCC_PB1Periph_TIM4.
 *            RCC_PB1Periph_TIM5
 *            RCC_PB1Periph_TIM6
 *            RCC_PB1Periph_TIM7
 *            RCC_PB1Periph_UART6
 *            RCC_PB1Periph_UART7
 *            RCC_PB1Periph_UART8
 *            RCC_PB1Periph_UART9
 *            RCC_PB1Periph_UART10
 *            RCC_PB1Periph_WWDG.
 *            RCC_PB1Periph_SPI2.
 *            RCC_PB1Periph_SPI3.
 *            RCC_PB1Periph_USART2.
 *            RCC_PB1Periph_USART3.
 *            RCC_PB1Periph_USART4
 *            RCC_PB1Periph_USART5
 *            RCC_PB1Periph_I2C.
 *            RCC_PB1Periph_CAN1.
 *            RCC_PB1Periph_BKP.
 *            RCC_PB1Periph_PWR.
 *            RCC_PB1Periph_DAC.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_PB1PeriphClockCmd(uint32_t RCC_PB1Periph, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        RCC->PB1PCENR |= RCC_PB1Periph;
    }
    else
    {
        RCC->PB1PCENR &= ~RCC_PB1Periph;
    }
}

/*********************************************************************
 * @fn      RCC_PB2PeriphResetCmd
 *
 * @brief   Forces or releases High Speed APB (PB2) peripheral reset.
 *
 * @param   RCC_PB2Periph - specifies the PB2 peripheral to reset.
 *            RCC_PB2Periph_AFIO.
 *            RCC_PB2Periph_GPIOA.
 *            RCC_PB2Periph_GPIOB.
 *            RCC_PB2Periph_GPIOC.
 *            RCC_PB2Periph_GPIOD.
 *            RCC_PB2Periph_GPIOE
 *            RCC_PB2Periph_ADC1.
 *            RCC_PB2Periph_ADC2
 *            RCC_PB2Periph_TIM1.
 *            RCC_PB2Periph_SPI1.
 *            RCC_PB2Periph_TIM8
 *            RCC_PB2Periph_USART1.
 *          NewState - ENABLE or DISABLE
 *
 * @return  none
 */
void RCC_PB2PeriphResetCmd(uint32_t RCC_PB2Periph, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        RCC->PB2PRSTR |= RCC_PB2Periph;
    }
    else
    {
        RCC->PB2PRSTR &= ~RCC_PB2Periph;
    }
}

/*********************************************************************
 * @fn      RCC_PB1PeriphResetCmd
 *
 * @brief   Forces or releases Low Speed APB (PB1) peripheral reset.
 *
 * @param   RCC_PB1Periph - specifies the PB1 peripheral to reset.
 *            RCC_PB1Periph_TIM2.
 *            RCC_PB1Periph_TIM3.
 *            RCC_PB1Periph_TIM4.
 *            RCC_PB1Periph_TIM5
 *            RCC_PB1Periph_TIM6
 *            RCC_PB1Periph_TIM7
 *            RCC_PB1Periph_UART6
 *            RCC_PB1Periph_UART7
 *            RCC_PB1Periph_UART8
 *            RCC_PB1Periph_UART9
 *            RCC_PB1Periph_UART10
 *            RCC_PB1Periph_WWDG.
 *            RCC_PB1Periph_SPI2.
 *            RCC_PB1Periph_SPI3.
 *            RCC_PB1Periph_USART2.
 *            RCC_PB1Periph_USART3.
 *            RCC_PB1Periph_USART4
 *            RCC_PB1Periph_USART5
 *            RCC_PB1Periph_I2C1.
 *            RCC_PB1Periph_CAN1.
 *            RCC_PB1Periph_BKP.
 *            RCC_PB1Periph_PWR.
 *            RCC_PB1Periph_DAC.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_PB1PeriphResetCmd(uint32_t RCC_PB1Periph, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        RCC->PB1PRSTR |= RCC_PB1Periph;
    }
    else
    {
        RCC->PB1PRSTR &= ~RCC_PB1Periph;
    }
}

/*********************************************************************
 * @fn      RCC_BackupResetCmd
 *
 * @brief   Forces or releases the Backup domain reset.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_BackupResetCmd(FunctionalState NewState)
{
    if(NewState)
    {
        RCC->BDCTLR |= (1 << 16);
    }
    else
    {
        RCC->BDCTLR &= ~(1 << 16);
    }
}

/*********************************************************************
 * @fn      RCC_ClockSecuritySystemCmd
 *
 * @brief   Enables or disables the Clock Security System.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_ClockSecuritySystemCmd(FunctionalState NewState)
{
    if(NewState)
    {
        RCC->CTLR |= (1 << 19);
    }
    else
    {
        RCC->CTLR &= ~(1 << 19);
    }
}

/*********************************************************************
 * @fn      RCC_MCOConfig
 *
 * @brief   Selects the clock source to output on MCO pin.
 *
 * @param   RCC_MCO - specifies the clock source to output.
 *            RCC_MCO_NoClock - No clock selected.
 *            RCC_MCO_SYSCLK - System clock selected.
 *            RCC_MCO_HSI - HSI oscillator clock selected.
 *            RCC_MCO_HSE - HSE oscillator clock selected.
 *            RCC_MCO_UTMI - UTMI clock divided by 2 selected
 *            RCC_MCO_XT1 - External 3-25 MHz oscillator clock selected
 *            RCC_MCO_ETHPLL_Div8 - ETH divided 8 clock selected
 *
 * @return  none
 */
void RCC_MCOConfig(uint8_t RCC_MCO)
{
    *(__IO uint8_t *)CFGR0_BYTE4_ADDRESS = RCC_MCO;
}

/*********************************************************************
 * @fn      RCC_GetFlagStatus
 *
 * @brief   Checks whether the specified RCC flag is set or not.
 *
 * @param   RCC_FLAG - specifies the flag to check.
 *            RCC_FLAG_HSIRDY - HSI oscillator clock ready.
 *            RCC_FLAG_HSERDY - HSE oscillator clock ready.
 *            RCC_FLAG_PLLRDY - PLL clock ready.
 *            RCC_FLAG_LSERDY - LSE oscillator clock ready.
 *            RCC_FLAG_LSIRDY - LSI oscillator clock ready.
 *            RCC_FLAG_PINRST - Pin reset.
 *            RCC_FLAG_PORRST - POR/PDR reset.
 *            RCC_FLAG_SFTRST - Software reset.
 *            RCC_FLAG_IWDGRST - Independent Watchdog reset.
 *            RCC_FLAG_WWDGRST - Window Watchdog reset.
 *            RCC_FLAG_LPWRRST - Low Power reset
 *
 * @return  FlagStatus - SET or RESET.
 */
FlagStatus RCC_GetFlagStatus(uint8_t RCC_FLAG)
{
    uint32_t tmp = 0;
    uint32_t statusreg = 0;

    FlagStatus bitstatus = RESET;
    tmp = RCC_FLAG >> 5;

    if(tmp == 1)
    {
        statusreg = RCC->CTLR;
    }
    else if(tmp == 2)
    {
        statusreg = RCC->BDCTLR;
    }
    else
    {
        statusreg = RCC->RSTSCKR;
    }

    tmp = RCC_FLAG & FLAG_Mask;

    if((statusreg & ((uint32_t)1 << tmp)) != (uint32_t)RESET)
    {
        bitstatus = SET;
    }
    else
    {
        bitstatus = RESET;
    }

    return bitstatus;
}

/*********************************************************************
 * @fn      RCC_ClearFlag
 *
 * @brief   Clears the RCC reset flags.
 *          Note-   
 *          The reset flags are: RCC_FLAG_PINRST, RCC_FLAG_PORRST, RCC_FLAG_SFTRST,
 *          RCC_FLAG_IWDGRST, RCC_FLAG_WWDGRST, RCC_FLAG_LPWRRST
 * @return  none
 */
void RCC_ClearFlag(void)
{
    RCC->RSTSCKR |= RSTSCKR_RMVF_Set;
}

/*********************************************************************
 * @fn      RCC_GetITStatus
 *
 * @brief   Checks whether the specified RCC interrupt has occurred or not.
 *
 * @param   RCC_IT - specifies the RCC interrupt source to check.
 *            RCC_IT_LSIRDY - LSI ready interrupt.
 *            RCC_IT_LSERDY - LSE ready interrupt.
 *            RCC_IT_HSIRDY - HSI ready interrupt.
 *            RCC_IT_HSERDY - HSE ready interrupt.
 *            RCC_IT_CSS - Clock Security System interrupt.
 *
 * @return  ITStatus - SET or RESET.
 */

ITStatus RCC_GetITStatus(uint8_t RCC_IT)
{
    ITStatus bitstatus = RESET;

    if((RCC->INTR & RCC_IT) != (uint32_t)RESET)
    {
        bitstatus = SET;
    }
    else
    {
        bitstatus = RESET;
    }

    return bitstatus;
}

/*********************************************************************
 * @fn      RCC_ClearITPendingBit
 *
 * @brief   Clears the RCC's interrupt pending bits.
 *
 * @param   RCC_IT - specifies the interrupt pending bit to clear.
 *            RCC_IT_LSIRDY - LSI ready interrupt.
 *            RCC_IT_LSERDY - LSE ready interrupt.
 *            RCC_IT_HSIRDY - HSI ready interrupt.
 *            RCC_IT_HSERDY - HSE ready interrupt.
 *            RCC_IT_CSS - Clock Security System interrupt.
 *
 * @return  none
 */
void RCC_ClearITPendingBit(uint8_t RCC_IT)
{
    *(__IO uint8_t *)INTR_BYTE3_ADDRESS = RCC_IT;
}

/*********************************************************************
 * @fn      RCC_I2S2CLKConfig
 *
 * @brief   Configures the I2S2 clock source(I2S2CLK).
 *
 * @param   RCC_I2S2CLKSource - specifies the I2S2 clock source.
 *          RCC_I2S2CLKSource_SYSCLK - system clock selected as I2S2 clock entry
 *          RCC_I2S2CLKSource_USBHSPLL - USBHS_PLLclock selected as I2S2 clock entry
 *          RCC_I2S2CLKSource_ETHPLL - ETH_PLLclock selected as I2S2 clock entry
 *           note:ETHPLL/USBHSPLL div4 as I2S2 clock
 * @return  none
 */
void RCC_I2S2CLKConfig(uint8_t RCC_I2S2CLKSource)
{
    if(RCC_I2S2CLKSource == RCC_I2S2CLKSource_USBHSPLL)
    {
        RCC->CFGR2 &=~(1<<7);
        RCC->CFGR2 |= (1<<17);        
    }
    else if(RCC_I2S2CLKSource == RCC_I2S2CLKSource_ETHPLL)
    {
        RCC->CFGR2 |= (1<<7);
        RCC->CFGR2 |= (1<<17);      
    }
    else if(RCC_I2S2CLKSource == RCC_I2S2CLKSource_SYSCLK)
    {
        RCC->CFGR2 &=~(1<<17); 
    }
}

/*********************************************************************
 * @fn      RCC_I2S3CLKConfig
 *
 * @brief   Configures the I2S3 clock source(I2S2CLK).
 *
 * @param   RCC_I2S3CLKSource - specifies the I2S3 clock source.
 *          RCC_I2S3CLKSource_SYSCLK - system clock selected as I2S3 clock entry
 *          RCC_I2S3CLKSource_USBHSPLL - USBHS_PLLclock selected as I2S3 clock entry
 *          RCC_I2S3CLKSource_ETHPLL - ETH_PLLclock selected as I2S3 clock entry
 *          note:ETHPLL/USBHSPLL div4 as I2S3 clock
 * @return  none
 */
void RCC_I2S3CLKConfig(uint8_t RCC_I2S3CLKSource)
{
    if(RCC_I2S3CLKSource == RCC_I2S3CLKSource_USBHSPLL)
    {
        RCC->CFGR2 &=~(1<<7);
        RCC->CFGR2 |= (1<<18);        
    }
    else if(RCC_I2S3CLKSource == RCC_I2S3CLKSource_ETHPLL)
    {
        RCC->CFGR2 |= (1<<7);
        RCC->CFGR2 |= (1<<18);      
    }
    else if(RCC_I2S3CLKSource == RCC_I2S3CLKSource_SYSCLK)
    {
        RCC->CFGR2 &=~(1<<18); 
    }
}

/*********************************************************************
 * @fn      RCC_HBPeriphResetCmd
 *
 * @brief   Forces or releases HB peripheral reset.
 *
 * @param   RCC_HBPeriph - specifies the HB peripheral to reset.
 *            RCC_HBPeriph_LTDC
 *            RCC_HBPeriph_PSRAM
 *            RCC_HBPeriph_I3C
 *            RCC_HBPeriph_ARGB
 *            RCC_HBPeriph_USBHS1
 *            RCC_HBPeriph_USBHS2
 *            RCC_HBPeriph_DVP
 *            RCC_HBPeriph_ETH_MAC
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_HBPeriphResetCmd(uint32_t RCC_HBPeriph, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        RCC->HBRSTR |= RCC_HBPeriph;
    }
    else
    {
        RCC->HBRSTR &= ~RCC_HBPeriph;
    }
}

/*********************************************************************
 * @fn      RCC_RNGCLKConfig
 *
 * @brief   Configures the RNG clock source.
 *
 * @param   RCC_RNGCLKSource - specifies the RNG clock source.
 *            RCC_RNGCLKSource_SYSCLK - system clock selected as RNG clock entry
 *            RCC_RNGCLKSource_USBHSPLL - USBHSPLL clock selected as RNG clock entry
 *            RCC_RNGCLKSource_ETHPLL- ETHPLL clock selected as RNG clock entry
 * 
 * @return  none
 */
void RCC_RNGCLKConfig(uint8_t RCC_RNGCLKSource)
{
    if(RCC_RNGCLKSource == RCC_RNGCLKSource_USBHSPLL)
    {
        RCC->CFGR2 &=~(1<<7);
        RCC->CFGR2 |= (1<<19);        
    }
    else if(RCC_RNGCLKSource == RCC_RNGCLKSource_ETHPLL)
    {
        RCC->CFGR2 |= (1<<7);
        RCC->CFGR2 |= (1<<19);      
    }
    else if(RCC_RNGCLKSource == RCC_RNGCLKSource_SYSCLK)
    {
        RCC->CFGR2 &=~(1<<19); 
    }
}

/*********************************************************************
 * @fn      RCC_USBHSPLLCLKConfig
 *
 * @brief   Configures the USBHSPLL clock source.
 *
 * @param   RCC_HSBHSPLLCLKSource - specifies the USBHSPLL clock source.
 *            RCC_USBHSPLLCLKSource_HSE - HSE clock selected as USBHSPLL clock entry
 *            RCC_USBHSPLLCLKSource_HSI - HSI clock selected as USBHSPLL clock entry
 *            RCC_USBHSPLLCLKSource_ETHCLK_20 - ETH clock 20M selected as USBHSPLL clock entry
 * @return  none
 */
void RCC_USBHSPLLCLKConfig(uint32_t RCC_USBHSPLLCLKSource)
{

    RCC->CFGR2 &= ~(3 << 26);
    RCC->CFGR2 |= RCC_USBHSPLLCLKSource << 26;
}

/*********************************************************************
 * @fn      RCC_USBHSPLLReferConfig
 *
 * @brief   Configures the USBHSPLL reference clock.
 *
 * @param   RCC_USBHSPLLCKREFCLKSource - Select reference clock.
 *            RCC_USBHSPLLCKREFCLK_25M - reference clock 25Mhz.
 *            RCC_USBHSPLLCKREFCLK_20M - reference clock 20Mhz.
 *            RCC_USBHSPLLCKREFCLK_24M - reference clock 24Mhz.
 *            RCC_USBHSPLLCKREFCLK_32M - reference clock 32Mhz.
 *
 * @return  none
 */
void RCC_USBHSPLLReferConfig(uint32_t RCC_USBHSPLLCKREFCLKSource)
{
    RCC->CFGR2 &= ~(3 << 28);
    RCC->CFGR2 |= RCC_USBHSPLLCKREFCLKSource << 28;
}

/*********************************************************************
 * @fn      RCC_UTMI1cmd
 *
 * @brief   Enable UTMI1 clock gate control.
 *
 * @param   NewState - ENABLE or DISABLE.
 * 
 * @return  none
 */
void RCC_UTMI1cmd(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        RCC->CFGR2 |= (1 << 30);
    }
    else
    {
        RCC->CFGR2 &= ~(1 << 30);
    }
}

/*********************************************************************
 * @fn      RCC_UTMI2cmd
 *
 * @brief   Enable UTMI2 clock gate control.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_UTMI2cmd(FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        RCC->CFGR2 |= (1 << 31);
    }
    else
    {
        RCC->CFGR2 &= ~(1 << 31);
    }
}

/*********************************************************************
 * @fn      RCC_ADCCLKSourceUSBHSConfig
 *
 * @brief   Configures the ADC clock (ADCCLK).
 *
 * @param   RCC_CLKDIV - defines the ADC clock divider. This clock is derived from
 *        the USBHSPLL clock .
 *          RCC_CLKDIV1 - ADC clock = USBHSPLL.
 *          RCC_CLKDIV2 - ADC clock = USBHSPLL/2.
 *          RCC_CLKDIV3 - ADC clock = USBHSPLL/3.
 *          RCC_CLKDIV4 - ADC clock = USBHSPLL/4.
 *          RCC_CLKDIV5 - ADC clock = USBHSPLL/5.
 *          RCC_CLKDIV6 - ADC clock = USBHSPLL/6.
 *          RCC_CLKDIV7 - ADC clock = USBHSPLL/7.
 *          RCC_CLKDIV8 - ADC clock = USBHSPLL/8.
 *          RCC_CLKDIV9 - ADC clock = USBHSPLL/9.
 *          RCC_CLKDIV10 - ADC clock = USBHSPLL/10.
 *          RCC_CLKDIV11 - ADC clock = USBHSPLL/11.
 *          RCC_CLKDIV12 - ADC clock = USBHSPLL/12.
 *          RCC_CLKDIV13 - ADC clock = USBHSPLL/13.
 *          RCC_CLKDIV14 - ADC clock = USBHSPLL/14.
 *          RCC_CLKDIV15 - ADC clock = USBHSPLL/15.
 *          RCC_CLKDIV16 - ADC clock = USBHSPLL/16.
 *          RCC_CLKDIV17 - ADC clock = USBHSPLL/17.
 *          RCC_CLKDIV18 - ADC clock = USBHSPLL/18.
 *          RCC_CLKDIV19 - ADC clock = USBHSPLL/19.
 *          RCC_CLKDIV20 - ADC clock = USBHSPLL/20.
 *          RCC_CLKDIV21 - ADC clock = USBHSPLL/21.
 *          RCC_CLKDIV22 - ADC clock = USBHSPLL/22.
 *          RCC_CLKDIV23 - ADC clock = USBHSPLL/23.
 *          RCC_CLKDIV24 - ADC clock = USBHSPLL/24.
 *          RCC_CLKDIV25 - ADC clock = USBHSPLL/25.
 *          RCC_CLKDIV26 - ADC clock = USBHSPLL/26.
 *          RCC_CLKDIV27 - ADC clock = USBHSPLL/27.
 *          RCC_CLKDIV28 - ADC clock = USBHSPLL/28.
 *          RCC_CLKDIV29 - ADC clock = USBHSPLL/29.
 *          RCC_CLKDIV30 - ADC clock = USBHSPLL/30.
 *          RCC_CLKDIV31 - ADC clock = USBHSPLL/31.
 *          RCC_CLKDIV32 - ADC clock = USBHSPLL/32.
 *
 * @return  none
 */
void RCC_ADCClockSourceUSBHSConfig(uint32_t RCC_CLKDIV)
{
    uint32_t tmpreg = 0;
    tmpreg = RCC->CFGR3;
    tmpreg &= CFGR3_ADCPRE_Reset_Mask;
    tmpreg |= RCC_CLKDIV;
    RCC->CFGR3 = tmpreg;
}

/*********************************************************************
 * @fn      RCC_SYSPLLConfig
 *
 * @brief   Configures the System PLL clock source.
 *
 * @param   RCC_SYSPLLClockSource - the System PLL clock source.
 *            RCC_SYSPLLClockSource_PLL - PLL clock selected as System PLL clock entry
 *            RCC_SYSPLLClockSource_USBHSPLL - USBHS PLL clock selected as System PLL clock entry
 *            RCC_SYSPLLClockSource_ETHPLL - ETH PLL clock selected as System PLL clock entry
 *            RCC_SYSPLLClockSource_USBHS_Mul2_Div3 - SERDES PLL clock selected as System PLL clock entry
 *            RCC_SYSPLLClockSource_ETH_Mul2_Div3 - USBSS PLL clock selected as System PLL clock entry
 *
 * @return  none
 */
void RCC_SYSPLLConfig(uint32_t RCC_SYSPLLClockSource)
{
    RCC->CFGR2 &= ~RCC_SYSPLLSRC;
    RCC->CFGR2 |= (RCC_SYSPLLClockSource << 13);
}

/*********************************************************************
 * @fn      RCC_SYSPLLGATEcmd
 *
 * @brief   ENABLE or DISABLE System clock  gate.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_SYSPLLGATEcmd(FunctionalState NewState)
{
    if(NewState)
    {
        RCC->CFGR2 |= RCC_SYSPLL_GATE;
    }
    else
    {
        RCC->CFGR2 &= ~RCC_SYSPLL_GATE;
    }
}

/*********************************************************************
 * @fn      RCC_ADCCLKSourceConfig
 *
 * @brief   Configures the ADC clock source.
 *
 * @param   RCC_ADCCLKSourceConfig - specifies the ADC clock source.
 *            RCC_ADCCLKSource_HCLK -  HCLK clock selected as ADC clock entry
 *            RCC_ADCCLKSource_USBHSPLL - USBHS PLL clock selected as ADC clock entry
 *
 * @return  none
 */
void RCC_ADCCLKSourceConfig(uint32_t RCC_ADCCLKSource)
{
    RCC->CFGR0 &= ~RCC_CFGR0_ADCSRC;
    RCC->CFGR0 |= (RCC_ADCCLKSource << 28);
}

/*********************************************************************
 * @fn      RCC_ADCCLKDutyCycleConfig
 *
 * @brief   Configures the ADC clock high level duty cycle.
 *
 * @param   RCC_DutyCycle - high level duty cycle.
 *            RCC_DutyCycle_50 - ADC clock 50% duty cycle .
 *            RCC_DutyCycle_62_5 -ADC clock 62_5% duty cycle ..
 *            RCC_DutyCycle_75 -ADC clock 75% duty cycle ..
 *
 * @return  none
 */
void RCC_ADCCLKDutyCycleConfig(uint32_t RCC_DutyCycle)
{
    RCC->CFGR0 &= ~RCC_ADC_DUTY_SEL;
    RCC->CFGR0 |= RCC_DutyCycle;
}

/*********************************************************************
 * @fn      RCC_LTDCCLKConfig
 *
 * @brief   Configures the LTDC clock source.
 *
 * @param   RCC_LTDCClockSource - the LTDC clock source.
 *            RCC_LTDCClockSource_ETHPLL - ETH PLL clock selected as LTDC clock entry
 *            RCC_LTDCClockSource_USBHSPLL - USBHS PLL clock selected as LTDC clock entry
 *
 * @return  none
 */
void RCC_LTDCCLKConfig(uint32_t RCC_LTDCClockSource)
{
    RCC->CFGR3 &= ~RCC_LTDCSRC;
    RCC->CFGR3 |= (RCC_LTDCClockSource << 6);
}

/*********************************************************************
 * @fn      RCC_LTDCClockSourceDivConfig
 *
 * @brief   Configures the LTDC clock Source Division.
 *
 * @param   RCC_LTDCClockSource_Div - the LTDC clock Source Division
 *            RCC_LTDCClockSource_Divx - x can select(1~64)
 *
 * @return  none
 */
void RCC_LTDCClockSourceDivConfig(uint32_t RCC_LTDCClockSource_Div)
{
    RCC->CFGR3 &= ~RCC_LTDCDIV;
    RCC->CFGR3 |= RCC_LTDCClockSource_Div << 0;
}