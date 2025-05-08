/********************************** (C) COPYRIGHT *******************************
 * File Name          : ch32x035_rcc.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/04/06
 * Description        : This file provides all the RCC firmware functions.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/ 
#include "ch32x035_rcc.h"

/* RCC registers bit mask */

/* CTLR register bit mask */
#define CTLR_HSITRIM_Mask           ((uint32_t)0xFFFFFF07)

/* CFGR0 register bit mask */
#define CFGR0_HPRE_Reset_Mask       ((uint32_t)0xFFFFFF0F)
#define CFGR0_HPRE_Set_Mask         ((uint32_t)0x000000F0)

/* RSTSCKR register bit mask */
#define RSTSCKR_RMVF_Set            ((uint32_t)0x01000000)

/* RCC Flag Mask */
#define FLAG_Mask                   ((uint8_t)0x1F)

/* CFGR0 register byte 4 (Bits[31:24]) base address */
#define CFGR0_BYTE4_ADDRESS         ((uint32_t)0x40021007)


static __I uint8_t APBAHBPrescTable[16] = {1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8};

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
    RCC->CFGR0 |= (uint32_t)0x00000050;
    RCC->CFGR0 &= (uint32_t)0xF8FFFF5F;
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
		RCC->CTLR |= (1<<0);
	}
	else{
		RCC->CTLR &= ~(1<<0);		
	}
}

/*********************************************************************
 * @fn      RCC_HCLKConfig
 *
 * @brief   Configures the AHB clock (HCLK).
 *
 * @param   RCC_SYSCLK - defines the AHB clock divider. This clock is derived from
 *        the system clock (SYSCLK).
 *            RCC_SYSCLK_Div1 - AHB clock = SYSCLK.
 *            RCC_SYSCLK_Div2 - AHB clock = SYSCLK/2.
 *            RCC_SYSCLK_Div3 - AHB clock = SYSCLK/3.
 *            RCC_SYSCLK_Div4 - AHB clock = SYSCLK/4.
 *            RCC_SYSCLK_Div5 - AHB clock = SYSCLK/5.
 *            RCC_SYSCLK_Div6 - AHB clock = SYSCLK/6.
 *            RCC_SYSCLK_Div7 - AHB clock = SYSCLK/7.
 *            RCC_SYSCLK_Div8 - AHB clock = SYSCLK/8.
 *            RCC_SYSCLK_Div16 - AHB clock = SYSCLK/16.
 *            RCC_SYSCLK_Div32 - AHB clock = SYSCLK/32.
 *            RCC_SYSCLK_Div64 - AHB clock = SYSCLK/64.
 *            RCC_SYSCLK_Div128 - AHB clock = SYSCLK/128.
 *            RCC_SYSCLK_Div256 - AHB clock = SYSCLK/256.
 *
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
void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks)
{
    uint32_t tmp = 0, presc = 0;

    RCC_Clocks->SYSCLK_Frequency = HSI_VALUE;

    tmp = RCC->CFGR0 & CFGR0_HPRE_Set_Mask;
    tmp = tmp >> 4;
    presc = APBAHBPrescTable[tmp];

    if(((RCC->CFGR0 & CFGR0_HPRE_Set_Mask) >> 4) < 8)
    {
        RCC_Clocks->HCLK_Frequency = RCC_Clocks->SYSCLK_Frequency / presc;
    }
    else
    {
        RCC_Clocks->HCLK_Frequency = RCC_Clocks->SYSCLK_Frequency >> presc;
    }

    RCC_Clocks->PCLK1_Frequency = RCC_Clocks->HCLK_Frequency;
    RCC_Clocks->PCLK2_Frequency = RCC_Clocks->HCLK_Frequency;
}

/*********************************************************************
 * @fn      RCC_AHBPeriphClockCmd
 *
 * @brief   Enables or disables the AHB peripheral clock.
 *
 * @param   RCC_AHBPeriph - specifies the AHB peripheral to gates its clock.
 *            RCC_AHBPeriph_DMA1.
 *            RCC_AHBPeriph_SRAM.
 *            RCC_AHBPeriph_USBFS.
 *            RCC_AHBPeriph_USBPD
 *          Note-
 *          SRAM  clock can be disabled only during sleep mode.
 *          NewState: ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_AHBPeriphClockCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
    RCC->AHBPCENR |= RCC_AHBPeriph;
  }
  else
  {
    RCC->AHBPCENR &= ~RCC_AHBPeriph;
  }
}

/*********************************************************************
 * @fn      RCC_APB2PeriphClockCmd
 *
 * @brief   Enables or disables the High Speed APB (APB2) peripheral clock.
 *
 * @param   RCC_APB2Periph - specifies the APB2 peripheral to gates its clock.
 *            RCC_APB2Periph_AFIO.
 *            RCC_APB2Periph_GPIOA.
 *            RCC_APB2Periph_GPIOB.
 *            RCC_APB2Periph_GPIOC.
 *            RCC_APB2Periph_ADC1.
 *            RCC_APB2Periph_TIM1.
 *            RCC_APB2Periph_SPI1.
 *            RCC_APB2Periph_USART1.
 *          NewState - ENABLE or DISABLE
 *
 * @return  none
 */
void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
    RCC->APB2PCENR |= RCC_APB2Periph;
  }
  else
  {
    RCC->APB2PCENR &= ~RCC_APB2Periph;
  }
}

/*********************************************************************
 * @fn      RCC_APB1PeriphClockCmd
 *
 * @brief   Enables or disables the Low Speed APB (APB1) peripheral clock.
 *
 * @param   RCC_APB1Periph - specifies the APB1 peripheral to gates its clock.
 *            RCC_APB1Periph_TIM2.
 *            RCC_APB1Periph_TIM3.
 *            RCC_APB1Periph_WWDG.
 *            RCC_APB1Periph_USART2.
 *            RCC_APB1Periph_USART3.
 *            RCC_APB1Periph_USART4
 *            RCC_APB1Periph_I2C1.
 *            RCC_APB1Periph_PWR.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
    RCC->APB1PCENR |= RCC_APB1Periph;
  }
  else
  {
    RCC->APB1PCENR &= ~RCC_APB1Periph;
  }
}

/*********************************************************************
 * @fn      RCC_AHBPeriphResetCmd
 *
 * @brief   Forces or releases AHB peripheral reset.
 *
 * @param   RCC_AHBPeriph - specifies the AHB peripheral to reset.
 *            RCC_AHBPeriph_USBFS.
 *            RCC_AHBPeriph_IO2W.
 *            RCC_AHBPeriph_USBPD.
 *          NewState: ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_AHBPeriphResetCmd(uint32_t RCC_AHBPeriph, FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
    RCC->AHBPCENR |= RCC_AHBPeriph;
  }
  else
  {
    RCC->AHBPCENR &= ~RCC_AHBPeriph;
  }
}

/*********************************************************************
 * @fn      RCC_APB2PeriphResetCmd
 *
 * @brief   Forces or releases APB (APB2) peripheral reset.
 *
 * @param   RCC_APB2Periph - specifies the APB2 peripheral to reset.
 *            RCC_APB2Periph_AFIO.
 *            RCC_APB2Periph_GPIOA.
 *            RCC_APB2Periph_GPIOB.
 *            RCC_APB2Periph_GPIOC.
 *            RCC_APB2Periph_ADC1.
 *            RCC_APB2Periph_TIM1.
 *            RCC_APB2Periph_SPI1.
 *            RCC_APB2Periph_USART1.
 *          NewState - ENABLE or DISABLE
 *
 * @return  none
 */
void RCC_APB2PeriphResetCmd(uint32_t RCC_APB2Periph, FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
    RCC->APB2PRSTR |= RCC_APB2Periph;
  }
  else
  {
    RCC->APB2PRSTR &= ~RCC_APB2Periph;
  }
}

/*********************************************************************
 * @fn      RCC_APB1PeriphResetCmd
 *
 * @brief   Forces or releases APB (APB1) peripheral reset.
 *
 * @param   RCC_APB1Periph - specifies the APB1 peripheral to reset.
 *            RCC_APB1Periph_TIM2.
 *            RCC_APB1Periph_TIM3.
 *            RCC_APB1Periph_WWDG.
 *            RCC_APB1Periph_USART2.
 *            RCC_APB1Periph_USART3.
 *            RCC_APB1Periph_USART4
 *            RCC_APB1Periph_I2C1.
 *            RCC_APB1Periph_PWR.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void RCC_APB1PeriphResetCmd(uint32_t RCC_APB1Periph, FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
    RCC->APB1PRSTR |= RCC_APB1Periph;
  }
  else
  {
    RCC->APB1PRSTR &= ~RCC_APB1Periph;
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
 *
 * @return  none
 */
void RCC_MCOConfig(uint8_t RCC_MCO)
{
  *(__IO uint8_t *) CFGR0_BYTE4_ADDRESS = RCC_MCO;
}

/*********************************************************************
 * @fn      RCC_GetFlagStatus
 *
 * @brief   Checks whether the specified RCC flag is set or not.
 *
 * @param   RCC_FLAG - specifies the flag to check.
 *            RCC_FLAG_HSIRDY - HSI oscillator clock ready.
 *            RCC_FLAG_OPARST - OPA reset.
 *            RCC_FLAG_PINRST - Pin reset.
 *            RCC_FLAG_PORRST - POR/PDR reset.
 *            RCC_FLAG_SFTRST - Software reset.
 *            RCC_FLAG_IWDGRST - Independent Watchdog reset.
 *            RCC_FLAG_WWDGRST - Window Watchdog reset.
 *            RCC_FLAG_LPWRRST - Low Power reset.
 *
 * @return  FlagStatus - SET or RESET.
 */
FlagStatus RCC_GetFlagStatus(uint8_t RCC_FLAG)
{
  uint32_t tmp = 0;
  uint32_t statusreg = 0;
	
  FlagStatus bitstatus = RESET;
  tmp = RCC_FLAG >> 5;
	
  if (tmp == 1)            
  {
    statusreg = RCC->CTLR;
  }
  else                    
  {
    statusreg = RCC->RSTSCKR;
  }

  tmp = RCC_FLAG & FLAG_Mask;
	
  if ((statusreg & ((uint32_t)1 << tmp)) != (uint32_t)RESET)
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




