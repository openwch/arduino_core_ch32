/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32l103_gpio.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/07/08
 * Description        : This file provides all the GPIO firmware functions.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "ch32l103_gpio.h"
#include "ch32l103_rcc.h"

/* MASK */
#define ECR_PORTPINCONFIG_MASK    ((uint16_t)0xFF80)
#define LSB_MASK                  ((uint16_t)0xFFFF)
#define DBGAFR_POSITION_MASK      ((uint32_t)0x000F0000)
#define DBGAFR_SWJCFG_MASK        ((uint32_t)0xF8FFFFFF)
#define DBGAFR_LOCATION_MASK      ((uint32_t)0x00200000)
#define DBGAFR_NUMBITS_MASK       ((uint32_t)0x00100000)
#define REMAP_MASK                ((uint32_t)0xC0000000)
#define REMAP_NUM_MASK            ((uint32_t)0x38000000)

uint32_t OPA_Trim = 0;
uint16_t ADC_Trim = 0;
uint32_t TS_Val = 0;
uint32_t CHIPID = 0;

/*********************************************************************
 * @fn      GPIO_DeInit
 *
 * @brief   Deinitializes the GPIOx peripheral registers to their default
 *        reset values.
 *
 * @param   GPIOx - where x can be (A..D) to select the GPIO peripheral.
 *
 * @return  none
 */
void GPIO_DeInit(GPIO_TypeDef *GPIOx)
{
    if(GPIOx == GPIOA)
    {
        RCC_PB2PeriphResetCmd(RCC_PB2Periph_GPIOA, ENABLE);
        RCC_PB2PeriphResetCmd(RCC_PB2Periph_GPIOA, DISABLE);
    }
    else if(GPIOx == GPIOB)
    {
        RCC_PB2PeriphResetCmd(RCC_PB2Periph_GPIOB, ENABLE);
        RCC_PB2PeriphResetCmd(RCC_PB2Periph_GPIOB, DISABLE);
    }
    else if(GPIOx == GPIOC)
    {
        RCC_PB2PeriphResetCmd(RCC_PB2Periph_GPIOC, ENABLE);
        RCC_PB2PeriphResetCmd(RCC_PB2Periph_GPIOC, DISABLE);
    }
    else if(GPIOx == GPIOD)
    {
        RCC_PB2PeriphResetCmd(RCC_PB2Periph_GPIOD, ENABLE);
        RCC_PB2PeriphResetCmd(RCC_PB2Periph_GPIOD, DISABLE);
    }
}

/*********************************************************************
 * @fn      GPIO_AFIODeInit
 *
 * @brief   Deinitializes the Alternate Functions (remap, event control
 *        and EXTI configuration) registers to their default reset values.
 *
 * @return  none
 */
void GPIO_AFIODeInit(void)
{
    RCC_PB2PeriphResetCmd(RCC_PB2Periph_AFIO, ENABLE);
    RCC_PB2PeriphResetCmd(RCC_PB2Periph_AFIO, DISABLE);
}

/*********************************************************************
 * @fn      GPIO_Init
 *
 * @brief   GPIOx - where x can be (A..D) to select the GPIO peripheral.
 *
 * @param   GPIO_InitStruct - pointer to a GPIO_InitTypeDef structure that
 *        contains the configuration information for the specified GPIO peripheral.
 *
 * @return  none
 */
void GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_InitStruct)
{
    uint32_t currentmode = 0x00, currentpin = 0x00, pinpos = 0x00, pos = 0x00;
    uint32_t tmpreg = 0x00, pinmask = 0x00;

    currentmode = ((uint32_t)GPIO_InitStruct->GPIO_Mode) & ((uint32_t)0x0F);

    if((((uint32_t)GPIO_InitStruct->GPIO_Mode) & ((uint32_t)0x10)) != 0x00)
    {
        currentmode |= (uint32_t)GPIO_InitStruct->GPIO_Speed;
    }

    if(((uint32_t)GPIO_InitStruct->GPIO_Pin & ((uint32_t)0x00FF)) != 0x00)
    {
        tmpreg = GPIOx->CFGLR;

        for(pinpos = 0x00; pinpos < 0x08; pinpos++)
        {
            pos = ((uint32_t)0x01) << pinpos;
            currentpin = (GPIO_InitStruct->GPIO_Pin) & pos;

            if(currentpin == pos)
            {
                pos = pinpos << 2;
                pinmask = ((uint32_t)0x0F) << pos;
                tmpreg &= ~pinmask;
                tmpreg |= (currentmode << pos);

                if(GPIO_InitStruct->GPIO_Mode == GPIO_Mode_IPD)
                {
                    GPIOx->BCR = (((uint32_t)0x01) << pinpos);
                }
                else
                {
                    if(GPIO_InitStruct->GPIO_Mode == GPIO_Mode_IPU)
                    {
                        GPIOx->BSHR = (((uint32_t)0x01) << pinpos);
                    }
                }
            }
        }
        GPIOx->CFGLR = tmpreg;
    }

    if(GPIO_InitStruct->GPIO_Pin > 0x00FF)
    {
        tmpreg = GPIOx->CFGHR;

        for(pinpos = 0x00; pinpos < 0x08; pinpos++)
        {
            pos = (((uint32_t)0x01) << (pinpos + 0x08));
            currentpin = ((GPIO_InitStruct->GPIO_Pin) & pos);

            if(currentpin == pos)
            {
                pos = pinpos << 2;
                pinmask = ((uint32_t)0x0F) << pos;
                tmpreg &= ~pinmask;
                tmpreg |= (currentmode << pos);

                if(GPIO_InitStruct->GPIO_Mode == GPIO_Mode_IPD)
                {
                    GPIOx->BCR = (((uint32_t)0x01) << (pinpos + 0x08));
                }

                if(GPIO_InitStruct->GPIO_Mode == GPIO_Mode_IPU)
                {
                    GPIOx->BSHR = (((uint32_t)0x01) << (pinpos + 0x08));
                }
            }
        }
        GPIOx->CFGHR = tmpreg;
    }
}

/*********************************************************************
 * @fn      GPIO_StructInit
 *
 * @brief   Fills each GPIO_InitStruct member with its default
 *
 * @param   GPIO_InitStruct - pointer to a GPIO_InitTypeDef structure
 *      which will be initialized.
 *
 * @return  none
 */
void GPIO_StructInit(GPIO_InitTypeDef *GPIO_InitStruct)
{
    GPIO_InitStruct->GPIO_Pin = GPIO_Pin_All;
    GPIO_InitStruct->GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStruct->GPIO_Mode = GPIO_Mode_IN_FLOATING;
}

/*********************************************************************
 * @fn      GPIO_ReadInputDataBit
 *
 * @brief   GPIOx - where x can be (A..D) to select the GPIO peripheral.
 *
 * @param    GPIO_Pin - specifies the port bit to read.
 *             This parameter can be GPIO_Pin_x where x can be (0..15).
 *
 * @return  The input port pin value.
 */
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    uint8_t bitstatus = 0x00;

    if((GPIOx->INDR & GPIO_Pin) != (uint32_t)Bit_RESET)
    {
        bitstatus = (uint8_t)Bit_SET;
    }
    else
    {
        bitstatus = (uint8_t)Bit_RESET;
    }

    return bitstatus;
}

/*********************************************************************
 * @fn      GPIO_ReadInputData
 *
 * @brief   Reads the specified GPIO input data port.
 *
 * @param   GPIOx - where x can be (A..D) to select the GPIO peripheral.
 *
 * @return  The output port pin value.
 */
uint16_t GPIO_ReadInputData(GPIO_TypeDef *GPIOx)
{
    uint16_t val;

    val = ( uint16_t )GPIOx->INDR;

    return ( val );
}

/*********************************************************************
 * @fn      GPIO_ReadOutputDataBit
 *
 * @brief   Reads the specified output data port bit.
 *
 * @param   GPIOx - where x can be (A..D) to select the GPIO peripheral.
 *          GPIO_Pin - specifies the port bit to read.
 *            This parameter can be GPIO_Pin_x where x can be (0..15).
 *
 * @return  none
 */
uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    uint8_t bitstatus = 0x00;

    if((GPIOx->OUTDR & GPIO_Pin) != (uint32_t)Bit_RESET)
    {
        bitstatus = (uint8_t)Bit_SET;
    }
    else
    {
        bitstatus = (uint8_t)Bit_RESET;
    }

    return bitstatus;
}

/*********************************************************************
 * @fn      GPIO_ReadOutputData
 *
 * @brief   Reads the specified GPIO output data port.
 *
 * @param   GPIOx - where x can be (A..D) to select the GPIO peripheral.
 *
 * @return  GPIO output port pin value.
 */
uint16_t GPIO_ReadOutputData(GPIO_TypeDef *GPIOx)
{
    uint16_t val;

    val = ( uint16_t )GPIOx->OUTDR;

    return ( val );
}

/*********************************************************************
 * @fn      GPIO_SetBits
 *
 * @brief   Sets the selected data port bits.
 *
 * @param   GPIOx - where x can be (A..D) to select the GPIO peripheral.
 *          GPIO_Pin - specifies the port bits to be written.
 *            This parameter can be any combination of GPIO_Pin_x where x can be (0..15).
 *
 * @return  none
 */
void GPIO_SetBits(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    GPIOx->BSHR = GPIO_Pin;
}

/*********************************************************************
 * @fn      GPIO_ResetBits
 *
 * @brief   Clears the selected data port bits.
 *
 * @param   GPIOx - where x can be (A..D) to select the GPIO peripheral.
 *          GPIO_Pin - specifies the port bits to be written.
 *            This parameter can be any combination of GPIO_Pin_x where x can be (0..15).
 *
 * @return  none
 */
void GPIO_ResetBits(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    GPIOx->BCR = GPIO_Pin;
}

/*********************************************************************
 * @fn      GPIO_WriteBit
 *
 * @brief   Sets or clears the selected data port bit.
 *
 * @param   GPIO_Pin - specifies the port bit to be written.
 *            This parameter can be one of GPIO_Pin_x where x can be (0..15).
 *          BitVal - specifies the value to be written to the selected bit.
 *            Bit_RESET - to clear the port pin.
 *            Bit_SET - to set the port pin.
 *
 * @return  none
 */
void GPIO_WriteBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, BitAction BitVal)
{
    if(BitVal != Bit_RESET)
    {
        GPIOx->BSHR = GPIO_Pin;
    }
    else
    {
        GPIOx->BCR = GPIO_Pin;
    }
}

/*********************************************************************
 * @fn      GPIO_Write
 *
 * @brief   Writes data to the specified GPIO data port.
 *
 * @param   GPIOx - where x can be (A..D) to select the GPIO peripheral.
 *          PortVal - specifies the value to be written to the port output data register.
 *
 * @return  none
 */
void GPIO_Write(GPIO_TypeDef *GPIOx, uint16_t PortVal)
{
    GPIOx->OUTDR = PortVal;
}

/*********************************************************************
 * @fn      GPIO_PinLockConfig
 *
 * @brief   Locks GPIO Pins configuration registers.
 *
 * @param   GPIOx - where x can be (A..D) to select the GPIO peripheral.
 *          GPIO_Pin - specifies the port bit to be written.
 *            This parameter can be any combination of GPIO_Pin_x where x can be (0..15).
 *
 * @return  none
 */
void GPIO_PinLockConfig(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    uint32_t tmp = 0x00010000;

    tmp |= GPIO_Pin;
    GPIOx->LCKR = tmp;
    GPIOx->LCKR = GPIO_Pin;
    GPIOx->LCKR = tmp;
    tmp = GPIOx->LCKR;
    tmp = GPIOx->LCKR;
}

/*********************************************************************
 * @fn      GPIO_EventOutputConfig
 *
 * @brief   Selects the GPIO pin used as Event output.
 *
 * @param   GPIO_PortSource - selects the GPIO port to be used as source
 *        for Event output.
 *            This parameter can be GPIO_PortSourceGPIOx where x can be (A..D).
 *          GPIO_PinSource - specifies the pin for the Event output.
 *            This parameter can be GPIO_PinSourcex where x can be (0..15).
 *
 * @return  none
 */
void GPIO_EventOutputConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource)
{
    uint32_t tmpreg = 0x00;

    tmpreg = AFIO->ECR;
    tmpreg &= ECR_PORTPINCONFIG_MASK;
    tmpreg |= (uint32_t)GPIO_PortSource << 0x04;
    tmpreg |= GPIO_PinSource;
    AFIO->ECR = tmpreg;
}

/*********************************************************************
 * @fn      GPIO_EventOutputCmd
 *
 * @brief   Enables or disables the Event Output.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void GPIO_EventOutputCmd(FunctionalState NewState)
{
    if(NewState)
    {
        AFIO->ECR |= (1 << 7);
    }
    else
    {
        AFIO->ECR &= ~(1 << 7);
    }
}

/*********************************************************************
 * @fn      GPIO_PinRemapConfig
 *
 * @brief   Changes the mapping of the specified pin.
 *
 * @param   GPIO_Remap - selects the pin to remap.
 *            GPIO_PartialRemap1_SPI1 - SPI1 Partial1 Alternate Function mapping
 *            GPIO_PartialRemap2_SPI1 - SPI1 Partial2 Alternate Function mapping
 *            GPIO_FullRemap_SPI1 - SPI1 Full Alternate Function mapping
 *            GPIO_PartialRemap1_I2C1 - I2C1 Partial1 Alternate Function mapping
 *            GPIO_FullRemap_I2C1 - I2C1 Full Alternate Function mapping
 *            GPIO_PartialRemap1_USART1 - USART1 Partial1 Alternate Function mapping
 *            GPIO_PartialRemap2_USART1 - USART1 Partial2 Alternate Function mapping
 *            GPIO_PartialRemap3_USART1 - USART1 Partial3 Alternate Function mapping
 *            GPIO_PartialRemap4_USART1 - USART1 Partial4 Alternate Function mapping
 *            GPIO_FullRemap_USART1 - USART1 Full Alternate Function mapping
 *            GPIO_PartialRemap1_USART2 - USART2 Partial1 Alternate Function mapping
 *            GPIO_PartialRemap2_USART2 - USART2 Partial2 Alternate Function mapping
 *            GPIO_FullRemap_USART2 - USART2 Full Alternate Function mapping
 *            GPIO_PartialRemap1_TIM1 - TIM1 Partial1 Alternate Function mapping
 *            GPIO_PartialRemap2_TIM1 - TIM1 Partial2 Alternate Function mapping
 *            GPIO_PartialRemap3_TIM1 - TIM1 Partial3 Alternate Function mapping
 *            GPIO_PartialRemap4_TIM1 - TIM1 Partial4 Alternate Function mapping
 *            GPIO_PartialRemap5_TIM1 - TIM1 Partial5 Alternate Function mapping
 *            GPIO_FullRemap_TIM1 - TIM1 Full Alternate Function mapping
 *            GPIO_PartialRemap1_TIM2 - TIM2 Partial1 Alternate Function mapping
 *            GPIO_PartialRemap2_TIM2 - TIM2 Partial2 Alternate Function mapping
 *            GPIO_PartialRemap3_TIM2 - TIM2 Partial3 Alternate Function mapping
 *            GPIO_PartialRemap4_TIM2 - TIM2 Partial4 Alternate Function mapping
 *            GPIO_PartialRemap5_TIM2 - TIM2 Partial5 Alternate Function mapping
 *            GPIO_FullRemap_TIM2 - TIM2 Full Alternate Function mapping
 *            GPIO_PartialRemap_USART3 - USART3 Partial Alternate Function mapping
 *            GPIO_FullRemap_USART3 - USART3 Full Alternate Function mapping
 *            GPIO_Remap_TIM3 - TIM3 Alternate Function mapping
 *            GPIO_Remap_TIM4 - TIM4 Alternate Function mapping
 *            GPIO_Remap1_CAN1 - CAN1 Alternate Function mapping
 *            GPIO_Remap2_CAN1 - CAN1 Alternate Function mapping
 *            GPIO_Remap_PD01 - PD01 Alternate Function mapping
 *            GPIO_Remap_SWJ_Disable - Full SDI Disabled (SDI)
 *            GPIO_Remap_USART4 - USART4 Alternate Function mapping
 *            GPIO_Remap_LPTIM - LPTIM Alternate Function mapping
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void GPIO_PinRemapConfig(uint32_t GPIO_Remap, FunctionalState NewState)
{
    uint32_t tmp = 0x00, tmp1 = 0x00, tmpreg1 = 0x00, tmpreg2 = 0x00, tmpmask = 0x00;

    if((GPIO_Remap & 0xC0000000) == 0xC0000000) /* PCFR1 + PCFR2 */
    {
        tmpreg1 = AFIO->PCFR1;
        tmpreg2 = AFIO->PCFR2;

        /* Clear bit */
        tmp1 = ((GPIO_Remap & REMAP_NUM_MASK) >> 27);

        if(tmp1 == 0)
        {
            tmpreg1 &= ~(1<<0);
            tmpreg2 &= ~(1<<24);
        }
        else if(tmp1 == 1)
        {
            tmpreg1 &= ~(1<<1);
            tmpreg2 &= ~(1<<23);
        }
        else if(tmp1 == 2)
        {
            tmpreg1 &= ~(1<<2);
            tmpreg2 &= ~(3<<19);
        }
        else if(tmp1 == 3)
        {
            tmpreg1 &= ~(1<<3);
            tmpreg2 &= ~(1<<18);
        }
        else if(tmp1 == 4)
        {
            tmpreg1 &= ~(3<<6);
            tmpreg2 &= ~(1<<22);
        }
        else if(tmp1 == 5)
        {
            tmpreg1 &= ~(3<<8);
            tmpreg2 &= ~(1<<21);
        }

        /* Set bit */
        if(NewState != DISABLE)
        {
            tmpreg1 |= (GPIO_Remap & 0x0000FFFF);
            tmpreg2 |= (GPIO_Remap & 0x01FF0000);
        }

        tmpreg1 |= ~DBGAFR_SWJCFG_MASK;

        AFIO->PCFR1 = tmpreg1;
        AFIO->PCFR2 = tmpreg2;
    }
    else if((GPIO_Remap & 0xC0000000) == 0x40000000) /* PCFR2 */
    {
        tmpreg2 = AFIO->PCFR2;

        /* Clear bit */
        tmp1 = ((GPIO_Remap & (~REMAP_MASK)) << 0x10);
        tmpreg2 &= ~tmp1;

        /* Set bit */
        if(NewState != DISABLE)
        {
            tmpreg2 |= tmp1;
        }

        AFIO->PCFR2 = tmpreg2;
    }
    else if((GPIO_Remap & 0xC0000000) == 0x00000000) /* PCFR1 */
    {
        tmpreg1 = AFIO->PCFR1;

        /* Clear bit */
        tmpmask = (GPIO_Remap & DBGAFR_POSITION_MASK) >> 0x10;
        tmp = GPIO_Remap & LSB_MASK;

        if((GPIO_Remap & (DBGAFR_LOCATION_MASK | DBGAFR_NUMBITS_MASK)) == (DBGAFR_LOCATION_MASK | DBGAFR_NUMBITS_MASK)) /* [26:24] 3bit SW_CFG */
        {
            tmpreg1 &= DBGAFR_SWJCFG_MASK;
            AFIO->PCFR1 &= DBGAFR_SWJCFG_MASK;
        }
        else if((GPIO_Remap & DBGAFR_NUMBITS_MASK) == DBGAFR_NUMBITS_MASK) /* [15:0] 2bit */
        {
            tmp1 = ((uint32_t)0x03) << tmpmask;
            tmpreg1 &= ~tmp1;
            tmpreg1 |= ~DBGAFR_SWJCFG_MASK;
        }
        else /* [31:0] 1bit */
        {
            tmpreg1 &= ~(tmp << ((GPIO_Remap >> 0x15) * 0x10));
            tmpreg1 |= ~DBGAFR_SWJCFG_MASK;
        }

        /* Set bit */
        if(NewState != DISABLE)
        {
            tmpreg1 |= (tmp << (((GPIO_Remap & 0x7FFFFFFF )>> 0x15) * 0x10));
        }

        AFIO->PCFR1 = tmpreg1;
    }
}

/*********************************************************************
 * @fn      GPIO_EXTILineConfig
 *
 * @brief   Selects the GPIO pin used as EXTI Line.
 *
 * @param   GPIO_PortSource - selects the GPIO port to be used as source for EXTI lines.
 *            This parameter can be GPIO_PortSourceGPIOx where x can be (A..D).
 *          GPIO_PinSource - specifies the EXTI line to be configured.
 *            This parameter can be GPIO_PinSourcex where x can be (0..15).
 *
 * @return  none
 */
void GPIO_EXTILineConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource)
{
    uint32_t tmp = 0x00;

    tmp = ((uint32_t)0x0F) << (0x04 * (GPIO_PinSource & (uint8_t)0x03));
    AFIO->EXTICR[GPIO_PinSource >> 0x02] &= ~tmp;
    AFIO->EXTICR[GPIO_PinSource >> 0x02] |= (((uint32_t)GPIO_PortSource) << (0x04 * (GPIO_PinSource & (uint8_t)0x03)));
}

/*********************************************************************
 * @fn      GPIO_IPD_Unused
 *
 * @brief   Configure unused GPIO as input pull-down.
 *
 * @param   none
 *
 * @return  none
 */
void GPIO_IPD_Unused(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    uint32_t chip = 0;

    OPA_Trim = (((*(uint32_t *)OPA_TRIM_BASE & 0x0000001F) << 25) | (((*(uint32_t *)OPA_TRIM_BASE & 0x00008000)^0x00008000) << 9)) \
            | (((*(uint32_t *)OPA_TRIM_BASE & 0x001F0000) << 1) | (((*(uint32_t *)OPA_TRIM_BASE & 0x80000000)^0x80000000) >> 15));
    ADC_Trim = (*(uint16_t *)ADC_TRIM_BASE);
    TS_Val = (*(uint32_t *)TS_BASE);
    CHIPID = (*(uint32_t *)CHIPID_BASE);

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOA | RCC_PB2Periph_GPIOB | RCC_PB2Periph_GPIOC |RCC_PB2Periph_GPIOD |RCC_PB2Periph_AFIO,ENABLE);
    chip =  *( uint32_t * )CHIPID_BASE & (~0x000000F0);
    switch(chip)
    {
        case 0x10320700:     //CH32L103K8U6
        {
            GPIO_PinRemapConfig(GPIO_Remap_PD01, ENABLE);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10\
                                          |GPIO_Pin_11|GPIO_Pin_12\
                                          |GPIO_Pin_13|GPIO_Pin_14\
                                          |GPIO_Pin_15;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
            GPIO_Init(GPIOB, &GPIO_InitStructure);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
           GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
            GPIO_Init(GPIOD, &GPIO_InitStructure);
            break;
        }
        case 0x103D0700:     //CH32L103F8U6
        {
             GPIO_PinRemapConfig(GPIO_Remap_PD01, ENABLE);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3\
                                          |GPIO_Pin_4|GPIO_Pin_5\
                                          |GPIO_Pin_8|GPIO_Pin_9\
                                          |GPIO_Pin_12;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
            GPIO_Init(GPIOB, &GPIO_InitStructure);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14\
                                          |GPIO_Pin_15;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
            GPIO_Init(GPIOD, &GPIO_InitStructure);
            break;
        }
        case 0x10370700:     //CH32L103F7P6
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9\
                                          |GPIO_Pin_10|GPIO_Pin_15;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_2\
                                          |GPIO_Pin_3|GPIO_Pin_4\
                                          |GPIO_Pin_5|GPIO_Pin_9\
                                          |GPIO_Pin_10|GPIO_Pin_11\
                                          |GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14\
                                          |GPIO_Pin_15;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
            GPIO_Init(GPIOB, &GPIO_InitStructure);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14\
                                          |GPIO_Pin_15;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
            break;
        }
        case 0x103B0700:     //CH32L103G8R6
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_9;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
            GPIO_Init(GPIOB, &GPIO_InitStructure);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14\
                                          |GPIO_Pin_15;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
            GPIO_Init(GPIOD, &GPIO_InitStructure);
            break;
        }
        case 0x103A0700:     //CH32L103F8P6
        {
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
            GPIO_Init(GPIOA, &GPIO_InitStructure);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_2\
                                          |GPIO_Pin_3|GPIO_Pin_4\
                                          |GPIO_Pin_5|GPIO_Pin_6\
                                          |GPIO_Pin_7 |GPIO_Pin_8\
                                          |GPIO_Pin_9|GPIO_Pin_10\
                                          |GPIO_Pin_11|GPIO_Pin_12;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
            GPIO_Init(GPIOB, &GPIO_InitStructure);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14\
                                          |GPIO_Pin_15;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
            GPIO_Init(GPIOC, &GPIO_InitStructure);
            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
            GPIO_Init(GPIOD, &GPIO_InitStructure);
            break;
        }
        case 0x10310700:     //CH32L103C8T6
        {
            break;
        }
        case 0x10300700:     //CH32L103C8U6
        {
            break;
        }
        default:
        {
            break;
        }

    }
}
