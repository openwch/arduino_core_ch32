/********************************** (C) COPYRIGHT *******************************
 * File Name          : ch32l103_lptim.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/08/29
 * Description        : This file provides all the TIM firmware functions.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "ch32l103_lptim.h"
#include "ch32l103_rcc.h"


/*********************************************************************
 * @fn      LPTIM_DeInit
 *
 * @brief   Deinitializes the LPTIM peripheral registers to their default
 *        reset values.
 *
 * @param   none
 *
 * @return  none
 */
void LPTIM_DeInit(void)
{
    RCC_PB1PeriphResetCmd(RCC_PB1Periph_LPTIM, ENABLE);
    RCC_PB1PeriphResetCmd(RCC_PB1Periph_LPTIM, DISABLE);
}

/*********************************************************************
 * @fn      LPTIM_TimeBaseInit
 *
 * @brief   Initializes the LPTIM Time Base Unit peripheral according to
 *        the specified parameters in the LPTIM_TimeBaseInitStruct.
 *
 * @param   LPTIM_TimeBaseInitStruct - pointer to a LPTIM_TimeBaseInitTypeDef
 *        structure.
 *
 * @return  none
 */
void LPTIM_TimeBaseInit(LPTIM_TimeBaseInitTypeDef *LPTIM_TimeBaseInitStruct)
{
    uint32_t temp1 = 0, temp2 = 0;

    temp2 = (LPTIM->CR & 0x00000001);

    if(LPTIM_TimeBaseInitStruct->LPTIM_Encoder == ENABLE)
    {
        temp1 |= LPTIM_TimeBaseInitStruct->LPTIM_EncoderMode;
    }
    else
    {
        temp1 |= LPTIM_TimeBaseInitStruct->LPTIM_ClockPolarity;
    }

    temp1 |= LPTIM_TimeBaseInitStruct->LPTIM_ClockSource | LPTIM_TimeBaseInitStruct->LPTIM_ClockSampleTime  \
             | LPTIM_TimeBaseInitStruct->LPTIM_TriggerSampleTime | LPTIM_TimeBaseInitStruct->LPTIM_ClockPrescaler  \
             | LPTIM_TimeBaseInitStruct->LPTIM_TriggerSource | LPTIM_TimeBaseInitStruct->LPTIM_ExTriggerPolarity  \
             | LPTIM_TimeBaseInitStruct->LPTIM_OutputPolarity | LPTIM_TimeBaseInitStruct->LPTIM_UpdateMode  \
             | LPTIM_TimeBaseInitStruct->LPTIM_CountSource | LPTIM_TimeBaseInitStruct->LPTIM_InClockSource \
             | (LPTIM_TimeBaseInitStruct->LPTIM_TimeOut << 19) | (LPTIM_TimeBaseInitStruct->LPTIM_OnePulseMode << 20)  \
             | (LPTIM_TimeBaseInitStruct->LPTIM_Encoder << 24) | (LPTIM_TimeBaseInitStruct->LPTIM_ForceOutHigh << 27);

    temp2 |= (LPTIM_TimeBaseInitStruct->LPTIM_SingleMode << 1) | (LPTIM_TimeBaseInitStruct->LPTIM_ContinuousMode << 2)  \
             | (LPTIM_TimeBaseInitStruct->LPTIM_PWMOut << 3) | (LPTIM_TimeBaseInitStruct->LPTIM_CounterDirIndicat << 4);

    LPTIM->CFGR = temp1;
    LPTIM->CR = temp2;
    LPTIM->CMP = LPTIM_TimeBaseInitStruct->LPTIM_Pulse;
    LPTIM->ARR = LPTIM_TimeBaseInitStruct->LPTIM_Period;
}

/*********************************************************************
 * @fn      LPTIM_TimeBaseStructInit
 *
 * @brief   Fills each LPTIM_TimeBaseInitStruct member with its default value.
 *
 * @param   TIM_TimeBaseInitStruct - pointer to a LPTIM_TimeBaseInitTypeDef structure.
 *
 * @return  none
 */
void LPTIM_TimeBaseStructInit(LPTIM_TimeBaseInitTypeDef *LPTIM_TimeBaseInitStruct)
{
    LPTIM_TimeBaseInitStruct->LPTIM_ClockSource = LPTIM_ClockSource_In;
    LPTIM_TimeBaseInitStruct->LPTIM_ClockPolarity = LPTIM_ClockPolarity_Rising;
    LPTIM_TimeBaseInitStruct->LPTIM_ClockSampleTime = LPTIM_ClockSampleTime_0T;
    LPTIM_TimeBaseInitStruct->LPTIM_TriggerSampleTime = LPTIM_TriggerSampleTime_0T;
    LPTIM_TimeBaseInitStruct->LPTIM_ClockPrescaler = LPTIM_TClockPrescaler_DIV1;
    LPTIM_TimeBaseInitStruct->LPTIM_TriggerSource = LPTIM_TriggerSource_ETR;
    LPTIM_TimeBaseInitStruct->LPTIM_ExTriggerPolarity = LPTIM_ExTriggerPolarity_Disable;
    LPTIM_TimeBaseInitStruct->LPTIM_TimeOut = DISABLE;
    LPTIM_TimeBaseInitStruct->LPTIM_OnePulseMode = DISABLE;
    LPTIM_TimeBaseInitStruct->LPTIM_OutputPolarity = LPTIM_OutputPolarity_High;
    LPTIM_TimeBaseInitStruct->LPTIM_UpdateMode = LPTIM_UpdateMode0;
    LPTIM_TimeBaseInitStruct->LPTIM_CountSource = LPTIM_CountSource_Internal;
    LPTIM_TimeBaseInitStruct->LPTIM_Encoder = DISABLE;
    LPTIM_TimeBaseInitStruct->LPTIM_InClockSource = LPTIM_InClockSource_PCLK1;
    LPTIM_TimeBaseInitStruct->LPTIM_ForceOutHigh = DISABLE;
    LPTIM_TimeBaseInitStruct->LPTIM_SingleMode = DISABLE;
    LPTIM_TimeBaseInitStruct->LPTIM_ContinuousMode = DISABLE;
    LPTIM_TimeBaseInitStruct->LPTIM_PWMOut = DISABLE;
    LPTIM_TimeBaseInitStruct->LPTIM_CounterDirIndicat = DISABLE;
    LPTIM_TimeBaseInitStruct->LPTIM_Pulse = 0;
    LPTIM_TimeBaseInitStruct->LPTIM_Period = 0x0001;
}

/*********************************************************************
 * @fn      LPTIM_CounterDirIndicat_Cmd
 *
 * @brief   Enable or Disable counter direction indicate function.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void LPTIM_CounterDirIndicat_Cmd(FunctionalState NewState)
{
    if(NewState)
    {
        LPTIM->CR |= (1<<3);
    }
    else{
        LPTIM->CR &= ~(1<<3);
    }
}

/*********************************************************************
 * @fn      LPTIM_OutCmd
 *
 * @brief   Enable or Disable PWM out function.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void LPTIM_OutCmd(FunctionalState NewState)
{
    if(NewState)
    {
        LPTIM->CR |= (1<<3);
    }
    else{
        LPTIM->CR &= ~(1<<3);
    }
}

/*********************************************************************
 * @fn      LPTIM_Cmd
 *
 * @brief   Enable or Disable LPTIM.
 *
 * @param   NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void LPTIM_Cmd(FunctionalState NewState)
{
    if(NewState)
    {
        LPTIM->CR |= (1<<0);
    }
    else{
        LPTIM->CR &= ~(1<<0);
    }
}

/*********************************************************************
 * @fn      LPTIM_GetCounter
 *
 * @brief   Gets the LPTIM Counter value.
 *
 * @param   none
 *
 * @return  LPTIM->CNT - Counter Register value.
 */
uint16_t LPTIM_GetCounter(void)
{
    return LPTIM->CNT;
}

/*********************************************************************
 * @fn      LPTIM_SetAutoreload
 *
 * @brief   Sets the LPTIM Autoreload Register value.
 *
 * @param   Autoreload - specifies the Autoreload register new value.
 *
 * @return  none
 */
void LPTIM_SetAutoreload(uint16_t Autoreload)
{
    LPTIM->ARR = Autoreload;
}

/*********************************************************************
 * @fn      LPTIM_SetCompare
 *
 * @brief   Sets the LPTIM Capture Compare Register value.
 *
 * @param   Compare - specifies the Capture Compare1 register new value.
 *
 * @return  none
 */
void LPTIM_SetCompare(uint16_t Compare)
{
    LPTIM->CMP = Compare;
}

/*********************************************************************
 * @fn      LPTIM_GetCapture
 *
 * @brief   Gets the LPTIM Input Capture value.
 *
 * @param   none
 *
 * @return  LPTIM->CMP - Capture Compare Register value.
 */
uint16_t LPTIM_GetCapture(void)
{
    return LPTIM->CMP;
}

/*********************************************************************
 * @fn      LPTIM_ITConfig
 *
 * @brief   Enables or disables the specified LPTIM interrupts.
 *
 * @param   LPTIM_IT - specifies the LPTIM interrupts sources to be enabled or disabled.
 *            LPTIM_IT_DOWN - LPTIM counter down Interrupt source.
 *            LPTIM_IT_UP - LPTIM counter up Interrupt source.
 *            LPTIM_IT_ARROK - LPTIM be loaded success Interrupt source.
 *            LPTIM_IT_CMPOK - LPTIM Capture Compare success Interrupt source.
 *            LPTIM_IT_EXTTRIG - TIM Trigger Interrupt source.
 *            LPTIM_IT_ARRM - TIM counter Count to ARR register Interrupt source.
 *            LPTIM_IT_CMPM - TIM counter Count to CMP register Interrupt source.
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void LPTIM_ITConfig(uint32_t LPTIM_IT, FunctionalState NewState)
{
    if(NewState != DISABLE)
    {
        LPTIM->IER |= LPTIM_IT;
    }
    else
    {
        LPTIM->IER &= (uint32_t)~LPTIM_IT;
    }
}

/*********************************************************************
 * @fn      LPTIM_GetFlagStatus
 *
 * @brief   Checks whether the specified LPTIM flag is set or not.
 *
 * @param   LPTIM_FLAG - specifies the flag to check.
 *            LPTIM_FLAG_DIR_SYNC - LPTIM counter direction indicate Flag.
 *            LPTIM_FLAG_DOWN - LPTIM counter down Flag.
 *            LPTIM_FLAG_UP - LPTIM counter up Flag.
 *            LPTIM_FLAG_ARROK - LPTIM be loaded success Flag.
 *            LPTIM_FLAG_CMPOK - LPTIM Capture Compare success Flag.
 *            LPTIM_FLAG_EXTTRIG - TIM Trigger Flag.
 *            LPTIM_FLAG_ARRM - TIM counter Count to ARR register Flag.
 *            LPTIM_FLAG_CMPM - TIM counter Count to CMP register Flag.
 *
 * @return  none
 */
FlagStatus LPTIM_GetFlagStatus(uint32_t LPTIM_FLAG)
{
    FlagStatus bitstatus = RESET;

    if((LPTIM->ISR & LPTIM_FLAG) != (uint32_t)RESET)
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
 * @fn      LPTIM_ClearFlag
 *
 * @brief   Clears the LPTIM's pending flags.
 *
 * @param   LPTIM_FLAG - specifies the flag to check.
 *            LPTIM_FLAG_DOWN - LPTIM counter down Flag.
 *            LPTIM_FLAG_UP - LPTIM counter up Flag.
 *            LPTIM_FLAG_ARROK - LPTIM be loaded success Flag.
 *            LPTIM_FLAG_CMPOK - LPTIM Capture Compare success Flag.
 *            LPTIM_FLAG_EXTTRIG - TIM Trigger Flag.
 *            LPTIM_FLAG_ARRM - TIM counter Count to ARR register Flag.
 *            LPTIM_FLAG_CMPM - TIM counter Count to CMP register Flag.
 *
 * @return  none
 */
void LPTIM_ClearFlag(uint32_t LPTIM_FLAG)
{
    LPTIM->ICR |= (uint32_t)LPTIM_FLAG;
}

/*********************************************************************
 * @fn      LPTIM_GetITStatus
 *
 * @brief   Checks whether the LPTIM interrupt has occurred or not.
 *
 * @param   LPTIM_IT - specifies the LPTIM interrupt source to check.
 *            LPTIM_FLAG_DIR_SYNC - LPTIM counter direction indicate Interrupt source.
 *            LPTIM_IT_DOWN - LPTIM counter down Interrupt source.
 *            LPTIM_IT_UP - LPTIM counter up Interrupt source.
 *            LPTIM_IT_ARROK - LPTIM be loaded success Interrupt source.
 *            LPTIM_IT_CMPOK - LPTIM Capture Compare success Interrupt source.
 *            LPTIM_IT_EXTTRIG - TIM Trigger Interrupt source.
 *            LPTIM_IT_ARRM - TIM counter Count to ARR register Interrupt source.
 *            LPTIM_IT_CMPM - TIM counter Count to CMP register Interrupt source.
 *
 * @return  none
 */
ITStatus LPTIM_GetITStatus(uint32_t LPTIM_IT)
{
    ITStatus bitstatus = RESET;

    if((LPTIM->ISR & LPTIM_IT) != (uint32_t)RESET)
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
 * @fn      LPTIM_ClearITPendingBit
 *
 * @brief   Clears the LPTIM's interrupt pending bits.
 *
 * @param   LPTIM_IT - specifies the LPTIM interrupt source to check.
 *            LPTIM_IT_DOWN - LPTIM counter down Interrupt source.
 *            LPTIM_IT_UP - LPTIM counter up Interrupt source.
 *            LPTIM_IT_ARROK - LPTIM be loaded success Interrupt source.
 *            LPTIM_IT_CMPOK - LPTIM Capture Compare success Interrupt source.
 *            LPTIM_IT_EXTTRIG - TIM Trigger Interrupt source.
 *            LPTIM_IT_ARRM - TIM counter Count to ARR register Interrupt source.
 *            LPTIM_IT_CMPM - TIM counter Count to CMP register Interrupt source.
 *
 * @return  none
 */
void LPTIM_ClearITPendingBit(uint32_t LPTIM_IT)
{
    LPTIM->ICR |= (uint32_t)LPTIM_IT;
}



