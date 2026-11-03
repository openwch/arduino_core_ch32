/********************************** (C) COPYRIGHT *******************************
 * File Name          : ch32v00X_misc.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/01/01
 * Description        : This file provides all the miscellaneous firmware functions .
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include <ch32v00X_misc.h>

__IO uint32_t NVIC_Priority_Group = 0;

/*********************************************************************
 * @fn      NVIC_PriorityGroupConfig
 *
 * @brief   Configures the priority grouping - pre-emption priority and subpriority.
 *
 * @param   NVIC_PriorityGroup - specifies the priority grouping bits length.
 *            NVIC_PriorityGroup_0 - 0 bits for pre-emption priority
 *                                   2 bits for subpriority
 *            NVIC_PriorityGroup_1 - 1 bits for pre-emption priority
 *                                   1 bits for subpriority
 *
 * @return  none
 */
void NVIC_PriorityGroupConfig(uint32_t NVIC_PriorityGroup)
{
    NVIC_Priority_Group = NVIC_PriorityGroup;
}

/*********************************************************************
 * @fn      NVIC_Init
 *
 * @brief   Initializes the NVIC peripheral according to the specified parameters in
 *        the NVIC_InitStruct.
 *
 * @param   NVIC_InitStruct - pointer to a NVIC_InitTypeDef structure that contains the
 *        configuration information for the specified NVIC peripheral.
 *              interrupt nesting enable(CSR-0x804 bit1 = 1)
 *            NVIC_IRQChannelPreemptionPriority - range from 0 to 1.
 *            NVIC_IRQChannelSubPriority - range from 0 to 1.
 *
 *              interrupt nesting disable(CSR-0x804 bit1 = 0)
 *            NVIC_IRQChannelPreemptionPriority - range is 0.
 *            NVIC_IRQChannelSubPriority - range from 0 to 3.
 *
 * @return  none
 */
void NVIC_Init(NVIC_InitTypeDef *NVIC_InitStruct)
{
#if (INTSYSCR_INEST == INTSYSCR_INEST_NoEN)
    if(NVIC_Priority_Group == NVIC_PriorityGroup_0)
    {
        NVIC_SetPriority(NVIC_InitStruct->NVIC_IRQChannel, NVIC_InitStruct->NVIC_IRQChannelSubPriority << 6);
    }
#else
    if(NVIC_Priority_Group == NVIC_PriorityGroup_1)
    {
        if(NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority == 1)
        {
            NVIC_SetPriority(NVIC_InitStruct->NVIC_IRQChannel, (1 << 7) | (NVIC_InitStruct->NVIC_IRQChannelSubPriority << 6));
        }
        else if(NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority == 0)
        {
            NVIC_SetPriority(NVIC_InitStruct->NVIC_IRQChannel, (0 << 7) | (NVIC_InitStruct->NVIC_IRQChannelSubPriority << 6));
        }
    }
#endif

    if(NVIC_InitStruct->NVIC_IRQChannelCmd != DISABLE)
    {
        NVIC_EnableIRQ(NVIC_InitStruct->NVIC_IRQChannel);
    }
    else
    {
        NVIC_DisableIRQ(NVIC_InitStruct->NVIC_IRQChannel);
    }
}


/* MMOLE 250727: moved from startup_ch32v00X.S to ch32v00X_misc.c (see issue #204) */
__attribute__ ((weak)) void while1_handler(void) {while(1);};
__attribute__ ((weak)) void SW_Handler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void WWDG_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void PVD_IRQHandler(void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void FLASH_IRQHandler(void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void RCC_IRQHandler(void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void EXTI7_0_IRQHandler(void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void AWU_IRQHandler(void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void DMA1_Channel1_IRQHandler(void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void DMA1_Channel2_IRQHandler(void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void DMA1_Channel3_IRQHandler(void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void DMA1_Channel4_IRQHandler(void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void DMA1_Channel5_IRQHandler(void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void DMA1_Channel6_IRQHandler(void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void DMA1_Channel7_IRQHandler(void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void ADC1_IRQHandler(void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void I2C1_EV_IRQHandler(void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void I2C1_ER_IRQHandler(void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void USART1_IRQHandler(void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void SPI1_IRQHandler(void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void TIM1_BRK_IRQHandler(void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void TIM1_UP_IRQHandler(void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void TIM1_TRG_COM_IRQHandler(void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void TIM1_CC_IRQHandler(void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void TIM2_IRQHandler(void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void USART2_IRQHandler(void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void OPCM_IRQHandler(void) __attribute__ ((weak, alias ("while1_handler")));

/*
	.weak   NMI_Handler
	.weak   HardFault_Handler
	.weak   SysTick_Handler
*/