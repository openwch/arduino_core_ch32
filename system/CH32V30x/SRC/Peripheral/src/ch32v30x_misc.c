/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32v30x_misc.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : This file provides all the miscellaneous firmware functions .
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32v30x_misc.h"

__IO uint32_t NVIC_Priority_Group = 0;

/*********************************************************************
 * @fn      NVIC_PriorityGroupConfig
 *
 * @brief   Configures the priority grouping - pre-emption priority and subpriority.
 *
 * @param   NVIC_PriorityGroup - specifies the priority grouping bits length.
 *            NVIC_PriorityGroup_0 - 0 bits for pre-emption priority
 *                                   4 bits for subpriority
 *            NVIC_PriorityGroup_1 - 1 bits for pre-emption priority
 *                                   3 bits for subpriority
 *            NVIC_PriorityGroup_2 - 2 bits for pre-emption priority
 *                                   2 bits for subpriority
 *            NVIC_PriorityGroup_3 - 3 bits for pre-emption priority
 *                                   1 bits for subpriority
 *            NVIC_PriorityGroup_4 - 4 bits for pre-emption priority
 *                                   0 bits for subpriority
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
 *
 * @return  none
 */
void NVIC_Init(NVIC_InitTypeDef *NVIC_InitStruct)
{
    uint8_t tmppre = 0;

    if(NVIC_Priority_Group == NVIC_PriorityGroup_0)
    {
        NVIC_SetPriority(NVIC_InitStruct->NVIC_IRQChannel, NVIC_InitStruct->NVIC_IRQChannelSubPriority << 4);
    }
    else if(NVIC_Priority_Group == NVIC_PriorityGroup_1)
    {
        if(NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority == 1)
        {
            NVIC_SetPriority(NVIC_InitStruct->NVIC_IRQChannel, (1 << 7) | (NVIC_InitStruct->NVIC_IRQChannelSubPriority << 4));
        }
        else
        {
            NVIC_SetPriority(NVIC_InitStruct->NVIC_IRQChannel, (0 << 7) | (NVIC_InitStruct->NVIC_IRQChannelSubPriority << 4));
        }
    }
    else if(NVIC_Priority_Group == NVIC_PriorityGroup_2)
    {
        if(NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority <= 1)
        {
            tmppre = NVIC_InitStruct->NVIC_IRQChannelSubPriority + (4 * NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority);
            NVIC_SetPriority(NVIC_InitStruct->NVIC_IRQChannel, (0 << 7) | (tmppre << 4));
        }
        else
        {
            tmppre = NVIC_InitStruct->NVIC_IRQChannelSubPriority + (4 * (NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority - 2));
            NVIC_SetPriority(NVIC_InitStruct->NVIC_IRQChannel, (1 << 7) | (tmppre << 4));
        }
    }
    else if(NVIC_Priority_Group == NVIC_PriorityGroup_3)
    {
        if(NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority <= 3)
        {
            tmppre = NVIC_InitStruct->NVIC_IRQChannelSubPriority + (2 * NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority);
            NVIC_SetPriority(NVIC_InitStruct->NVIC_IRQChannel, (0 << 7) | (tmppre << 4));
        }
        else
        {
            tmppre = NVIC_InitStruct->NVIC_IRQChannelSubPriority + (2 * (NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority - 4));
            NVIC_SetPriority(NVIC_InitStruct->NVIC_IRQChannel, (1 << 7) | (tmppre << 4));
        }
    }
    else if(NVIC_Priority_Group == NVIC_PriorityGroup_4)
    {
        NVIC_SetPriority(NVIC_InitStruct->NVIC_IRQChannel, NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority << 4);
    }

    if(NVIC_InitStruct->NVIC_IRQChannelCmd != DISABLE)
    {
        NVIC_EnableIRQ(NVIC_InitStruct->NVIC_IRQChannel);
    }
    else
    {
        NVIC_DisableIRQ(NVIC_InitStruct->NVIC_IRQChannel);
    }
}

/* MxRD 2611102: moved from startup_ch32v30x.S to ch32v30x_misc.c (see issue openwch#204, PR ch32-community#85) 
   CH32V30x has two startup assembly files: "startup_ch32v30x_D8.S" and "startup_ch32v30x_D8C.S".
   The D8C version is for CH32V307x-CH32V305x, the D8 is for CH32V307x-CH32V305x.
   The choice is made depending on a definition: CH32V30x_C or CH32V30x.
   The D9C version has additional handlers.
*/
__attribute__ ((weak)) void while1_handler(void) {while(1);};
__attribute__ ((weak)) void Ecall_M_Mode_Handler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void Ecall_U_Mode_Handler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void Break_Point_Handler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void SW_Handler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void WWDG_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void PVD_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void TAMPER_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void RTC_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void FLASH_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void RCC_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void EXTI0_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void EXTI1_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void EXTI2_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void EXTI3_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void EXTI4_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void DMA1_Channel1_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void DMA1_Channel2_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void DMA1_Channel3_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void DMA1_Channel4_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void DMA1_Channel5_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void DMA1_Channel6_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void DMA1_Channel7_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void ADC1_2_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void USB_HP_CAN1_TX_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void USB_LP_CAN1_RX0_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void CAN1_RX1_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void CAN1_SCE_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void EXTI9_5_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void TIM1_BRK_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void TIM1_UP_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void TIM1_TRG_COM_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void TIM1_CC_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void TIM2_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void TIM3_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void TIM4_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void I2C1_EV_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void I2C1_ER_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void I2C2_EV_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void I2C2_ER_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void SPI1_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void SPI2_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void USART1_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void USART2_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void USART3_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void EXTI15_10_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void RTCAlarm_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void TIM8_BRK_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void TIM8_UP_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void TIM8_TRG_COM_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void TIM8_CC_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void RNG_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void FSMC_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void SDIO_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void TIM5_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void SPI3_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void UART4_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void UART5_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void TIM6_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void TIM7_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void DMA2_Channel1_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void DMA2_Channel2_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void DMA2_Channel3_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void DMA2_Channel4_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void DMA2_Channel5_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void OTG_FS_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void UART6_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void UART7_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void UART8_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void TIM9_BRK_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void TIM9_UP_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void TIM9_TRG_COM_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void TIM9_CC_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void TIM10_BRK_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void TIM10_UP_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void TIM10_TRG_COM_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void TIM10_CC_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void DMA2_Channel6_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void DMA2_Channel7_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void DMA2_Channel8_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void DMA2_Channel9_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void DMA2_Channel10_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void DMA2_Channel11_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));

#if defined(CH32V30x_C)
__attribute__ ((weak)) void USBWakeUp_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void ETH_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void ETH_WKUP_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void CAN2_TX_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void CAN2_RX0_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void CAN2_RX1_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void CAN2_SCE_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void USBHSWakeup_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void USBHS_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void DVP_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
#endif // #if defined(CH32V30x_C)\

/*

// No default handlers are needed for handlers defined elsewhere.
// NMI_Handler and HardFault_Handler are defined in system/CH32V30x/USER/ch32V30x_it.c
// SysTick_Handler is defined in cores/arduino/ch32/clock.c
// Note: Unlike V20x, the V30x has no definition for Ecall_M_Mode_Handler and Ecall_U_Mode_Handler
NMI_Handler:  1:  j 1b
HardFault_Handler:  1:  j 1b
SysTick_Handler:  1:  j 1b

*/