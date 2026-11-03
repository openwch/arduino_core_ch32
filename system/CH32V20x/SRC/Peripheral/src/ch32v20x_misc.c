/********************************** (C) COPYRIGHT *******************************
 * File Name          : ch32v20x_misc.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2021/06/06
 * Description        : This file provides all the miscellaneous firmware functions .
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *********************************************************************************/
#include "ch32v20x_misc.h"

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


/* MxRD 261030: moved from startup_ch32v20x.S to ch32v00X_misc.c (see issue openwch#204, PR ch32-community#85) 
   V20x has multiple startup_... assembly files, selection is made in ch32_def_build.h
         defined(CH32V203):   startup_ch32v20x_D6.S  - define CH32V20x_D6 - for V203F6, etc
         defined(CH32V203xB): startup_ch32v20x_D8.S  - define CH32V20x_D8 - 
         defined(CH32V208):   startup_ch32v20x_D8W.S - define CH32V20x_D8W
   V20x D8 adds Ethernet, TIM5, and oscillator handlers. D8W also adds BB_IRQHandler and LLE_IRQHandler.

   system/.../Peripheral/inc/ch32v20x.h defines _D6 as default:
        #define CH32V20x_D6     // CH32V203F6-CH32V203F8-CH32V203G6-CH32V203G8-CH32V203K6-CH32V203K8-CH32V203C6-CH32V203C8
        //#define CH32V20x_D8   // CH32V203RBT6
        //#define CH32V20x_D8W  // CH32V208
   Currently it seems the only properly supported branch is CH32V20x_D6 and since the build.txt doesn't selects _D8, 
   it's (incorrectly) also used for CH32V203RBT6.
   TODO: defines CH32V20x_D8 and CH32V20x_D8W are currently not set by the board selection!
*/

#if defined(CH32V20x_D6) || defined(CH32V20x_D8) ||defined(CH32V20x_D8W)
//#warning D6-D8-D8W handlers
__attribute__ ((weak)) void while1_handler(void) {while(1);};
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
__attribute__ ((weak)) void USBWakeUp_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void USBHD_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void USBHDWakeUp_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void UART4_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void DMA1_Channel8_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));

/*
  // these are already implemented in the core
  .weak   NMI_Handler
  .weak   HardFault_Handler
  .weak   SysTick_Handler
  
  .weak   Ecall_M_Mode_Handler
  .weak   Ecall_U_Mode_Handler
*/
#endif // defined(CH32V20x_D6/D8/D8W)
#if defined(CH32V20x_D8) || defined(CH32V20x_D8W)
  // V20x D8 adds Ethernet, TIM5, and oscillator handlers. 
  // TODO: defines CH32V20x_D8 and CH32V20x_D8W are currently not set by the board selection!
  #warning D8-D8W handlers for Ethernet, TIM5, and oscillator are included.
__attribute__ ((weak)) void ETH_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void ETHWakeUp_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void TIM5_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void OSC32KCal_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void OSCWakeUp_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
#endif // defined(CH32V20x_D8/D8W)
#if defined(CH32V20x_D8W)
  // V20x D8W also adds BB_IRQHandler and LLE_IRQHandler.
  // TODO: defines CH32V20x_D8 and CH32V20x_D8W are currently not set by the board selection!
  #warning D8W handlers BB_IRQHandler and LLE_IRQHandler are included.
__attribute__ ((weak)) void BB_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
__attribute__ ((weak)) void LLE_IRQHandler (void) __attribute__ ((weak, alias ("while1_handler")));
#endif // defined(CH32V20x_D8W)
