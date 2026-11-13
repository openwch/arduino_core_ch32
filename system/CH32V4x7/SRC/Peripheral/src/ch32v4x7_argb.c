/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32v4x7_argb.c
* Author             : WCH
* Version            : V1.0.1
* Date               : 2026/04/08
* Description        : This file provides all the ARGB firmware functions.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#include "ch32v4x7_argb.h"

/*********************************************************************
 * @fn      ARGB_DeInit
 *
 * @brief   The ARGB_DeInit function resets the ARGB peripheral registers to their default values.
 *
 * @param   none
 *
 * @return  none
 */
void ARGB_DeInit(void)
{
    RCC_HBPeriphResetCmd(RCC_HBPeriph_ARGB, ENABLE);
    RCC_HBPeriphResetCmd(RCC_HBPeriph_ARGB, DISABLE);
}

/*********************************************************************
 * @fn      ARGB_Init
 *
 * @brief   The ARGB_Init function initializes the ARGB peripheral with the specified configuration settings provided in
 *        the ARGB_InitStruct parameter.
 *
 * @param   ARGB_InitStruct - The ARGB_InitStruct is a structure that contains the initialization parameters for
 *        configuring the ARGB peripheral. It includes the following members:
 *
 * @return  none
 */
void ARGB_Init(ARGB_InitTypeDef *ARGB_InitStruct)
{
    ARGB->CTLR = (ARGB->CTLR & ~(ARGB_LSB | ARGB_MODE)) | ARGB_InitStruct->ARGB_Mode | ARGB_InitStruct->ARGB_Endian;
    ARGB->DAT_ARR = ARGB_InitStruct->ARGB_DataPeriod;
    ARGB->RST_ARR = ARGB_InitStruct->ARGB_RSTPeriod;
    ARGB->DAT_CYC = ARGB_InitStruct->ARGB_Length;
    ARGB->CCRH = ARGB_InitStruct->ARGB_T1H;
    ARGB->CCRL = ARGB_InitStruct->ARGB_T0H;
}

/*********************************************************************
 * @fn      ARGB_StructInit
 *
 * @brief   The ARGB_StructInit function fills each member of the ARGB_InitStruct structure with its default value.
 *
 * @param   ARGB_InitStruct - The ARGB_InitStruct is a pointer
 *
 * @return  none
 */
void ARGB_StructInit(ARGB_InitTypeDef *ARGB_InitStruct)
{
    ARGB_InitStruct->ARGB_Length = 0;
    ARGB_InitStruct->ARGB_T1H = 0;
    ARGB_InitStruct->ARGB_T0H = 0;
    ARGB_InitStruct->ARGB_DataPeriod = 0;
    ARGB_InitStruct->ARGB_RSTPeriod = 0;
    ARGB_InitStruct->ARGB_Mode = ARGB_Mode_SendRSTFirst;
    ARGB_InitStruct->ARGB_Endian = ARGB_Endian_MSB;
}

/*********************************************************************
 * @fn      ARGB_Cmd
 *
 * @brief   The ARGB_Cmd function enables or disables the ARGB module based on the input state.
 *
 * @param   NewState - The `NewState` parameter is of type `FunctionalState`, which is an enumeration type that can have
 *        two values: `ENABLE` and `DISABLE`. It is used to enable or disable a specific feature or functionality in the
 * code.
 *
 * @return  none
 */
void ARGB_Cmd(FunctionalState NewState)
{
    if (NewState != DISABLE)
    {
        ARGB->CTLR |= ARGB_TE;
    }
    else
    {
        ARGB->CTLR &= ~ARGB_TE;
    }
}

/*********************************************************************
 * @fn      ARGB_DMACmd
 *
 * @brief   The function ARGB_DMACmd enables or disables the ARGB DMA feature based on the input state.
 *
 * @param   NewState - The `NewState` parameter is of type `FunctionalState`, which is an enumeration type that can have
 *        two values: `ENABLE` and `DISABLE`. It is used to enable or disable a specific feature or functionality in the
 *        code.
 *
 * @return  none
 */
void ARGB_DMACmd(FunctionalState NewState)
{
    if (NewState != DISABLE)
    {
        ARGB->CTLR |= ARGB_DMAEN;
    }
    else
    {
        ARGB->CTLR &= ~ARGB_DMAEN;
    }
}

/*********************************************************************
 * @fn      ARGB_ITConfig
 *
 * @brief   The function ARGB_ITConfig configures an interrupt for an ARGB peripheral based on the specified state.
 *
 * @param   ARGB_IT - Enumeration representing different interrupt
 *            ARGB_IT_FRH - Frame data part circle done interrupt
 *            ARGB_IT_RST - Reset sent interrupt
 *            ARGB_IT_TC - Frame circle done interrupt
 *            ARGB_IT_BYTE - Byte sent interrupt
 *            ARGB_IT_HALF - Half frame data part circle halfdone interrupt
 *            ARGB_IT_TXE - Transmit data register empty interrupt
 *
 * @return  none
 */
void ARGB_ITConfig(ARGB_IT_Typedef ARGB_IT, FunctionalState NewState)
{
    if (NewState != DISABLE)
    {
        ARGB->CTLR |= ARGB_IT;
    }
    else
    {
        ARGB->CTLR &= ~ARGB_IT;
    }
}

/*********************************************************************
 * @fn      ARGB_ClearFlag
 *
 * @brief   The function ARGB_ClearFlag clears the pending bit for a specified interrupt in the ARGB module.
 *
 * @param   ARGB_IT - Enumeration representing different interrupt
 *            ARGB_IT_FRH - Frame data part circle done interrupt
 *            ARGB_IT_RST - Reset sent interrupt
 *            ARGB_IT_TC - Frame circle done interrupt
 *            ARGB_IT_BYTE - Byte sent interrupt
 *            ARGB_IT_HALF - Half frame data part circle halfdone interrupt
 *            ARGB_IT_TXE - Transmit data register empty interrupt
 *
 * @return  none
 */
void ARGB_ClearFlag(ARGB_IT_Typedef ARGB_IT)
{
    ARGB->STATR = (ARGB_IT >> 8);
}

/*********************************************************************
 * @fn      ARGB_GetFlagStatus
 *
 * @brief   The function ARGB_GetFlagStatus checks the status of a specific interrupt flag in a register and returns the
 *        status as either SET or RESET.
 *
 * @param   ARGB_IT - Enumeration representing different interrupt
 *            ARGB_IT_FRH - Frame data part circle done interrupt
 *            ARGB_IT_RST - Reset sent interrupt
 *            ARGB_IT_TC - Frame circle done interrupt
 *            ARGB_IT_BYTE - Byte sent interrupt
 *            ARGB_IT_HALF - Half frame data part circle halfdone interrupt
 *            ARGB_IT_TXE - Transmit data register empty interrupt
 *
 * @return  Wheather the specified interrupt flag is set (SET) or not (RESET).
 */
FlagStatus ARGB_GetFlagStatus(ARGB_IT_Typedef ARGB_IT)
{
    if ((ARGB->STATR & (ARGB_IT >> 8)))
    {
        return SET;
    }
    else
    {
        return RESET;
    }
}

/*********************************************************************
 * @fn      ARGB_ClearITPendingBit
 *
 * @brief   The function ARGB_ClearITPendingBit clears the pending bit for a specified interrupt in the ARGB module.
 *
 * @param   ARGB_IT - Enumeration representing different interrupt
 *            ARGB_IT_FRH - Frame data part circle done interrupt
 *            ARGB_IT_RST - Reset sent interrupt
 *            ARGB_IT_TC - Frame circle done interrupt
 *            ARGB_IT_BYTE - Byte sent interrupt
 *            ARGB_IT_HALF - Half frame data part circle halfdone interrupt
 *            ARGB_IT_TXE - Transmit data register empty interrupt
 *
 * @return  none
 */
void ARGB_ClearITPendingBit(ARGB_IT_Typedef ARGB_IT)
{
    ARGB->STATR = (ARGB_IT >> 8);
}

/*********************************************************************
 * @fn      ARGB_GetITStatus
 *
 * @brief   The function ARGB_GetITStatus checks the status of a specific interrupt flag in a register and returns the
 *        status as either SET or RESET.
 *
 * @param   ARGB_IT - Enumeration representing different interrupt
 *            ARGB_IT_FRH - Frame data part circle done interrupt
 *            ARGB_IT_RST - Reset sent interrupt
 *            ARGB_IT_TC - Frame circle done interrupt
 *            ARGB_IT_BYTE - Byte sent interrupt
 *            ARGB_IT_HALF - Half frame data part circle halfdone interrupt
 *            ARGB_IT_TXE - Transmit data register empty interrupt
 *
 * @return  Wheather the specified interrupt flag is set (SET) or not (RESET).
 */
FlagStatus ARGB_GetITStatus(ARGB_IT_Typedef ARGB_IT)
{
    if ((ARGB->STATR & (ARGB_IT >> 8)) && (ARGB->CTLR & ARGB_IT))
    {
        return SET;
    }
    else
    {
        return RESET;
    }
}

/*********************************************************************
 * @fn      ARGB_GetCurrBaseCNT
 *
 * @brief   The function ARGB_GetCurrBaseCNT returns the current base counter value of the ARGB module.
 *
 * @return  Current base counter value
 */
uint32_t ARGB_GetCurrBaseCNT(void)
{
    return (ARGB->CNTR);
}

/*********************************************************************
 * @fn      ARGB_GetBitCNT
 *
 * @brief   The function ARGB_GetBitCNT returns the current bit counter value of the ARGB module.
 *
 * @return  Current bit counter value
 */
uint32_t ARGB_GetBitCNT(void)
{
    return (ARGB->STATR & (ARGB_BIT_CNT)) >> 8;
}

/*********************************************************************
 * @fn      ARGB_GetByteCNT
 *
 * @brief   The function ARGB_GetByteCNT returns the current byte counter value of the ARGB module.
 *
 * @return  Current byte counter value
 */
uint32_t ARGB_GetByteCNT(void)
{
    return (ARGB->STATR & (ARGB_BYTE_CNT)) >> 16;
}

/*********************************************************************
 * @fn      ARGB_SendData
 *
 * @brief   Transmits single data through the ARGB peripheral.
 *
 * @param   Data - the data to transmit.
 *
 * @return  none
 */
void ARGB_SendData(uint8_t Data)
{
    ARGB->DATAR = Data;
}

/*********************************************************************
 * @fn      ARGB_GetCurrState
 *
 * @brief   Get currently sending status through the ARGB peripheral.
 *
 * @return  The currently sending status, which can be ARGB_Idle, ARGB_SendingData, or ARGB_SendingRST.
 */
ARGB_STATypeDef ARGB_GetCurrState(void)
{
    return ((ARGB->STATR & ARGB_STATE) >> 6);
}