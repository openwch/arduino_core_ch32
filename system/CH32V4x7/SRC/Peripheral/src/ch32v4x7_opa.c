/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32v4x7_opa.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/12/01
* Description        : This file provides all the OPA firmware functions.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32v4x7_opa.h"

#define OPA_MASK         ((uint32_t)0x00FE)


/*********************************************************************
 * @fn      OPA_DeInit
 *
 * @brief   Deinitializes the OPA peripheral registers to their default
 *        reset values.
 *
 * @return  none
 */
void OPA_DeInit(void)
{
    OPA->CR = 0;
}

/*********************************************************************
 * @fn      OPA_Init
 *
 * @brief   Initializes the OPA peripheral according to the specified
 *        parameters in the OPA_InitStruct.
 *
 * @param   OPA_InitStruct - pointer to a OPA_InitTypeDef structure
 *
 * @return  none
 */
void OPA_Init(OPA_InitTypeDef *OPA_InitStruct)
{
    uint32_t tmp = 0;
    tmp = OPA->CR;
    tmp &= ~(OPA_MASK);
    tmp |= ((OPA_InitStruct->PSEL <<5) | (OPA_InitStruct->NSEL << 2) | (OPA_InitStruct->Mode << 1)| 
             (OPA_InitStruct->FB <<7)|(OPA_InitStruct->HS <<8)|(OPA_InitStruct->PGADIF <<6));
    OPA->CR = tmp;
}

/*********************************************************************
 * @fn      OPA_StructInit
 *
 * @brief   Fills each OPA_StructInit member with its reset value.
 *
 * @param   OPA_StructInit - pointer to a OPA_InitTypeDef structure
 *
 * @return  none
 */
void OPA_StructInit(OPA_InitTypeDef *OPA_InitStruct)
{
    OPA_InitStruct->PSEL = CHP0;
    OPA_InitStruct->NSEL = CHN0;
    OPA_InitStruct->Mode = OUT_IO_OUT0;
    OPA_InitStruct->FB = FB_OFF;
    OPA_InitStruct->HS = HS_OFF;
    OPA_InitStruct->PGADIF = PGADIF_GND;
}

/*********************************************************************
 * @fn      OPA_Cmd
 *
 * @brief   Enables or disables the specified OPA peripheral.
 *
 * @param   OPA_NUM - Select OPA
 *            NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void OPA_Cmd(FunctionalState NewState)
{
    if(NewState == ENABLE)
    {
        OPA->CR |= (1 << 0);
    }
    else
    {
        OPA->CR &= ~(1 << 0);
    }
}
