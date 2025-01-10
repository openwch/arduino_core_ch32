/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32l103_opa.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/11/05
 * Description        : This file provides all the OPA firmware functions.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "ch32l103_opa.h"

/* FLASH Keys */
#define OPCM_KEY1                 ((uint32_t)0x45670123)
#define OPCM_KEY2                 ((uint32_t)0xCDEF89AB)

/* mask definition*/
#define POLL_CNT_MASK             ((uint16_t)0x7000)

volatile uint32_t CTLR2_tmp = 0;


/********************************************************************************
 * @fn      OPCM_Unlock
 *
 * @brief   Unlocks the OPCM Controller.
 *
 * @return  none
 */
void OPCM_Unlock(void)
{
    OPA->OPCMKEY = OPCM_KEY1;
    OPA->OPCMKEY = OPCM_KEY2;
}

/********************************************************************************
 * @fn      OPCM_Lock
 *
 * @brief   Locks the OPCM Controller.
 *
 * @return  none
 */
void OPCM_Lock(void)
{
    OPA->CTLR1 |= (1<<7);
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
    uint16_t tmp0 = 0, tmp1 = 0;
    uint32_t tmp2 = 0;

    tmp0 = OPA->CFGR1;
    tmp1 = OPA->CFGR2;
    tmp2 = OPA->CTLR1;

    if(OPA_InitStruct->OPA_NUM == OPA1)
    {
        tmp1 &= 0xF800;
        tmp2 &= 0x80000001;

        tmp1 |= (OPA_InitStruct->POLL_NUM << 9);
        tmp2 |= (OPA_InitStruct->Mode << 1) | (OPA_InitStruct->PSEL << 4) \
                | (OPA_InitStruct->FB << 7) | (OPA_InitStruct->NSEL << 8) \
                | OPA_Trim;
    }

    tmp0 |= (OPA_InitStruct->PSEL_POLL) | (OPA_InitStruct->BKIN_EN << 2)
                     | (OPA_InitStruct->RST_EN << 4) | (OPA_InitStruct->OUT_IE << 8)
                     | (OPA_InitStruct->CNT_IE << 10) | (OPA_InitStruct->NMI_IE << 11);
    tmp1 |= OPA_InitStruct->OPA_POLL_Interval;

    OPA->CFGR1 = tmp0;
    OPA->CFGR2 = tmp1;
    OPA->CTLR1 = tmp2;
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
    OPA_InitStruct->OPA_POLL_Interval = 0;
    OPA_InitStruct->OPA_NUM = OPA1;
    OPA_InitStruct->Mode = OUT_IO_OFF;
    OPA_InitStruct->PSEL = CHP_OFF;
    OPA_InitStruct->FB = FB_OFF;
    OPA_InitStruct->NSEL = CHN_OFF;
    OPA_InitStruct->PSEL_POLL = CHP_OPA1_OFF;
    OPA_InitStruct->BKIN_EN = BKIN_OPA1_OFF;
    OPA_InitStruct->RST_EN = RST_OPA1_OFF;
    OPA_InitStruct->OUT_IE = OUT_IE_OPA1_OFF;
    OPA_InitStruct->CNT_IE = CNT_IE_OFF;
    OPA_InitStruct->NMI_IE = NMI_IE_OFF;
    OPA_InitStruct->POLL_NUM = CHP_POLL_NUM_1;
}

/*********************************************************************
 * @fn      OPA_Cmd
 *
 * @brief   Enables or disables the specified OPA peripheral.
 *
 * @param   OPA_NUM - Select OPA
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void OPA_Cmd(OPA_Num_TypeDef OPA_NUM, FunctionalState NewState)
{
    if(NewState == ENABLE)
    {
        OPA->CTLR1 |= (uint32_t)(1 << (OPA_NUM*16));
    }
    else
    {
        OPA->CTLR1 &= ~(uint32_t)(1 << (OPA_NUM*16));
    }
}

/*********************************************************************
 * @fn      OPA_LP_Cmd
 *
 * @brief   Enables or disables the OPA enter low power mode.
 *
 * @param   NewState - new state of the OPA enter low power mode
 *        (ENABLE or DISABLE).
 *
 * @return  none
 */
void OPA_LP_Cmd(FunctionalState NewState)
{
    if(NewState)
    {
        OPA->CTLR1 |= (1 << 12);
    }
    else
    {
        OPA->CTLR1 &= ~(1 << 12);
    }
}

/*********************************************************************
 * @fn      OPA_CMP_Init
 *
 * @brief   Initializes the CMP peripheral according to the specified
 *        parameters in the CMP_InitTypeDef.
 *
 * @param   CMP_InitStruct - pointer to a CMP_InitTypeDef structure
 *
 * @return  none
 */
void OPA_CMP_Init(CMP_InitTypeDef *CMP_InitStruct)
{
    uint32_t tmp1 = 0;

    tmp1 = CTLR2_tmp;

    if(CMP_InitStruct->CMP_NUM == CMP1)
    {
        tmp1 &= 0xFFFFFFC1;
        tmp1 |= (CMP_InitStruct->Mode << 1) | (CMP_InitStruct->NSEL << 3)
                | (CMP_InitStruct->PSEL << 4) | (CMP_InitStruct->HYEN <<5);
    }
    else if(CMP_InitStruct->CMP_NUM == CMP2)
    {
        tmp1 &= 0xFFFFC1FF;
        tmp1 |= (CMP_InitStruct->Mode << 9) | (CMP_InitStruct->NSEL << 11)
                | (CMP_InitStruct->PSEL << 12) | (CMP_InitStruct->HYEN <<13);
    }
    else if(CMP_InitStruct->CMP_NUM == CMP3)
    {
        tmp1 &= 0xFFC1FFFF;
        tmp1 |= (CMP_InitStruct->Mode << 17) | (CMP_InitStruct->NSEL << 19)
                | (CMP_InitStruct->PSEL << 20) | (CMP_InitStruct->HYEN <<21);
    }

    CTLR2_tmp = tmp1;
    OPA->CTLR2 = tmp1;
}

/*********************************************************************
 * @fn      OPA_CMP_StructInit
 *
 * @brief   Fills each OPA_CMP_StructInit member with its reset value.
 *
 * @param   CMP_StructInit - pointer to a OPA_CMP_StructInit structure
 *
 * @return  none
 */
void OPA_CMP_StructInit(CMP_InitTypeDef *CMP_InitStruct)
{
    CMP_InitStruct->CMP_NUM = CMP1;
    CMP_InitStruct->Mode = OUT_IO0;
    CMP_InitStruct->NSEL = CMP_CHN0;
    CMP_InitStruct->PSEL = CMP_CHP_0;
}

/*********************************************************************
 * @fn      OPA_CMP_Cmd
 *
 * @brief   Enables or disables the specified CMP peripheral.
 *
 * @param   CMP_NUM - Select CMP
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void OPA_CMP_Cmd(CMP_Num_TypeDef CMP_NUM, FunctionalState NewState)
{
    if(NewState == ENABLE)
    {
        CTLR2_tmp |= (uint32_t)(1 << (CMP_NUM*8));
    }
    else
    {
        CTLR2_tmp &= ~(uint32_t)(1 << (CMP_NUM*8));
    }

    OPA->CTLR2 = CTLR2_tmp;
}

/*********************************************************************
 * @fn      OPA_CMP_LP_Cmd
 *
 * @brief   Enables or disables the CMP enter low power mode.
 *
 * @param   CMP_NUM - Select CMP
 *          NewState - new state of the CMP enter low power mode
 *        (ENABLE or DISABLE).
 *
 * @return  none
 */
void OPA_CMP_LP_Cmd(CMP_Num_TypeDef CMP_NUM, FunctionalState NewState)
{
    uint8_t tmp1 = 0;

    tmp1 = 6 + CMP_NUM * 8;

    if(NewState)
    {
        OPA->CTLR2 |= (1 << tmp1);
    }
    else
    {
        OPA->CTLR2 &= ~(1 << tmp1);
    }
}

/*********************************************************************
 * @fn      OPA_CMP_WakeUp_ModeConfig
 *
 * @brief   Configures the CMP wake up Mode.
 *
 * @param   CMP_WakeUP_Mode -  Specifies the trigger signal active edge for wake up of the CMP.
 *            CMP_WakeUp_Rising_Falling -  the trigger signal rise and fall edge for wake up.
 *            CMP_WakeUp_Rising - the trigger signal rise edge for wake up.
 *            CMP_WakeUp_Falling - the trigger signal fall edge for wake up.
 *
 * @return  none
 */
void OPA_CMP_WakeUp_ModeConfig(uint32_t CMP_WakeUP_Mode)
{
    OPA->CTLR2 &= ~CMP_WakeUp_Falling;
    OPA->CTLR2 |= CMP_WakeUP_Mode;
}

/*********************************************************************
 * @fn      OPA_GetFlagStatus
 *
 * @brief   Checks whether the OPA flag is set or not.
 *
 * @param   OPA_FLAG - specifies the OPA flag to check.
 *            OPA_FLAG_OUT_OPA1 - OPA1 out flag
 *            OPA_FLAG_OUT_CNT - OPA out flag rising edge of sampling data
 *
 * @return  FlagStatus - SET or RESET.
 */
FlagStatus OPA_GetFlagStatus(uint16_t OPA_FLAG)
{
    FlagStatus bitstatus = RESET;

    if((OPA->CFGR1 & OPA_FLAG) != (uint16_t)RESET)
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
 * @fn      OPA_ClearFlag
 *
 * @brief   Clears the OPA flag.
 *
 * @param   OPA_FLAG - specifies the OPA flag to clear.
 *            OPA_FLAG_OUT_OPA1 - OPA1 out flag
 *            OPA_FLAG_OUT_CNT - OPA out flag rising edge of sampling data
 *
 * @return  none
 */
void OPA_ClearFlag(uint16_t OPA_FLAG)
{
    OPA->CFGR1 &= (uint16_t)~OPA_FLAG;
}

/*********************************************************************
 * @fn      OPA_POLL_CNT
 *
 * @brief   Displays the current channel being polled by the OPA
 *
 * @param   none
 *
 * @return  OPA_POLL_NUM_TypeDef - Current channel for OPA polling
 */
OPA_POLL_NUM_TypeDef OPA_POLL_CNT(void)
{
    uint16_t tmp1 = 0;
    tmp1 = OPA->CFGR2;
    tmp1 &= POLL_CNT_MASK;
    tmp1 = tmp1 >> 12;
    return tmp1;
}
