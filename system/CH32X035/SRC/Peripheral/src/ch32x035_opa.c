/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32x035_opa.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/04/06
 * Description        : This file provides all the OPA firmware functions.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32x035_opa.h"


/* FLASH Keys */
#define OPA_KEY1                 ((uint32_t)0x45670123)
#define OPA_KEY2                 ((uint32_t)0xCDEF89AB)

volatile uint32_t CTLR2_tmp = 0;

/********************************************************************************
 * @fn             OPA_Unlock
 *
 * @brief          Unlocks the OPA Controller.
 *
 * @return         None
 */
void OPA_Unlock(void)
{
    OPA->OPAKEY = OPA_KEY1;
    OPA->OPAKEY = OPA_KEY2;
}

/********************************************************************************
 * @fn             OPA_Lock
 *
 * @brief          Locks the OPA Controller.
 *
 * @return         None
 */
void OPA_Lock(void)
{
    OPA->CTLR1 |= (1<<31);
}

/********************************************************************************
 * @fn             OPA_POLL_Unlock
 *
 * @brief          Unlocks the OPA POLL Controller.
 *
 * @return         None
 */
void OPA_POLL_Unlock(void)
{
    OPA->POLLKEY = OPA_KEY1;
    OPA->POLLKEY = OPA_KEY2;
}

/********************************************************************************
 * @fn             OPA_POLL_Lock
 *
 * @brief          Locks the OPA POLL Controller.
 *
 * @return         None
 */
void OPA_POLL_Lock(void)
{
    OPA->CFGR1 |= (1<<7);
}

/********************************************************************************
 * @fn             OPA_CMP_Unlock
 *
 * @brief          Unlocks the CMP Controller.
 *
 * @return         None
 */
void OPA_CMP_Unlock(void)
{
    OPA->CMPKEY = OPA_KEY1;
    OPA->CMPKEY = OPA_KEY2;
}

/********************************************************************************
 * @fn             OPA_CMP_Lock
 *
 * @brief          Locks the CMP Controller.
 *
 * @return         None
 */
void OPA_CMP_Lock(void)
{
    CTLR2_tmp |= (1<<31);
    OPA->CTLR2 = CTLR2_tmp;
    CTLR2_tmp &= ~(1<<31);
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
        tmp1 &= 0xFCFF;
        tmp2 &= 0xFFFF0001;

        tmp1 |= (OPA_InitStruct->POLL_NUM << 9);
        tmp2 |= (OPA_InitStruct->Mode << 1) | (OPA_InitStruct->PSEL << 3)
                | (OPA_InitStruct->FB << 5) | (OPA_InitStruct->NSEL << 6);
    }
    else if(OPA_InitStruct->OPA_NUM == OPA2)
    {
        tmp1 &= 0xF3FF;
        tmp2 &= 0x0001FFFF;

        tmp1 |= (OPA_InitStruct->POLL_NUM << 11);
        tmp2 |= (OPA_InitStruct->Mode << 17) | (OPA_InitStruct->PSEL << 19)
                | (OPA_InitStruct->FB << 21) | (OPA_InitStruct->NSEL << 22);
    }

    tmp0 |= (OPA_InitStruct->PSEL_POLL) | (OPA_InitStruct->BKIN_EN << 2)
                     | (OPA_InitStruct->RST_EN << 4) | (OPA_InitStruct->BKIN_SEL << 6)
                     | (OPA_InitStruct->OUT_IE << 8) | (OPA_InitStruct->CNT_IE << 10)
                     | (OPA_InitStruct->NMI_IE << 11);
    tmp1 &= 0xFF00;
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
    OPA_InitStruct->Mode = OUT_IO_OUT0;
    OPA_InitStruct->PSEL = CHP0;
    OPA_InitStruct->FB = FB_OFF;
    OPA_InitStruct->NSEL = CHN0;
    OPA_InitStruct->PSEL_POLL = CHP_OPA1_OFF_OPA2_OFF;
    OPA_InitStruct->BKIN_EN = BKIN_OPA1_OFF_OPA2_OFF;
    OPA_InitStruct->RST_EN = RST_OPA1_OFF_OPA2_OFF;
    OPA_InitStruct->BKIN_SEL = BKIN_OPA1_TIM1_OPA2_TIM2;
    OPA_InitStruct->OUT_IE = OUT_IE_OPA1_OFF_OPA2_OFF;
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
 *            NewState - ENABLE or DISABLE.
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
        tmp1 &= 0xFFFFFFE1;
        tmp1 |= (CMP_InitStruct->Mode << 1) | (CMP_InitStruct->NSEL << 2)
                | (CMP_InitStruct->PSEL << 3) | (CMP_InitStruct->HYEN << 4);
    }
    else if(CMP_InitStruct->CMP_NUM == CMP2)
    {
        tmp1 &= 0xFFFFFC3F;
        tmp1 |= (CMP_InitStruct->Mode << 6) | (CMP_InitStruct->NSEL << 7)
                | (CMP_InitStruct->PSEL << 8) | (CMP_InitStruct->HYEN << 9);
    }
    else if(CMP_InitStruct->CMP_NUM == CMP3)
    {
        tmp1 &= 0xFFFF87FF;
        tmp1 |= (CMP_InitStruct->Mode << 11) | (CMP_InitStruct->NSEL << 12)
                | (CMP_InitStruct->PSEL << 13) | (CMP_InitStruct->HYEN << 14);
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
    CMP_InitStruct->Mode = OUT_IO_TIM2;
    CMP_InitStruct->NSEL = CMP_CHN0;
    CMP_InitStruct->PSEL = CMP_CHP1;
    CMP_InitStruct->HYEN = CMP_HYEN1;
}

/*********************************************************************
 * @fn      OPA_CMP_Cmd
 *
 * @brief   Enables or disables the specified CMP peripheral.
 *
 * @param   CMP_NUM - Select CMP
 *            NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void OPA_CMP_Cmd(CMP_Num_TypeDef CMP_NUM, FunctionalState NewState)
{
    if(NewState == ENABLE)
    {
        CTLR2_tmp |= (uint32_t)(1 << (CMP_NUM*5));
    }
    else
    {
        CTLR2_tmp &= ~(uint32_t)(1 << (CMP_NUM*5));
    }

    OPA->CTLR2 = CTLR2_tmp;
}

/*********************************************************************
 * @fn      OPA_GetFlagStatus
 *
 * @brief   Checks whether the OPA flag is set or not.
 *
 * @param   OPA_FLAG - specifies the SPI/I2S flag to check.
 *            OPA_FLAG_OUT_OPA1 - OPA1 out flag
 *            OPA_FLAG_OUT_OPA2 - OPA2 out flag
 *            OPA_FLAG_OUT_CNT - OPA out flag rising edge of sampling data
 *
 * @return  FlagStatus: SET or RESET.
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
 *            OPA_FLAG_OUT_OPA2 - OPA2 out flag
 *            OPA_FLAG_OUT_CNT - OPA out flag rising edge of sampling data
 * @return  none
 */
void OPA_ClearFlag(uint16_t OPA_FLAG)
{
    OPA->CFGR1 &= (uint16_t)~OPA_FLAG;
}
