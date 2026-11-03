/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH572_cmp.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2024/12/17
 * Description        : source file(ch572/ch570)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH572_common.h"

void CMP_Init (CMPSwTypeDef s, CMPNrefLevelTypeDef v)
{
    R8_CMP_CTRL_0 = (s << 2)|(v << 4);
}

void CMP_OutToTIMCAPCfg(FunctionalState s)
{
    if (s)
    {
        R8_CMP_CTRL_0 |= RB_CMP_CAP;
    }
    else
    {
        R8_CMP_CTRL_0 &= ~RB_CMP_CAP;
    }
}

void CMP_INTCfg (CMPOutSelTypeDef sel, FunctionalState s)
{
    R8_CMP_CTRL_1 = (sel << 2);
    if (s)
    {
        R8_CMP_CTRL_1 |= RB_CMP_IE;
    }
    else
    {
        R8_CMP_CTRL_1 &= ~RB_CMP_IE;
    }
}

