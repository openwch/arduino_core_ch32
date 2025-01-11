/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32x035_awu.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/04/06
 * Description        : This file provides all the AWU firmware functions.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32x035_awu.h"

/* PSC registers bit mask */
#define AWUPSC_MASK      ((uint32_t)0xFFFFFFF0)

/* WR register bit mask */
#define AWUWR_MASK       ((uint32_t)0xFFFFFFC0)

/*********************************************************************
 * @fn      AutoWakeUpCmd
 *
 * @brief   Enables or disables the Auto WakeUp functionality.
 *
 * @param   NewState - new state of the Auto WakeUp functionality
 *        (ENABLE or DISABLE).
 *
 * @return  none
 */
void AutoWakeUpCmd(FunctionalState NewState)
{
    if(NewState)
    {
        AWU->CSR |= (1 << 1);
    }
    else
    {
        AWU->CSR &= ~(1 << 1);
    }
}

/*********************************************************************
 * @fn      AWU_SetPrescaler
 *
 * @brief   Sets the Auto Wake up Prescaler
 *
 * @param   AWU_Prescaler - specifies the Auto Wake up Prescaler
 *            AWU_Prescaler_1 - AWU counter clock = LSI/1
 *            AWU_Prescaler_2 - AWU counter clock = LSI/2
 *            AWU_Prescaler_4 - AWU counter clock = LSI/4
 *            AWU_Prescaler_8 - AWU counter clock = LSI/8
 *            AWU_Prescaler_16 - AWU counter clock = LSI/16
 *            AWU_Prescaler_32 - AWU counter clock = LSI/32
 *            AWU_Prescaler_64 - AWU counter clock = LSI/64
 *            AWU_Prescaler_128 - AWU counter clock = LSI/128
 *            AWU_Prescaler_256 - AWU counter clock = LSI/256
 *            AWU_Prescaler_512 - AWU counter clock = LSI/512
 *            AWU_Prescaler_1024 - AWU counter clock = LSI/1024
 *            AWU_Prescaler_2048 - AWU counter clock = LSI/2048
 *            AWU_Prescaler_4096 - AWU counter clock = LSI/4096
 *            AWU_Prescaler_10240 - AWU counter clock = LSI/10240
 *            AWU_Prescaler_61440 - AWU counter clock = LSI/61440
 *
 * @return  none
 */
void AWU_SetPrescaler(uint32_t AWU_Prescaler)
{
    uint32_t tmpreg = 0;
    tmpreg = AWU->PSC & AWUPSC_MASK;
    tmpreg |= AWU_Prescaler;
    AWU->PSC = tmpreg;
}

/*********************************************************************
 * @fn      AWU_SetWindowValue
 *
 * @brief   Sets the WWDG window value
 *
 * @param   WindowValue - specifies the window value to be compared to the
 *        downcounter,which must be lower than 0x3F
 *
 * @return  none
 */
void AWU_SetWindowValue(uint8_t WindowValue)
{
    __IO uint32_t tmpreg = 0;

    tmpreg = AWU->WR & AWUWR_MASK;
    tmpreg |= WindowValue;

    AWU->WR = tmpreg;
}
