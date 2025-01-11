/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32v10x_dbgmcu.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2020/04/30
 * Description        : This file provides all the DBGMCU firmware functions.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32v10x_dbgmcu.h"

#define IDCODE_DEVID_MASK    ((uint32_t)0x0000FFFF)

/*********************************************************************
 * @fn      DBGMCU_GetREVID
 *
 * @brief   Returns the device revision identifier.
 *
 * @return  Revision identifier.
 */
uint32_t DBGMCU_GetREVID(void)
{
    return ((*(uint32_t *)0x1FFFF884) >> 16);
}

/*********************************************************************
 * @fn      DBGMCU_GetDEVID
 *
 * @brief   Returns the device identifier.
 *
 * @return  Device identifier.
 */
uint32_t DBGMCU_GetDEVID(void)
{
    return ((*(uint32_t *)0x1FFFF884) & IDCODE_DEVID_MASK);
}


/*********************************************************************
 * @fn      DBGMCU_Config
 *
 * @brief   Configures the specified peripheral and low power mode behavior
 *        when the MCU under Debug mode.
 *
 * @param   DBGMCU_Periph - specifies the peripheral and low power mode.
 *            DBGMCU_IWDG_STOP - Debug IWDG stopped when Core is halted
 *            DBGMCU_WWDG_STOP - Debug WWDG stopped when Core is halted
 *            DBGMCU_I2C1_SMBUS_TIMEOUT - I2C1 SMBUS timeout mode stopped when Core is halted
 *            DBGMCU_I2C2_SMBUS_TIMEOUT - I2C2 SMBUS timeout mode stopped when Core is halted
 *            DBGMCU_TIM1_STOP - TIM1 counter stopped when Core is halted
 *            DBGMCU_TIM2_STOP - TIM2 counter stopped when Core is halted
 *            DBGMCU_TIM3_STOP - TIM3 counter stopped when Core is halted
 *            DBGMCU_TIM4_STOP - TIM4 counter stopped when Core is halted
 *            DBGMCU_SLEEP - Keep debugger connection during SLEEP mode
 *            DBGMCU_STOP - Keep debugger connection during STOP mode
 *            DBGMCU_STANDBY - Keep debugger connection during STANDBY mode
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void DBGMCU_Config(uint32_t DBGMCU_Periph, FunctionalState NewState)
{
    if((DBGMCU_Periph == DBGMCU_SLEEP) || (DBGMCU_Periph == DBGMCU_STOP) || (DBGMCU_Periph == DBGMCU_STANDBY))
    {
        if(NewState != DISABLE)
        {
            DBGMCU->CFGR1 |= DBGMCU_Periph;
        }
        else
        {
            DBGMCU->CFGR1 &= ~DBGMCU_Periph;
        }
    }
    else
    {
        if(NewState != DISABLE)
        {
            DBGMCU->CFGR0 |= DBGMCU_Periph;
        }
        else
        {
            DBGMCU->CFGR0 &= ~DBGMCU_Periph;
        }
    }
}
/*********************************************************************
 * @fn      DBGMCU_GetCHIPID
 *
 * @brief   Returns the CHIP identifier.
 *
 * @return Device identifier.
 *          ChipID List-
 *	CH32V103C8T6-0x25004102
 * 	CH32V103R8T6-0x2500410F
 */
uint32_t DBGMCU_GetCHIPID( void )
{
    return( *( uint32_t * )0x1FFFF884 );
}