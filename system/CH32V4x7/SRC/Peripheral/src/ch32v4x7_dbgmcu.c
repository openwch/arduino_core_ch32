/********************************** (C) COPYRIGHT  *******************************
* File Name          : ch32v4x7_dbgmcu.c
* Author             : WCH
* Version            : V1.0.2
* Date               : 2026/08/21
* Description        : This file provides all the DBGMCU firmware functions.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32v4x7_dbgmcu.h"

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
    return ((*(uint32_t *)0x1FFFF704) & IDCODE_DEVID_MASK);
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
    return ((*(uint32_t *)0x1FFFF704) >> 16);
}

/*********************************************************************
 * @fn      __get_DEBUG_CR
 *
 * @brief   Return the DEBUGE Control Register
 *
 * @return  DEBUGE Control value
 */
uint32_t __get_DEBUG_CR(void)
{
    uint32_t result;

    __asm volatile("csrr %0,""0x7C0" : "=r"(result));
    return (result);
}

/*********************************************************************
 * @fn      __set_DEBUG_CR
 *
 * @brief   Set the DEBUGE Control Register
 *
 * @param   value  - set DEBUGE Control value
 *
 * @return  none
 */
void __set_DEBUG_CR(uint32_t value)
{
    __asm volatile("csrw 0x7C0, %0" : : "r"(value));
}


/*********************************************************************
 * @fn      DBGMCU_Config
 *
 * @brief   Configures the specified peripheral and low power mode behavior
 *        when the MCU under Debug mode.
 *
 * @param   DBGMCU_Periph - specifies the peripheral and low power mode.
 *            DBGMCU_SLEEP - Debug sleep stopped when Core is halted
 *            DBGMCU_STOP - Debug stop stopped when Core is halted
 *            DBGMCU_STANDBY - Debug standby stopped when Core is halted
 *            DBGMCU_IWDG_STOP - Debug IWDG stopped when Core is halted
 *            DBGMCU_WWDG_STOP - Debug WWDG stopped when Core is halted
 *            DBGMCU_I2C1_SMBUS_TIMEOUT - Debug I2C1 SMBUS time out when Core is halted
 *            DBGMCU_TIM1_STOP - TIM1 counter stopped when Core is halted
 *            DBGMCU_TIM2_STOP - TIM2 counter stopped when Core is halted
 *            DBGMCU_TIM3_STOP - TIM3 counter stopped when Core is halted
 *            DBGMCU_TIM4_STOP - TIM4 counter stopped when Core is halted
 *            DBGMCU_TIM5_STOP - TIM5 counter stopped when Core is halted
 *            DBGMCU_TIM6_STOP - TIM6 counter stopped when Core is halted
 *            DBGMCU_TIM7_STOP - TIM7 counter stopped when Core is halted
 *            DBGMCU_TIM8_STOP - TIM8 counter stopped when Core is halted
 *            DBGMCU_CAN1_STOP - Debug CAN1 stopped when Core is halted
 *          NewState - ENABLE or DISABLE.
 *
 * @return  none
 */
void DBGMCU_Config(uint32_t DBGMCU_Periph, FunctionalState NewState)
{
    uint32_t val;

    if(NewState != DISABLE)
    {
        __set_DEBUG_CR(DBGMCU_Periph);
    }
    else
    {
        val = __get_DEBUG_CR();
        val &= ~(uint32_t)DBGMCU_Periph;
        __set_DEBUG_CR(val);
    }

}

/*********************************************************************
 * @fn      DBGMCU_GetCHIPID
 *
 * @brief   Returns the CHIP identifier.
 *
 * @return Device identifier.
 *          ChipID List-
 *  CH32V407VET - 0x467000x0
 *  CH32V407WEU - 0x467100x1
 *  CH32V407RET - 0x467200x2
 *  CH32V467VET - 0x467300x0
 *  CH32V467WEU - 0x467400x1
 *  CH32V467RET - 0x467500x2
 *  CH32V406RET - 0x467700x2
 */
uint32_t DBGMCU_GetCHIPID( void )
{
    return( *( uint32_t * )0x1FFFF704 );
}

