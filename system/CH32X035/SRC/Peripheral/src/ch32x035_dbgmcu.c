/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32x035_dbgmcu.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/04/06
 * Description        : This file provides all the DBGMCU firmware functions.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32x035_dbgmcu.h"

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
	return ((*(uint32_t *)0x1FFFF704) >> 16);
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
	return ((*(uint32_t *)0x1FFFF704) & IDCODE_DEVID_MASK);
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
 *            DBGMCU_IWDG_STOP - Debug IWDG stopped when Core is halted
 *            DBGMCU_WWDG_STOP - Debug WWDG stopped when Core is halted
 *            DBGMCU_TIM1_STOP - TIM1 counter stopped when Core is halted
 *            DBGMCU_TIM2_STOP - TIM2 counter stopped when Core is halted
 *            DBGMCU_TIM3_STOP - TIM3 counter stopped when Core is halted
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
 *	CH32X035R8T6-0x035006x1
 *  CH32X035C8T6-0x035106x1
 *  CH32X035F8U6-0x035E06x1
 *  CH32X035G8U6-0x035606x1
 *  CH32X035G8R6-0x035B06x1
 *  CH32X035F7P6-0x035706x1
 *  CH32X033F8P6-0x035A06x1
 */
uint32_t DBGMCU_GetCHIPID( void )
{
	return( *( uint32_t * )0x1FFFF704 );
}
