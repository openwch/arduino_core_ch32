/********************************** (C) COPYRIGHT *******************************
 * File Name          : ch32x035_pwr.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/06/14
 * Description        : This file provides all the PWR firmware functions.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32x035_pwr.h"
#include "ch32x035_rcc.h"

/* PWR registers bit mask */
/* CTLR register bit mask */
#define CTLR_DS_MASK     ((uint32_t)0xFFFFFFFD)
#define CTLR_PLS_MASK    ((uint32_t)0xFFFFFF9F)

/*********************************************************************
 * @fn      PWR_DeInit
 *
 * @brief   Deinitializes the PWR peripheral registers to their default
 *        reset values.
 *
 * @return  none
 */
void PWR_DeInit(void)
{
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_PWR, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_PWR, DISABLE);
}

/*********************************************************************
 * @fn      PWR_PVDLevelConfig
 *
 * @brief   Configures the voltage threshold detected by the Power Voltage
 *        Detector(PVD).
 *
 * @param   PWR_PVDLevel - specifies the PVD detection level
 *            PWR_PVDLevel_0 - PVD detection level set to mode 0
 *            PWR_PVDLevel_1 - PVD detection level set to mode 1
 *            PWR_PVDLevel_2 - PVD detection level set to mode 2
 *            PWR_PVDLevel_3 - PVD detection level set to mode 3
 *
 * @return  none
 */
void PWR_PVDLevelConfig(uint32_t PWR_PVDLevel)
{
    uint32_t tmpreg = 0;
    tmpreg = PWR->CTLR;
    tmpreg &= CTLR_PLS_MASK;
    tmpreg |= PWR_PVDLevel;
    PWR->CTLR = tmpreg;
}

/*********************************************************************
 * @fn      PWR_EnterSTOPMode
 *
 * @brief   Enters STOP mode.
 *
 * @param   PWR_STOPEntry - specifies if STOP mode in entered with WFI or WFE instruction.
 *            PWR_STOPEntry_WFI - enter STOP mode with WFI instruction
 *            PWR_STOPEntry_WFE - enter STOP mode with WFE instruction
 *
 * @return  none
 */
void PWR_EnterSTOPMode(uint8_t PWR_STOPEntry)
{
    uint32_t tmpreg = 0;
    tmpreg = PWR->CTLR;
    tmpreg &= CTLR_DS_MASK;
    PWR->CTLR = tmpreg;

    NVIC->SCTLR |= (1 << 2);

    if(PWR_STOPEntry == PWR_STOPEntry_WFI)
    {
        __WFI();
    }
    else
    {
        __WFE();
    }

    NVIC->SCTLR &= ~(1 << 2);
}

/*********************************************************************
 * @fn      PWR_EnterSTANDBYMode
 *
 * @brief   Enters STANDBY mode.
 *
 * @return  none
 */
void PWR_EnterSTANDBYMode(void)
{
    PWR->CTLR |= PWR_CTLR_PDDS;
    NVIC->SCTLR |= (1 << 2);

    __WFI();
}

/*********************************************************************
 * @fn      PWR_GetFlagStatus
 *
 * @brief   Checks whether the specified PWR flag is set or not.
 *
 * @param   PWR_FLAG - specifies the flag to check.
 *            PWR_FLAG_PVDO - PVD Output
 *            PWR_FLAG_FLASH - Flash low power flag
 *
 * @return  The new state of PWR_FLAG (SET or RESET).
 */
FlagStatus PWR_GetFlagStatus(uint32_t PWR_FLAG)
{
    FlagStatus bitstatus = RESET;

    if((PWR->CSR & PWR_FLAG) != (uint32_t)RESET)
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
 * @fn      PWR_VDD_SupplyVoltage
 *
 * @brief   Checks VDD Supply Voltage.
 *
 * @param   none
 *
 * @return  PWR_VDD - VDD Supply Voltage.
 *            PWR_VDD_5V - VDD = 5V
 *            PWR_VDD_3V3 - VDD = 3.3V
 */
PWR_VDD PWR_VDD_SupplyVoltage(void)
{

    PWR_VDD VDD_Voltage = PWR_VDD_3V3;
    Delay_Init();
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_PWR, ENABLE);
    PWR_PVDLevelConfig(PWR_PVDLevel_3);
    Delay_Us(10);
    if( PWR_GetFlagStatus(PWR_FLAG_PVDO) == (uint32_t)RESET)
    {
        VDD_Voltage = PWR_VDD_5V;
    }
    PWR_PVDLevelConfig(PWR_PVDLevel_0);

    return VDD_Voltage;
}
