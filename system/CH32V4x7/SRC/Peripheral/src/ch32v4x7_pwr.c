/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32v4x7_pwr.c
* Author             : WCH
* Version            : V1.0.1
* Date               : 2026/04/10
* Description        : This file provides all the PWR firmware functions.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32v4x7_pwr.h"
#include "ch32v4x7_rcc.h"

/* PWR registers bit mask */
/* CTLR register bit mask */
#define CTLR_DS_MASK     ((uint32_t)0xFFFFFFFC)
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
    RCC_PB1PeriphResetCmd(RCC_PB1Periph_PWR, ENABLE);
    RCC_PB1PeriphResetCmd(RCC_PB1Periph_PWR, DISABLE);
}

/*********************************************************************
 * @fn      PWR_BackupAccessCmd
 *
 * @brief   Enables or disables access to the RTC and backup registers.
 *
 * @param   NewState - new state of the access to the RTC and backup registers,
 *            This parameter can be: ENABLE or DISABLE.
 *
 * @return  none
 */
void PWR_BackupAccessCmd(FunctionalState NewState)
{
    if(NewState)
    {
        PWR->CTLR |= (1 << 8);
    }
    else
    {
        PWR->CTLR &= ~(1 << 8);
    }
}

/*********************************************************************
 * @fn      PWR_PVDCmd
 *
 * @brief   Enables or disables the Power Voltage Detector(PVD).
 *
 * @param   NewState - new state of the PVD(ENABLE or DISABLE).
 *
 * @return  none
 */
void PWR_PVDCmd(FunctionalState NewState)
{
    if(NewState)
    {
        PWR->CTLR |= (1 << 4);
    }
    else
    {
        PWR->CTLR &= ~(1 << 4);
    }
}

/*********************************************************************
 * @fn      PWR_PVDLevelConfig
 *
 * @brief   Configures the voltage threshold detected by the Power Voltage
 *        Detector(PVD).
 *
 * @param   PWR_PVDLevel - specifies the PVD detection level
 *            PWR_PVDLevel_MODE0 - rising edge 2.85v / falling edge 2.81v.
 *            PWR_PVDLevel_MODE1 - rising edge 2.94v / falling edge 2.90v.
 *            PWR_PVDLevel_MODE2 - rising edge 3.04v / falling edge 3.00v.
 *            PWR_PVDLevel_MODE3 - rising edge 3.14v / falling edge 3.10v.
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
 * @fn      PWR_WakeUpPinCmd
 *
 * @brief   Enables or disables the WakeUp Pin functionality.
 *
 * @param   NewState - new state of the WakeUp Pin functionality
 *        (ENABLE or DISABLE).
 *
 * @return  none
 */
void PWR_WakeUpPinCmd(FunctionalState NewState)
{
    if(NewState)
    {
        PWR->CSR |= (1 << 8);
    }
    else
    {
        PWR->CSR &= ~(1 << 8);
    }
}

/*********************************************************************
 * @fn      PWR_EnterSTOPMode
 *
 * @brief   Enters STOP mode.
 *
 * @param   PWR_Regulator - specifies the regulator state in STOP mode.
 *            PWR_Regulator_ON - STOP mode with regulator ON
 *            PWR_Regulator_LowPower - STOP mode with regulator in low power mode
 *          PWR_STOPEntry - specifies if STOP mode in entered with WFI or WFE instruction.
 *            PWR_STOPEntry_WFI - enter STOP mode with WFI instruction
 *            PWR_STOPEntry_WFE - enter STOP mode with WFE instruction
 *
 * @return  none
 */
void PWR_EnterSTOPMode(uint32_t PWR_Regulator, uint8_t PWR_STOPEntry)
{
    uint32_t tmpreg = 0;
    tmpreg = PWR->CTLR;
    tmpreg &= CTLR_DS_MASK;
    tmpreg |= PWR_Regulator;
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
    PWR->CTLR |= PWR_CTLR_CWUF;
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
 *            PWR_FLAG_WU - Wake Up flag
 *            PWR_FLAG_SB - StandBy flag
 *            PWR_FLAG_PVDO - PVD Output
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
 * @fn      PWR_ClearFlag
 *
 * @brief   Clears the PWR's pending flags.
 *
 * @param   PWR_FLAG - specifies the flag to clear.
 *            PWR_FLAG_WU - Wake Up flag
 *            PWR_FLAG_SB - StandBy flag
 *
 * @return  none
 */
void PWR_ClearFlag(uint32_t PWR_FLAG)
{
    PWR->CTLR |= PWR_FLAG << 2;
}

/*********************************************************************
 * @fn      PWR_EnterSTANDBYMode_RAM
 *
 * @brief   Enters STANDBY mode with RAM data retention function on.
 *
 * @return  none
 */
void PWR_EnterSTANDBYMode_RAM(void)
{
    uint32_t tmpreg = 0;
    tmpreg = PWR->CTLR;

    tmpreg |= PWR_CTLR_CWUF;
    tmpreg |= PWR_CTLR_PDDS;

    //32K RAM power in standby.
    tmpreg |= (0x1 << 16);

    PWR->CTLR = tmpreg;

    NVIC->SCTLR |= (1 << 2);

    __WFI();
}

/*********************************************************************
 * @fn      PWR_EnterSTANDBYMode_RAM_LV
 *
 * @brief   Enters STANDBY mode with RAM data retention function and LV mode on.
 *
 * @return  none
 */
void PWR_EnterSTANDBYMode_RAM_LV(void)
{
    uint32_t tmpreg = 0;
    tmpreg = PWR->CTLR;

    tmpreg |= PWR_CTLR_CWUF;
    tmpreg |= PWR_CTLR_PDDS;

    //32K RAM power in standby.
    tmpreg |= (0x1 << 16);
    //32K RAM low-voltage mode.
    tmpreg |= (0x1 << 20);

    PWR->CTLR = tmpreg;

    NVIC->SCTLR |= (1 << 2);

    __WFI();
}

/*********************************************************************
 * @fn      PWR_EnterSTANDBYMode_RAM_VBAT_EN
 *
 * @brief   Enters STANDBY mode with RAM data retention function on (VBAT Enable).
 *
 * @return  none
 */
void PWR_EnterSTANDBYMode_RAM_VBAT_EN(void)
{
    uint32_t tmpreg = 0;
    tmpreg = PWR->CTLR;

    tmpreg |= PWR_CTLR_CWUF;
    tmpreg |= PWR_CTLR_PDDS;

    //32K RAM power in standby.(VBAT Enable).
    tmpreg |= (0x1 << 18);

    PWR->CTLR = tmpreg;

    NVIC->SCTLR |= (1 << 2);

    __WFI();
}

/*********************************************************************
 * @fn      PWR_EnterSTANDBYMode_RAM_LV_VBAT_EN
 *
 * @brief   Enters STANDBY mode with RAM data retention function and LV mode on(VBAT Enable).
 *
 * @return  none
 */
void PWR_EnterSTANDBYMode_RAM_LV_VBAT_EN(void)
{
    uint32_t tmpreg = 0;
    tmpreg = PWR->CTLR;

    tmpreg |= PWR_CTLR_CWUF;
    tmpreg |= PWR_CTLR_PDDS;

    //32K RAM power in standby. (VBAT Enable).
    tmpreg |= (0x1 << 18);
    //32K RAM low-voltage mode.
    tmpreg |= (0x1 << 20);

    PWR->CTLR = tmpreg;

    NVIC->SCTLR |= (1 << 2);

    __WFI();
}

/*********************************************************************
 * @fn      PWR_EnterSTOPMode_RAM_LV
 *
 * @brief   Enters STOP mode with RAM data retention function and LV mode on.
 *
 * @param   PWR_Regulator - specifies the regulator state in STOP mode.
 *            PWR_Regulator_LowPower - STOP mode with regulator in low power mode
 *          PWR_STOPEntry - specifies if STOP mode in entered with WFI or WFE instruction.
 *            PWR_STOPEntry_WFI - enter STOP mode with WFI instruction
 *            PWR_STOPEntry_WFE - enter STOP mode with WFE instruction
 *
 * @return  none
 */
void PWR_EnterSTOPMode_RAM_LV(uint32_t PWR_Regulator, uint8_t PWR_STOPEntry)
{
    uint32_t tmpreg = 0;
    tmpreg = PWR->CTLR;
    tmpreg &= CTLR_DS_MASK;
    tmpreg |= PWR_Regulator;

    tmpreg |= (0x1 << 20);
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
 * @fn      PWR_VDD18Cmd
 *
 * @brief   Enables or disables the VDD18 power.
 *
 * @param   NewState - new state of the VDD18 power functionality
 *        (ENABLE or DISABLE).
 *
 * @return  none
 */

void PWR_VDD18Cmd(FunctionalState NewState)
{
    if(NewState)
    {
        PWR->CTLR |= (1 << 9);
    }
    else
    {
        PWR->CTLR &= ~(1 << 9);
    }
}

/*********************************************************************
 * @fn      PWR_VDD18LevelConfig
 *
 * @brief   Config the VDD18 level.
 *
 * @param   PWR_VDD18_Level - the VDD18 level.
 *            PWR_VDD18_Level0 - set VDD18 to level 0.
 *            PWR_VDD18_Level1 - set VDD18 to level 1.
 *
 * @return  none
 */
void PWR_VDD18LevelConfig(uint32_t PWR_VDD18_Level)
{
    EXTEN->EXTEN_CTR &= ~EXTEN_LDO18_ADJ;
    EXTEN->EXTEN_CTR |= PWR_VDD18_Level;
}
