/********************************** (C) COPYRIGHT *******************************
 * File Name          : ch32l103_pwr.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/01/22
 * Description        : This file provides all the PWR firmware functions.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#include "ch32l103_pwr.h"
#include "ch32l103_rcc.h"

/* PWR registers bit mask */
/* CTLR register bit mask */
#define CTLR_DS_MASK     ((uint32_t)0xFFFFFFFC)
#define CTLR_PLS_MASK    ((uint32_t)0xFFFFFF1F)

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
 *            PWR_PVDLevel_0 - PVD detection level set to mode 0.
 *            PWR_PVDLevel_1 - PVD detection level set to mode 1.
 *            PWR_PVDLevel_2 - PVD detection level set to mode 2.
 *            PWR_PVDLevel_3 - PVD detection level set to mode 3.
 *            PWR_PVDLevel_4 - PVD detection level set to mode 4.
 *            PWR_PVDLevel_5 - PVD detection level set to mode 5.
 *            PWR_PVDLevel_6 - PVD detection level set to mode 6.
 *            PWR_PVDLevel_7 - PVD detection level set to mode 7.
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
    if(PWR_Regulator==PWR_Regulator_LowPower)
    {
        tmpreg &= ~(3 << 10);
        tmpreg |= (1 <<11);
    }
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
    uint32_t tmpreg = 0;
    tmpreg = PWR->CTLR;
    /* flash low power mode 1 */
    tmpreg &= ~(3 << 10);
    tmpreg |= (1 << 11);

    tmpreg |= PWR_CTLR_CWUF;
    tmpreg |= PWR_CTLR_PDDS;

    PWR->CTLR = tmpreg;

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
 * @return  none
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

    /* flash low power mode 1 */
	tmpreg &= ~(3 << 10);
    tmpreg |= (1 << 11);

    tmpreg |= PWR_CTLR_CWUF;
    tmpreg |= PWR_CTLR_PDDS;

    //2K+18K in standby power.
    tmpreg |= (0x1 << 16) | (0x1 << 17);

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

    /* flash low power mode 1 */
	tmpreg &= ~(3 << 10);
    tmpreg |= (1 << 11);

    tmpreg |= PWR_CTLR_CWUF;
    tmpreg |= PWR_CTLR_PDDS;

    //2K+18K in standby power.
    tmpreg |= (0x1 << 16) | (0x1 << 17);
    //2K+18K in standby LV .
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

    /* flash low power mode 1 */
	tmpreg &= ~(3 << 10);
    tmpreg |= (1 << 11);

    tmpreg |= PWR_CTLR_CWUF;
    tmpreg |= PWR_CTLR_PDDS;

    //2K+18K in standby power (VBAT Enable).
    tmpreg |= (0x1 << 18) | (0x1 << 19);

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

    /* flash low power mode 1 */
	tmpreg &= ~(3 << 10);
    tmpreg |= (1 << 11);

    tmpreg |= PWR_CTLR_CWUF;
    tmpreg |= PWR_CTLR_PDDS;

    //2K+18K in standby power (VBAT Enable).
    tmpreg |= (0x1 << 18) | (0x1 << 19);
    //2K+18K in standby LV .
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
 *            PWR_Regulator_ON - STOP mode with regulator ON
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

    if(PWR_Regulator==PWR_Regulator_LowPower)
    {
        tmpreg &= ~(3 << 10);
        tmpreg |= (1 << 11);
    }

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
 * @fn      PWR_LDO_LP_Cmd
 *
 * @brief   Enables or disables the LDO low power mode.
 *
 * @param   NewState - new state of the LDO low power mode(ENABLE or DISABLE).
 *
 * @return  none
 */
void PWR_LDO_LP_Cmd(FunctionalState NewState)
{
    if(NewState)
    {
        PWR->CTLR |= (1 << 13);
    }
    else
    {
        PWR->CTLR &= ~(1 << 13);
    }
}

/*********************************************************************
 * @fn      PWR_STOPMode_Auto_LDO_LP_Cmd
 *
 * @brief   Enables or disables the LDO auto enter low power mode in
 *        stop mode.
 *
 * @param   NewState - new state of the LDO auto enter low power mode
 *        in stop mode(ENABLE or DISABLE).
 *
 * @return  none
 */
void PWR_STOPMode_Auto_LDO_LP_Cmd(FunctionalState NewState)
{
    if(NewState)
    {
        PWR->CTLR |= (1 << 12);
    }
    else
    {
        PWR->CTLR &= ~(1 << 12);
    }
}

/*********************************************************************
 * @fn      PWR_FLASH_LP_Cmd
 *
 * @brief   Enables or disables the FLASH enter low power mode 0.
 *
 * @param   NewState - new state of the FLASH enter low power mode 0.
 *        (ENABLE or DISABLE).
 *
 * @return  none
 */
void PWR_FLASH_LP_Cmd(FunctionalState NewState)
{
    if(NewState)
    {
        PWR->CTLR |= (7 << 9);
    }
    else
    {
        PWR->CTLR &= ~(1 << 9);
    }
}


