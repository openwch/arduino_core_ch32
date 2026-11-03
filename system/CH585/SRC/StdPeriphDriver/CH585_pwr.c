/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH585_pwr.c
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : source file(ch585/ch584)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH585_common.h"

/*********************************************************************
 * @fn      PWR_DCDCCfg
 *
 * @brief   �����ڲ�DC/DC��Դ�����ڽ�Լϵͳ����
 *
 * @param   s       - �Ƿ��DCDC��Դ
 *
 * @return  none
 */
void PWR_DCDCCfg(FunctionalState s)
{
    uint16_t adj = R16_AUX_POWER_ADJ;
    uint16_t plan = R16_POWER_PLAN;

    if(s == DISABLE)
    {
        
        adj &= ~RB_DCDC_CHARGE;
        plan &= ~(RB_PWR_DCDC_EN | RB_PWR_DCDC_PRE); // ��· DC/DC
        sys_safe_access_enable();
        R16_AUX_POWER_ADJ = adj;
        R16_POWER_PLAN = plan;
        sys_safe_access_disable();
    }
    else
    {
        uint32_t HW_Data[2];
        FLASH_EEPROM_CMD(CMD_GET_ROM_INFO, ROM_CFG_ADR_HW, HW_Data, 0);
        if((HW_Data[0]) & (1 << 13))
        {
            return;
        }
        adj |= RB_DCDC_CHARGE;
        plan |= RB_PWR_DCDC_PRE;
        sys_safe_access_enable();
        R16_AUX_POWER_ADJ = adj;
        R16_POWER_PLAN = plan;
        sys_safe_access_disable();
        DelayUs(10);
        sys_safe_access_enable();
        R16_POWER_PLAN |= RB_PWR_DCDC_EN;
        sys_safe_access_disable();
    }
}

/*********************************************************************
 * @fn      PWR_UnitModCfg
 *
 * @brief   �ɿص�Ԫģ��ĵ�Դ����
 *
 * @param   s       - �Ƿ�򿪵�Դ
 * @param   unit    - please refer to unit of controllable power supply
 *
 * @return  none
 */
void PWR_UnitModCfg(FunctionalState s, uint8_t unit)
{
    uint8_t ck32k_cfg = R8_CK32K_CONFIG;

    if(s == DISABLE) //�ر�
    {
        ck32k_cfg &= ~(unit & 0x03);
    }
    else //��
    {
        ck32k_cfg |= (unit & 0x03);
    }

    sys_safe_access_enable();
    R8_CK32K_CONFIG = ck32k_cfg;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      PWR_SafeClkCfg
 *
 * @brief   ��ȫ����ʱ�ӿ���λ
 *
 * @param   s       - �Ƿ�򿪶�Ӧ����ʱ��
 * @param   perph   - please refer to SAFE CLK control bit define
 *
 * @return  none
 */
void PWR_SafeClkCfg(FunctionalState s, uint16_t perph)
{
    uint32_t sleep_ctrl = R8_SAFE_CLK_CTRL;

    if(s == DISABLE)
    {
        sleep_ctrl |= perph;
    }
    else
    {
        sleep_ctrl &= ~perph;
    }

    sys_safe_access_enable();
    R8_SAFE_CLK_CTRL = sleep_ctrl;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      PWR_PeriphClkCfg
 *
 * @brief   ����ʱ�ӿ���λ��ע�⣺���Ҫ�ر�����ʱ�ӣ�����Ҫ�ȹرն�Ӧ������ж�
 *
 * @param   s       - �Ƿ�򿪶�Ӧ����ʱ��
 * @param   perph   - please refer to Peripher CLK control bit define
 *
 * @return  none
 */
void PWR_PeriphClkCfg(FunctionalState s, uint16_t perph)
{
    uint32_t sleep_ctrl = R32_SLEEP_CONTROL;

    if(s == DISABLE)
    {
        sleep_ctrl |= perph;
    }
    else
    {
        sleep_ctrl &= ~perph;
    }

    sys_safe_access_enable();
    R32_SLEEP_CONTROL = sleep_ctrl;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      PWR_PeriphWakeUpCfg
 *
 * @brief   ˯�߻���Դ����
 *
 * @param   s       - �Ƿ�򿪴�����˯�߻��ѹ���
 * @param   perph   - ��Ҫ���õĻ���Դ
 *                    RB_SLP_USB_WAKE   -  USBFS Ϊ����Դ
 *                    RB_SLP_USB2_WAKE  -  USBHS Ϊ����Դ
 *                    RB_SLP_RTC_WAKE   -  RTC Ϊ����Դ
 *                    RB_SLP_GPIO_WAKE  -  GPIO Ϊ����Դ
 *                    RB_SLP_BAT_WAKE   -  BAT Ϊ����Դ
 *                    RB_GPIO_EDGE_WAKE -  GPIO�������ػ������ض��ܻ���
 * @param   mode    - refer to WakeUP_ModeypeDef
 *
 * @return  none
 */
void PWR_PeriphWakeUpCfg(FunctionalState s, uint8_t perph, WakeUP_ModeypeDef mode)
{
    uint8_t m;

    if(s == DISABLE)
    {
        sys_safe_access_enable();
        R8_SLP_WAKE_CTRL &= ~perph;
        sys_safe_access_disable();
    }
    else
    {
        switch(mode)
        {
            case Short_Delay:
                m = 0x01;
                break;

            case Long_Delay:
                m = 0x00;
                break;

            default:
                m = 0x01;
                break;
        }

        sys_safe_access_enable();
        R8_SLP_WAKE_CTRL |= RB_WAKE_EV_MODE | perph;
        sys_safe_access_disable();
        sys_safe_access_enable();
        R8_SLP_POWER_CTRL &= ~(RB_WAKE_DLY_MOD);
        sys_safe_access_disable();
        sys_safe_access_enable();
        R8_SLP_POWER_CTRL |= m;
        sys_safe_access_disable();
    }
}

/*********************************************************************
 * @fn      PowerMonitor
 *
 * @brief   ��Դ���
 *
 * @param   s       - �Ƿ�򿪴˹���
 * @param   vl      - refer to VolM_LevelypeDef
 *
 * @return  none
 */
void PowerMonitor(FunctionalState s, VolM_LevelypeDef vl)
{
    uint8_t ctrl = R8_BAT_DET_CTRL;
    uint8_t cfg = R8_BAT_DET_CFG;

    if(s == DISABLE)
    {
        sys_safe_access_enable();
        R8_BAT_DET_CTRL = 0;
        sys_safe_access_disable();
    }
    else
    {
        if(vl & 0x80)
        {
            cfg = vl & 0x03;
            ctrl = RB_BAT_MON_EN | ((vl >> 2) & 1);
        }
        else
        {
            
            cfg = vl & 0x03;
            ctrl = RB_BAT_DET_EN;
        }
        sys_safe_access_enable();
        R8_BAT_DET_CTRL = ctrl;
        R8_BAT_DET_CFG = cfg;
        sys_safe_access_disable();

        mDelayuS(1);
        sys_safe_access_enable();
        R8_BAT_DET_CTRL |= RB_BAT_LOW_IE | RB_BAT_LOWER_IE;
        sys_safe_access_disable();
    }
}

/*********************************************************************
 * @fn      LowPower_Idle
 *
 * @brief   �͹���-Idleģʽ
 *
 * @param   none
 *
 * @return  none
 */
__HIGH_CODE
void LowPower_Idle(void)
{
    FLASH_ROM_SW_RESET();
    R8_FLASH_CTRL = 0x04; //flash�ر�

    PFIC->SCTLR &= ~(1 << 2); // sleep
    __WFI();
    __nop();
    __nop();
}

/*********************************************************************
 * @fn      LowPower_Halt
 *
 * @brief   �͹���-Haltģʽ���˵͹����е�HSI/5ʱ�����У����Ѻ���Ҫ�û��Լ�����ѡ��ϵͳʱ��Դ
 *
 * @param   none
 *
 * @return  none
 */
__HIGH_CODE
void LowPower_Halt(void)
{
    uint32_t i;
    uint8_t x32Mpw;
    uint16_t clk_sys_cfg;
    uint8_t flash_cfg,flash_sck;

    clk_sys_cfg = R16_CLK_SYS_CFG;
    flash_cfg = R8_FLASH_CFG;
    flash_sck = R8_FLASH_SCK;
    FLASH_ROM_SW_RESET();
    R8_FLASH_CTRL = 0x04; //flash�ر�
    x32Mpw = R8_XT32M_TUNE;
    if(!(R8_HFCK_PWR_CTRL&RB_CLK_XT32M_KEEP))
    {
        x32Mpw = (x32Mpw & 0xfc) | 0x03; // 150%�����
    }

    sys_safe_access_enable();
    R8_BAT_DET_CTRL = 0; // �رյ�ѹ���
    sys_safe_access_disable();
    sys_safe_access_enable();
    R8_XT32M_TUNE = x32Mpw;
    sys_safe_access_disable();
//    sys_safe_access_enable();
//    R8_PLL_CONFIG |= (1 << 5);
//    sys_safe_access_disable();

    if(R16_CLK_SYS_CFG & RB_OSC32M_SEL) //ʹ���ⲿ32M
    {
        sys_safe_access_enable();
        R8_SLP_POWER_CTRL &= ~(RB_WAKE_DLY_MOD);
        R8_SLP_POWER_CTRL |= 0x01;
        R8_FLASH_CFG = 0X57;
        R8_FLASH_SCK = R8_FLASH_SCK & (~(1<<4));
        R16_CLK_SYS_CFG = CLK_SOURCE_HSE_4MHz;
        sys_safe_access_disable();
    }
    else//ʹ���ڲ�16M
    {
      sys_safe_access_enable();
      R8_SLP_POWER_CTRL &= ~(RB_WAKE_DLY_MOD);
      R8_SLP_POWER_CTRL |= 0x01;
      R8_FLASH_CFG = 0X57;
      R8_FLASH_SCK = R8_FLASH_SCK & (~(1<<4));
      R16_CLK_SYS_CFG = CLK_SOURCE_HSI_4MHz;
      sys_safe_access_disable();
    }

    PFIC->SCTLR |= (1 << 2); //deep sleep
    __WFI();
    __nop();
    __nop();

    if((!(clk_sys_cfg & RB_OSC32M_SEL)) && (clk_sys_cfg & 0x100)) //ʹ���ڲ�16M
    {
      i = 40;
      do
      {
          __nop();
      }while(--i);
    }

    sys_safe_access_enable();
    R8_FLASH_CFG = flash_cfg;
    R8_FLASH_SCK = flash_sck;
    R16_CLK_SYS_CFG = clk_sys_cfg;
    sys_safe_access_disable();

//    sys_safe_access_enable();
//    R8_PLL_CONFIG &= ~(1 << 5);
//    sys_safe_access_disable();
}

/*******************************************************************************
* Function Name  : LowPower_Sleep
* Description    : �͹���-Sleepģʽ��ע�⻽�Ѻ�flash�ȶ�����Ҫ300us
* Input          : rm:
                    RB_PWR_RAM32K	-	32K retention SRAM ����
                    RB_PWR_RAM96K	-	96K main SRAM ����
                    RB_PWR_EXTEND	-	USB �� BLE ��Ԫ�������򹩵�
                    RB_PWR_XROM   - FlashROM ����
                   NULL	-	���ϵ�Ԫ���ϵ�
* Return         : None
*******************************************************************************/
__HIGH_CODE
void LowPower_Sleep(uint16_t rm)
{
    uint8_t x32Mpw;
    uint16_t power_plan;
    uint16_t clk_sys_cfg;
    uint16_t hfck_pwr_ctrl;
    uint8_t flash_cfg,flash_sck;
    uint32_t i;

    clk_sys_cfg = R16_CLK_SYS_CFG;
    hfck_pwr_ctrl = R8_HFCK_PWR_CTRL;
    x32Mpw = R8_XT32M_TUNE;
    x32Mpw = (x32Mpw & 0xfc) | 0x03; // 150%�����
    flash_cfg = R8_FLASH_CFG;
    flash_sck = R8_FLASH_SCK;

    sys_safe_access_enable();
    R8_BAT_DET_CTRL = 0; // �رյ�ѹ���
    sys_safe_access_disable();
    sys_safe_access_enable();
    R8_XT32M_TUNE = x32Mpw;
    sys_safe_access_disable();

    PFIC->SCTLR |= (1 << 2); //deep sleep

    power_plan = R16_POWER_PLAN & (RB_PWR_DCDC_EN | RB_PWR_DCDC_PRE);
    power_plan |= RB_PWR_PLAN_EN | RB_PWR_CORE | rm | (2<<11);
    if(chip_info&0x800000)
    {
        power_plan &= ~RB_XT_PRE_EN;  //��֧��RB_XT_PRE_EN����
    }

    sys_safe_access_enable();
    R16_POWER_PLAN = power_plan;
    R8_HFCK_PWR_CTRL |= RB_CLK_RC16M_PON;   //˯����Ҫ���ڲ�HSI
    sys_safe_access_disable();
    if(R16_CLK_SYS_CFG & RB_OSC32M_SEL) //ʹ���ⲿ32M
    {
        sys_safe_access_enable();
        R8_SLP_POWER_CTRL &= ~(RB_WAKE_DLY_MOD);
        R8_SLP_POWER_CTRL |= 0x01;
        R8_FLASH_CFG = 0X57;
        R8_FLASH_SCK = R8_FLASH_SCK & (~(1<<4));
        R16_CLK_SYS_CFG = CLK_SOURCE_HSE_4MHz;
        sys_safe_access_disable();
    }
    else//ʹ���ڲ�16M
    {
      sys_safe_access_enable();
      R8_SLP_POWER_CTRL &= ~(RB_WAKE_DLY_MOD);
      R8_SLP_POWER_CTRL |= 0x01;
      R8_FLASH_CFG = 0X57;
      R8_FLASH_SCK = R8_FLASH_SCK & (~(1<<4));
      R16_CLK_SYS_CFG = CLK_SOURCE_HSI_4MHz;
      sys_safe_access_disable();
    }

    __WFI();
    __nop();
    __nop();

    if(rm & RB_PWR_EXTEND)
    {
        // ע�⣺���ʹ���˸���USB����˯��ʹ��RB_PWR_EXTEND�����Ѻ���Ҫ�����и���USB�Ĵ�����λ
        R32_U2H_BC_CTRL = 0;
        (*((PUINT32V)0x4000C254)) = 0;
    }
    if((!(clk_sys_cfg & RB_OSC32M_SEL)) && (clk_sys_cfg & 0x100)) //ʹ���ڲ�16M
    {
      i = 40;
      do
      {
          __nop();
      }while(--i);
    }

    sys_safe_access_enable();
    R8_FLASH_CFG = flash_cfg;
    R8_FLASH_SCK = flash_sck;
    R16_CLK_SYS_CFG = clk_sys_cfg;
    R8_HFCK_PWR_CTRL = hfck_pwr_ctrl;
    sys_safe_access_disable();
    sys_safe_access_enable();
    R16_POWER_PLAN &= ~RB_PWR_PLAN_EN;
    sys_safe_access_disable();

//    DelayUs(300); //����˳�����������flash���룬����Ҫ��ʱ300us���˳�
}

/*********************************************************************
 * @fn      LowPower_Shutdown
 *
 * @brief   �͹���-Shutdownģʽ���˵͹����е�HSI/5ʱ�����У����Ѻ���Ҫ�û��Լ�����ѡ��ϵͳʱ��Դ
 *          @note ע����ô˺�����DCDC����ǿ�ƹرգ����Ѻ�����ֶ��ٴδ�
 *
 * @param   rm      - ����ģ��ѡ��
 *                    RB_PWR_RAM32K   -   32K retention SRAM ����
 *                    RB_PWR_RAM96K   -   96K main SRAM ����
 *                    RB_PWR_EXTEND   -   USB �� BLE ��Ԫ�������򹩵�
 *                    NULL          -   ���ϵ�Ԫ���ϵ�
 *
 * @return  none
 */
__HIGH_CODE
void LowPower_Shutdown(uint16_t rm)
{
    uint8_t x32Kpw;

    FLASH_ROM_SW_RESET();
    x32Kpw = R8_XT32K_TUNE;
    x32Kpw = (x32Kpw & 0xfc) | 0x01; // LSE�����������͵������

    SetSysClock(CLK_SOURCE_HSI_PLL_13MHz);
    sys_safe_access_enable();
    R8_BAT_DET_CTRL = 0; // �رյ�ѹ���
    sys_safe_access_disable();
    sys_safe_access_enable();
    R8_XT32K_TUNE = x32Kpw;
    sys_safe_access_disable();

    PFIC->SCTLR |= (1 << 2); //deep sleep

    sys_safe_access_enable();
    R8_SLP_POWER_CTRL |= 0x40;
    sys_safe_access_disable();
    sys_safe_access_enable();
    R16_POWER_PLAN = RB_PWR_PLAN_EN | rm;
    sys_safe_access_disable();
    __WFI();
    __nop();
    __nop();
    sys_safe_access_enable();
    R16_INT32K_TUNE = 0xFFFF;
    R8_RST_WDOG_CTRL |= RB_SOFTWARE_RESET;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      LowPower_Halt_WFE
 *
 * @brief   ʹ��WFE���ѵĵ͹���-Haltģʽ���л����ڲ�4M
 *
 * @param   none
 *
 * @return  none
 */
__HIGH_CODE
void LowPower_Halt_WFE(void)
{
    uint32_t i;
    uint8_t x32Mpw;

    FLASH_ROM_SW_RESET();
    R8_FLASH_CTRL = 0x04; //flash�ر�
    x32Mpw = R8_XT32M_TUNE;
    if(!(R8_HFCK_PWR_CTRL&RB_CLK_XT32M_KEEP))
    {
        x32Mpw = (x32Mpw & 0xfc) | 0x03; // 150%�����
    }

    sys_safe_access_enable();
    R8_BAT_DET_CTRL = 0; // �رյ�ѹ���
    sys_safe_access_disable();
    sys_safe_access_enable();
    R8_XT32M_TUNE = x32Mpw;
    sys_safe_access_disable();

    sys_safe_access_enable();
    R8_SLP_POWER_CTRL &= ~(RB_WAKE_DLY_MOD);
    R8_SLP_POWER_CTRL |= 0x01;
    R8_FLASH_CFG = 0X57;
    R8_FLASH_SCK = R8_FLASH_SCK & (~(1<<4));
    R16_CLK_SYS_CFG = CLK_SOURCE_HSI_4MHz;
    sys_safe_access_disable();

    PFIC->SCTLR |= (1 << 2); //deep sleep
    __WFE();
    __nop();
    __nop();

}

/*******************************************************************************
* Function Name  : LowPower_Sleep_WFE
* Description    : ʹ��WFE���ѵĵ͹���-Sleepģʽ���л����ڲ�4M
* Input          : rm:
                    RB_PWR_RAM32K   -   32K retention SRAM ����
                    RB_PWR_RAM96K   -   96K main SRAM ����
                    RB_PWR_EXTEND   -   USB �� BLE ��Ԫ�������򹩵�
                    RB_PWR_XROM   - FlashROM ����
                   NULL -   ���ϵ�Ԫ���ϵ�
* Return         : None
*******************************************************************************/
__HIGH_CODE
void LowPower_Sleep_WFE(uint16_t rm)
{
    uint16_t power_plan;

    sys_safe_access_enable();
    R8_BAT_DET_CTRL = 0; // �رյ�ѹ���
    sys_safe_access_disable();

    PFIC->SCTLR |= (1 << 2); //deep sleep
    power_plan = R16_POWER_PLAN & (RB_PWR_DCDC_EN | RB_PWR_DCDC_PRE);
    power_plan |= RB_PWR_PLAN_EN | RB_PWR_CORE | rm | (2<<11);
    power_plan &= ~0x4000;  //��֧��RB_XT_PRE_EN����

    sys_safe_access_enable();
    R16_POWER_PLAN = power_plan;
    R8_HFCK_PWR_CTRL |= RB_CLK_RC16M_PON;   //˯����Ҫ���ڲ�HSI
    sys_safe_access_disable();

    sys_safe_access_enable();
    R8_SLP_POWER_CTRL &= ~(RB_WAKE_DLY_MOD);
    R8_SLP_POWER_CTRL |= 0x01;
    R8_FLASH_CFG = 0X57;
    R8_FLASH_SCK = R8_FLASH_SCK & (~(1<<4));
    R16_CLK_SYS_CFG = CLK_SOURCE_HSI_4MHz;
    sys_safe_access_disable();

    __WFE();
    __nop();
    __nop();

    if(rm & RB_PWR_EXTEND)
    {
        // ע�⣺���ʹ���˸���USB����˯��ʹ��RB_PWR_EXTEND�����Ѻ���Ҫ�����и���USB�Ĵ�����λ
        R32_U2H_BC_CTRL = 0;
        (*((PUINT32V)0x4000C254)) = 0;
    }

    sys_safe_access_enable();
    R16_POWER_PLAN &= ~RB_PWR_PLAN_EN;
    sys_safe_access_disable();

//    DelayUs(100); //����˳�����������flash���룬����Ҫ��ʱ100us���˳�
}

