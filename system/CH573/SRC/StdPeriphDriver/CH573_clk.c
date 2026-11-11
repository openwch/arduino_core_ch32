/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH573_clk.c
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH573_common.h"

uint16_t Int32K_Tune_FLASH = 0;
uint16_t Int32K_Tune_RAM = 0;

/*********************************************************************
 * @fn      LClk32K_Select
 *
 * @brief   32K 低频时钟来源
 *
 * @param   hc  - 选择32K使用内部还是外部
 *
 * @return  none
 */
void LClk32K_Select(LClk32KTypeDef hc)
{
    uint8_t cfg = R8_CK32K_CONFIG;

    if(hc == Clk32K_LSI)
    {
        cfg &= ~RB_CLK_OSC32K_XT;
    }
    else
    {
        cfg |= RB_CLK_OSC32K_XT;
    }

    sys_safe_access_enable();
    R8_CK32K_CONFIG = cfg;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      HSECFG_Current
 *
 * @brief   HSE晶体 偏置电流配置
 *
 * @param   c   - 75%,100%,125%,150%
 *
 * @return  none
 */
void HSECFG_Current(HSECurrentTypeDef c)
{
    uint8_t x32M_c;

    x32M_c = R8_XT32M_TUNE;
    x32M_c = (x32M_c & 0xfc) | (c & 0x03);

    sys_safe_access_enable();
    R8_XT32M_TUNE = x32M_c;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      HSECFG_Capacitance
 *
 * @brief   HSE晶体 负载电容配置
 *
 * @param   c   - refer to HSECapTypeDef
 *
 * @return  none
 */
void HSECFG_Capacitance(HSECapTypeDef c)
{
    uint8_t x32M_c;

    x32M_c = R8_XT32M_TUNE;
    x32M_c = (x32M_c & 0x8f) | (c << 4);

    sys_safe_access_enable();
    R8_XT32M_TUNE = x32M_c;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      LSECFG_Current
 *
 * @brief   LSE晶体 偏置电流配置
 *
 * @param   c   - 70%,100%,140%,200%
 *
 * @return  none
 */
void LSECFG_Current(LSECurrentTypeDef c)
{
    uint8_t x32K_c;

    x32K_c = R8_XT32K_TUNE;
    x32K_c = (x32K_c & 0xfc) | (c & 0x03);

    sys_safe_access_enable();
    R8_XT32K_TUNE = x32K_c;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      LSECFG_Capacitance
 *
 * @brief   LSE晶体 负载电容配置
 *
 * @param   c   - refer to LSECapTypeDef
 *
 * @return  none
 */
void LSECFG_Capacitance(LSECapTypeDef c)
{
    uint8_t x32K_c;

    x32K_c = R8_XT32K_TUNE;
    x32K_c = (x32K_c & 0x0f) | (c << 4);

    sys_safe_access_enable();
    R8_XT32K_TUNE = x32K_c;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      Calibration_LSI_FLASH
 *
 * @brief   校准在FLASH中运行时的内部32K时钟
 *
 * @param   none
 *
 * @return  误差：万分之（单位）
 */
uint16_t Calibration_LSI_FLASH(void)
{
    uint16_t     rev, basev;
    uint32_t     calv;
    uint16_t     i;
    uint16_t     loc, loc_t;
    float        CNT_STEP_K;
    signed short diff_1, diff_2, diffc;
    uint8_t      k = 0;

    /* 根据当前时钟获取标称值和斜率（T-step） */
    rev = R16_CLK_SYS_CFG & 0xff;
    if((rev & RB_CLK_SYS_MOD) == (2 << 6))
    { // 32M做主频
        calv = ((5 * 32000000 + (CAB_LSIFQ >> 1)) / CAB_LSIFQ);
        CNT_STEP_K = -1.6;
    }
    else if((rev & RB_CLK_SYS_MOD) == (1 << 6))
    { // PLL进行分频
        calv = (((uint32_t)5 * 480000000 / (rev & 0x1f) + (CAB_LSIFQ >> 1)) / CAB_LSIFQ);
        CNT_STEP_K = -0.1 * (rev & 0x1f);
    }
    else if((rev & RB_CLK_SYS_MOD) == (0 << 6))
    { // 32M进行分频
        calv = ((5 * 32000000 / (rev & 0x1f) + (CAB_LSIFQ >> 1)) / CAB_LSIFQ);
        CNT_STEP_K = -1.6 * (rev & 0x1f);
    }
    else
    { // 32K做主频
        calv = (5);
        CNT_STEP_K = 0;
    }
    basev = calv; // 获取校准标称值

    if(Int32K_Tune_FLASH)
    {
        loc = Int32K_Tune_FLASH;
    }
    else
    {
        loc = 2048;
    }
    //  if (loc == 2048)
    //    loc = 1837; //10040.625-1.640625*5000;  //经验曲线

    diff_2 = 0;
    diffc = 0;

    sys_safe_access_enable();
    R8_OSC_CAL_CTRL = RB_OSC_CNT_EN;
    do
    {
        sys_safe_access_enable();
        R16_INT32K_TUNE = loc;
        sys_safe_access_disable();

        /* 读取当前值 */
        while(!(R8_OSC_CAL_CTRL & RB_OSC_CNT_HALT));
        i = R16_OSC_CAL_CNT; // 用于丢弃
        while(R8_OSC_CAL_CTRL & RB_OSC_CNT_HALT);
        while(!(R8_OSC_CAL_CTRL & RB_OSC_CNT_HALT));
        i = R16_OSC_CAL_CNT; // 实时校准后采样值
        k++;
        diff_1 = i - basev;

        Int32K_Tune_FLASH = loc;
        if(diff_1 == 0)
        {
            return 0; // 校准正好
        }
        else if((diff_1 * diff_2) < 0)
        { // 处于两点之间
            if((diffc == 1) || (diffc == -1) || (diffc == 0))
            {
                // 都变成正数
                if(diff_2 < 0)
                {
                    diff_2 = ~(diff_2 - 1);
                }
                else
                {
                    diff_1 = ~(diff_1 - 1);
                }

                if(diff_1 > diff_2)
                {
                    sys_safe_access_enable();
                    R16_INT32K_TUNE = loc_t;
                    sys_safe_access_disable();

                    return (diff_2 * 10000 / basev); // 返回误差值，万分之
                }
                else
                {
                    return (diff_1 * 10000 / basev);
                }
            }
        }

        if(((i > basev ? (i - basev) : (basev - i)) * 10000 / basev) < 11)
        {
            return ((i > basev ? (i - basev) : (basev - i)) * 10000 / basev);
        }
        // 保存上一次值
        diff_2 = diff_1;
        loc_t = loc;
        diffc = diff_1 * CNT_STEP_K;
        loc = loc - diffc;
        if(loc > RB_INT32K_TUNE)
        {
            loc = RB_INT32K_TUNE;
        }
        if(loc == loc_t)
        {
            if(diff_1 > 0)
            {
                loc = loc + 1; // 当前频率偏小
            }
            else
            {
                loc = loc - 1; // 当前频率偏大
            }
        }
    } while(k < 20);

    Int32K_Tune_FLASH = loc;
    return ((i > basev ? (i - basev) : (basev - i)) * 10000 / basev);
}

/*********************************************************************
 * @fn      Get_Calibration_Cnt_RAM
 *
 * @brief   获取在RAM中计数的校准计数值
 *
 * @param   loc - TUNE值
 *
 * @return  计数值
 */
__attribute__((section(".highcode")))
uint16_t Get_Calibration_Cnt_RAM(uint16_t loc)
{
    uint16_t i;

    sys_safe_access_enable();
    R16_INT32K_TUNE = loc;
    sys_safe_access_disable();
    /* 读取当前值 */
    while(!(R8_OSC_CAL_CTRL & RB_OSC_CNT_HALT));
    i = R16_OSC_CAL_CNT; // 用于丢弃
    while(R8_OSC_CAL_CTRL & RB_OSC_CNT_HALT);
    while(!(R8_OSC_CAL_CTRL & RB_OSC_CNT_HALT));
    i = R16_OSC_CAL_CNT;
    return i;
}

/*********************************************************************
 * @fn      Calibration_LSI_RAM
 *
 * @brief   校准在RAM中运行时的内部32K时钟
 *
 * @param   none
 *
 * @return  误差：万分之（单位）
 */
uint16_t Calibration_LSI_RAM(void)
{
    uint16_t     rev, basev;
    uint32_t     calv;
    uint16_t     i;
    uint16_t     loc, loc_t;
    float        CNT_STEP_K;
    signed short diff_1, diff_2, diffc;
    uint8_t      k = 0;

    /* 根据当前时钟获取标称值和斜率（T-step） */
    rev = R16_CLK_SYS_CFG & 0xff;
    if((rev & RB_CLK_SYS_MOD) == (2 << 6))
    { // 32M做主频
        calv = ((5 * 32000000 + (CAB_LSIFQ >> 1)) / CAB_LSIFQ);
        CNT_STEP_K = -1.6;
    }
    else if((rev & RB_CLK_SYS_MOD) == (1 << 6))
    { // PLL进行分频
        calv = (((uint32_t)5 * 480000000 / (rev & 0x1f) + (CAB_LSIFQ >> 1)) / CAB_LSIFQ);
        CNT_STEP_K = -0.1 * (rev & 0x1f);
    }
    else if((rev & RB_CLK_SYS_MOD) == (0 << 6))
    { // 32M进行分频
        calv = ((5 * 32000000 / (rev & 0x1f) + (CAB_LSIFQ >> 1)) / CAB_LSIFQ);
        CNT_STEP_K = -1.6 * (rev & 0x1f);
    }
    else
    { // 32K做主频
        calv = (5);
        CNT_STEP_K = 0;
    }
    basev = calv; // 获取校准标称值

    if(Int32K_Tune_RAM)
    {
        loc = Int32K_Tune_RAM;
    }
    else
    {
        loc = 2048;
    }
    //  if (loc == 2048)
    //    loc = 1837; //10040.625-1.640625*5000;  //经验曲线

    diff_2 = 0;
    diffc = 0;

    sys_safe_access_enable();
    R8_OSC_CAL_CTRL = RB_OSC_CNT_EN;
    do
    {
        i = Get_Calibration_Cnt_RAM(loc); // 实时校准后采样值
        k++;
        diff_1 = i - basev;

        Int32K_Tune_RAM = loc;
        if(diff_1 == 0)
        {
            return 0; // 校准正好
        }
        else if((diff_1 * diff_2) < 0)
        { // 处于两点之间
            if((diffc == 1) || (diffc == -1) || (diffc == 0))
            {
                // 都变成正数
                if(diff_2 < 0)
                {
                    diff_2 = ~(diff_2 - 1);
                }
                else
                {
                    diff_1 = ~(diff_1 - 1);
                }

                if(diff_1 > diff_2)
                {
                    sys_safe_access_enable();
                    R16_INT32K_TUNE = loc_t;
                    sys_safe_access_disable();

                    return (diff_2 * 10000 / basev); // 返回误差值，万分之
                }
                else
                {
                    return (diff_1 * 10000 / basev);
                }
            }
        }

        if(((i > basev ? (i - basev) : (basev - i)) * 10000 / basev) < 11)
        {
            return ((i > basev ? (i - basev) : (basev - i)) * 10000 / basev);
        }
        // 保存上一次值
        diff_2 = diff_1;
        loc_t = loc;
        diffc = diff_1 * CNT_STEP_K;
        loc = loc - diffc;
        if(loc > RB_INT32K_TUNE)
        {
            loc = RB_INT32K_TUNE;
        }
        if(loc == loc_t)
        {
            if(diff_1 > 0)
            {
                loc = loc + 1; // 当前频率偏小
            }
            else
            {
                loc = loc - 1; // 当前频率偏大
            }
        }
    } while(k < 20);

    Int32K_Tune_RAM = loc;
    return ((i > basev ? (i - basev) : (basev - i)) * 10000 / basev);
}

/*********************************************************************
 * @fn      LSI_SetTune_FLASH
 *
 * @brief   设置在FLASH中运行时的内部32k校准值
 *
 * @param   none
 *
 * @return  none
 */
void LSI_SetTune_FLASH(void)
{
    sys_safe_access_enable();
    R16_INT32K_TUNE = Int32K_Tune_FLASH;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      LSI_SetTune_RAM
 *
 * @brief   设置在RAM中运行时的内部32k校准值
 *
 * @param   none
 *
 * @return  none
 */
void LSI_SetTune_RAM(void)
{
    sys_safe_access_enable();
    R16_INT32K_TUNE = Int32K_Tune_RAM;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      RTCInitTime
 *
 * @brief   RTC时钟初始化当前时间
 *
 * @param   y       - 配置年，MAX_Y = BEGYEAR + 44
 * @param   mon     - 配置月，MAX_MON = 12
 * @param   d       - 配置日，MAX_D = 31
 * @param   h       - 配置小时，MAX_H = 23
 * @param   m       - 配置分钟，MAX_M = 59
 * @param   s       - 配置秒，MAX_S = 59
 *
 * @return  none
 */
void RTC_InitTime(uint16_t y, uint16_t mon, uint16_t d, uint16_t h, uint16_t m, uint16_t s)
{
    uint32_t         t;
    uint16_t         year, month, day, sec2, t32k;
    volatile uint8_t clk_pin;

    year = y;
    month = mon;
    day = 0;
    while(year > BEGYEAR)
    {
        day += YearLength(year - 1);
        year--;
    }
    while(month > 1)
    {
        day += monthLength(IsLeapYear(y), month - 2);
        month--;
    }

    day += d - 1;
    sec2 = (h % 24) * 1800 + m * 30 + s / 2;
    t32k = (s & 1) ? (0x8000) : (0);
    t = sec2;
    t = t << 16 | t32k;

    do
    {
        clk_pin = (R8_CK32K_CONFIG & RB_32K_CLK_PIN);
    } while((clk_pin != (R8_CK32K_CONFIG & RB_32K_CLK_PIN)) || (!clk_pin));

    sys_safe_access_enable();
    R32_RTC_TRIG = day;
    R8_RTC_MODE_CTRL |= RB_RTC_LOAD_HI;
    R32_RTC_TRIG = t;
    R8_RTC_MODE_CTRL |= RB_RTC_LOAD_LO;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      RTC_GetTime
 *
 * @brief   获取当前时间
 *
 * @param   py      - 获取到的年，MAX_Y = BEGYEAR + 44
 * @param   pmon    - 获取到的月，MAX_MON = 12
 * @param   pd      - 获取到的日，MAX_D = 31
 * @param   ph      - 获取到的小时，MAX_H = 23
 * @param   pm      - 获取到的分钟，MAX_M = 59
 * @param   ps      - 获取到的秒，MAX_S = 59
 *
 * @return  none
 */
void RTC_GetTime(uint16_t *py, uint16_t *pmon, uint16_t *pd, uint16_t *ph, uint16_t *pm, uint16_t *ps)
{
    uint32_t t;
    uint16_t day, sec2, t32k;

    day = R32_RTC_CNT_DAY & 0x3FFF;
    sec2 = R16_RTC_CNT_2S;
    t32k = R16_RTC_CNT_32K;

    t = sec2 * 2 + ((t32k < 0x8000) ? 0 : 1);

    *py = BEGYEAR;
    while(day >= YearLength(*py))
    {
        day -= YearLength(*py);
        (*py)++;
    }

    *pmon = 0;
    while(day >= monthLength(IsLeapYear(*py), *pmon))
    {
        day -= monthLength(IsLeapYear(*py), *pmon);
        (*pmon)++;
    }
    (*pmon)++;
    *pd = day + 1;
    *ph = t / 3600;
    *pm = t % 3600 / 60;
    *ps = t % 60;
}

/*********************************************************************
 * @fn      RTC_SetCycle32k
 *
 * @brief   基于LSE/LSI时钟，配置当前RTC 周期数
 *
 * @param   cyc     - 配置周期计数初值，MAX_CYC = 0xA8BFFFFF = 2831155199
 *
 * @return  none
 */
void RTC_SetCycle32k(uint32_t cyc)
{
    volatile uint8_t clk_pin;

    do
    {
        clk_pin = (R8_CK32K_CONFIG & RB_32K_CLK_PIN);
    } while((clk_pin != (R8_CK32K_CONFIG & RB_32K_CLK_PIN)) || (!clk_pin));

    sys_safe_access_enable();
    R32_RTC_TRIG = cyc;
    sys_safe_access_enable();
    R8_RTC_MODE_CTRL |= RB_RTC_LOAD_LO;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      RTC_GetCycle32k
 *
 * @brief   基于LSE/LSI时钟，获取当前RTC 周期数
 *
 * @param   none
 *
 * @return  当前周期数，MAX_CYC = 0xA8BFFFFF = 2831155199
 */
uint32_t RTC_GetCycle32k(void)
{
    volatile uint32_t i;

    do
    {
        i = R32_RTC_CNT_32K;
    } while(i != R32_RTC_CNT_32K);

    return (i);
}

/*********************************************************************
 * @fn      RTC_TMRFunCfg
 *
 * @brief   RTC定时模式配置（注意定时基准固定为32768Hz）
 *
 * @param   t   - refer to RTC_TMRCycTypeDef
 *
 * @return  none
 */
void RTC_TMRFunCfg(RTC_TMRCycTypeDef t)
{
    sys_safe_access_enable();
    R8_RTC_MODE_CTRL &= ~(RB_RTC_TMR_EN | RB_RTC_TMR_MODE);
    sys_safe_access_enable();
    R8_RTC_MODE_CTRL |= RB_RTC_TMR_EN | (t);
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      RTC_TRIGFunCfg
 *
 * @brief   RTC时间触发模式配置
 *
 * @param   cyc - 相对当前时间的触发间隔时间，基于LSE/LSI时钟周期数
 *
 * @return  none
 */
void RTC_TRIGFunCfg(uint32_t cyc)
{
    uint32_t t;

    t = RTC_GetCycle32k() + cyc;
    if(t > 0xA8C00000)
    {
        t -= 0xA8C00000;
    }

    sys_safe_access_enable();
    R32_RTC_TRIG = t;
    sys_safe_access_enable();
    R8_RTC_MODE_CTRL |= RB_RTC_TRIG_EN;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      RTC_ModeFunDisable
 *
 * @brief   RTC 模式功能关闭
 *
 * @param   m   - 需要关闭的当前模式
 *
 * @return  none
 */
void RTC_ModeFunDisable(RTC_MODETypeDef m)
{
    uint8_t i = 0;

    if(m == RTC_TRIG_MODE)
    {
        i |= RB_RTC_TRIG_EN;
    }
    else if(m == RTC_TMR_MODE)
    {
        i |= RB_RTC_TMR_EN;
    }

    sys_safe_access_enable();
    R8_RTC_MODE_CTRL &= ~(i);
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      RTC_GetITFlag
 *
 * @brief   获取RTC中断标志
 *
 * @param   f   - refer to RTC_EVENTTypeDef
 *
 * @return  中断标志状态
 */
uint8_t RTC_GetITFlag(RTC_EVENTTypeDef f)
{
    if(f == RTC_TRIG_EVENT)
    {
        return (R8_RTC_FLAG_CTRL & RB_RTC_TRIG_FLAG);
    }
    else
    {
        return (R8_RTC_FLAG_CTRL & RB_RTC_TMR_FLAG);
    }
}

/*********************************************************************
 * @fn      RTC_ClearITFlag
 *
 * @brief   清除RTC中断标志
 *
 * @param   f   - refer to RTC_EVENTTypeDef
 *
 * @return  none
 */
void RTC_ClearITFlag(RTC_EVENTTypeDef f)
{
    switch(f)
    {
        case RTC_TRIG_EVENT:
            R8_RTC_FLAG_CTRL = RB_RTC_TRIG_CLR;
            break;
        case RTC_TMR_EVENT:
            R8_RTC_FLAG_CTRL = RB_RTC_TMR_CLR;
            break;
        default:
            break;
    }
}

