/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH572_clk.c
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : source file(ch572/ch570)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH572_common.h"

/**
 * @brief  LSIʱ�ӣ�Hz��
 */
uint32_t Freq_LSI = 0;

/*********************************************************************
 * @fn      LClk_Cfg
 *
 * @brief   ��Ƶʱ�ӵ�Դ����
 *
 * @param   s   - �Ƿ�򿪵�Դ
 *
 * @return  none
 */
void LClk_Cfg(FunctionalState s)
{
    uint8_t cfg = R8_LSI_CONFIG;

    if(s == DISABLE)
    {
        cfg &= ~RB_CLK_LSI_PON;
    }
    else
    {
        cfg |= RB_CLK_LSI_PON;
    }

    sys_safe_access_enable();
    R8_LSI_CONFIG = cfg;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      HSECFG_Current
 *
 * @brief   HSE���� ƫ�õ�������
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
 * @brief   HSE���� ���ص�������
 *
 * @param   c   - refer to HSECapTypeDef
 *
 * @return  none
 */
void HSECFG_Capacitance(HSECapTypeDef c)
{
    uint8_t x32M_c;

    x32M_c = R8_XT32M_TUNE;
    x32M_c = (x32M_c & 0x0f) | (c << 4);

    sys_safe_access_enable();
    R8_XT32M_TUNE = x32M_c;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      RTC_InitClock
 *
 * @brief   ��ʼ�� RTCʱ��, ����������Խ��,��ʼ��ʱ��Խ��,ʱ�Ӿ���Խ��
 *
 * @param   cnt     - the total number of cycles captured by the oscillator
 *
 * @return  RTCʱ��, 24~42KHz
 */
uint32_t RTC_InitClock(RTC_OSCCntTypeDef cnt)
{
    uint32_t count;
    uint32_t cyc;
    uint32_t last_ov_cnt = 0;
    uint32_t new_ov_cnt = 0;
    uint32_t ov_cnt_ov_cnt = 0;

    if(cnt<Count_32)
    {
        cyc = 1<<cnt;
    }
    else if(cnt<Count_1024)
    {
        cyc = 1<<(cnt+2);
    }
    else if(cnt<Count_2047)
    {
        cyc = 1024;
    }
    else
    {
        cyc = 2047;
    }
    R16_OSC_CAL_CNT |= RB_OSC_CAL_OV_CLR | RB_OSC_CAL_IF;
    sys_safe_access_enable();
    R8_OSC_CAL_CTRL |= RB_CNT_CLR;
    R8_OSC_CAL_CTRL = cnt;
    R8_OSC_CAL_CTRL |= RB_OSC_CNT_EN;
    sys_safe_access_disable();
    while(!(R16_OSC_CAL_CNT&RB_OSC_CAL_IF))
    {
        new_ov_cnt = R8_OSC_CAL_OV_CNT;
        if(new_ov_cnt<last_ov_cnt)
        {
            ov_cnt_ov_cnt++;
        }
        last_ov_cnt = new_ov_cnt;
    }
    count = ((uint32_t)R16_OSC_CAL_CNT&RB_OSC_CAL_CNT) + ((uint32_t)R8_OSC_CAL_OV_CNT+ov_cnt_ov_cnt*256)*16384;
    Freq_LSI = GetSysClock()/(count/cyc);
    return Freq_LSI;
}

/*********************************************************************
 * @fn      RTCInitTime
 *
 * @brief   RTCʱ�ӳ�ʼ����ǰʱ�䣬ע�⣺��������������ʵʱLSIƵ�ʼ�������LSIƵ���нϴ󲨶�ʱ����ȡ����ʱ�ӻ���ƫ�
 *
 * @param   y       - �����꣬MAX_Y = BEGYEAR + 44
 * @param   mon     - �����£�MAX_MON = 12
 * @param   d       - �����գ�MAX_D = 31
 * @param   h       - ����Сʱ��MAX_H = 23
 * @param   m       - ���÷��ӣ�MAX_M = 59
 * @param   s       - �����룬MAX_S = 59
 *
 * @return  none
 */
void RTC_InitTime(uint16_t y, uint16_t mon, uint16_t d, uint16_t h, uint16_t m, uint16_t s)
{
    uint32_t         t;
    uint32_t         year, month, day, sec2;
    uint32_t         t32k;
    volatile uint8_t clk_pin;
    uint32_t temp,temp1,temp2,temp3;
    uint32_t tmp,tmp1,tmp2;

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
    t32k = (s & 1) ? (Freq_LSI) : (0);
    t = sec2;
    t = t << 16 | t32k;

    temp = day * Freq_LSI;
    temp1 = temp % 32768; // ��������
    temp = temp / 32768; // ����

    temp2 = temp1 * 675; // temp1 / 32768 * 2831155200 / 65536
    temp3 = temp2 % 512;   // �������� ת����2sΪ��λ������
    temp2 = temp2 / 512;   // �������� ת����2sΪ��λ����

    temp1 = temp3 * 128; // 2sΪ��λ������ ת���� ������  temp3 / 512 * 65536

    tmp = sec2 * Freq_LSI;
    tmp1 = tmp % 32768; // 2s����
    tmp = tmp / 32768; // 2s��

    tmp2 = tmp1 * 2 ; //  2sΪ��λ������ ת���� ������  // tmp1 / 32768 * 65536

    t32k = (t32k * Freq_LSI + 16384) / 32768; // ������

    t32k += tmp2 + temp1; // ��������
    tmp += (t32k/65536) + temp2; // ��2s��
    temp += (tmp/43200);    // ����

    t32k %= 65536;  // ������
    tmp %= 43200;   // 2s��

    t = tmp;     // 64000
    t = t << 16 | t32k; // 1


    do
    {
        clk_pin = (R8_LSI_CONFIG & RB_LSI_CLK_PIN);
    } while(clk_pin != (R8_LSI_CONFIG & RB_LSI_CLK_PIN));
    if(!clk_pin)
    {
        while(!clk_pin)
        {
            do
            {
                clk_pin = (R8_LSI_CONFIG & RB_LSI_CLK_PIN);
            } while(clk_pin != (R8_LSI_CONFIG & RB_LSI_CLK_PIN));
        }
    }

    sys_safe_access_enable();
    R32_RTC_TRIG = temp;
    R8_RTC_MODE_CTRL |= RB_RTC_LOAD_HI;
    sys_safe_access_disable();
    while((R32_RTC_TRIG & 0x3FFF) != (R32_RTC_CNT_DIV2 & 0x3FFF));
    sys_safe_access_enable();
    R32_RTC_TRIG = t;
    R8_RTC_MODE_CTRL |= RB_RTC_LOAD_LO;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      RTC_GetTime
 *
 * @brief   ��ȡ��ǰʱ��
 *
 * @param   py      - ��ȡ�����꣬MAX_Y = BEGYEAR + 44
 * @param   pmon    - ��ȡ�����£�MAX_MON = 12
 * @param   pd      - ��ȡ�����գ�MAX_D = 31
 * @param   ph      - ��ȡ����Сʱ��MAX_H = 23
 * @param   pm      - ��ȡ���ķ��ӣ�MAX_M = 59
 * @param   ps      - ��ȡ�����룬MAX_S = 59
 *
 * @return  none
 */
void RTC_GetTime(uint16_t *py, uint16_t *pmon, uint16_t *pd, uint16_t *ph, uint16_t *pm, uint16_t *ps)
{
    uint32_t t;
    uint32_t day, sec2, t32k;
    uint32_t temp,temp1,temp2,temp3;
    uint32_t tmp,tmp1,tmp2;

    day = R32_RTC_CNT_DIV2 & 0x3FFF;
    sec2 = R16_RTC_CNT_DIV1;
    t32k = R16_RTC_CNT_LSI;

    temp = day * 32768;
    temp1 = temp % Freq_LSI; // ��������
    temp = temp / Freq_LSI; // ����

    temp2 = temp1 * 43200; // temp1 / Freq_LSI * 43200
    temp3 = temp2 % Freq_LSI;   // �������� ת����2sΪ��λ������
    temp2 = temp2 / Freq_LSI;   // �������� ת����2sΪ��λ����

    temp1 = (temp3 * 65536 + Freq_LSI/2 ) / Freq_LSI; // 2sΪ��λ������ ת���� ������  temp3 / Freq_LSI * 65536

    tmp = sec2 * 32768;
    tmp1 = tmp % Freq_LSI; // 2s����           5376
    tmp = tmp / Freq_LSI; // 2s��                 1799

    tmp2 = (tmp1 * 65536 + Freq_LSI/2 ) / Freq_LSI; //  2sΪ��λ������ ת���� ������  // tmp1 / Freq_LSI * 65536  11010

    t32k = (t32k * 32768 + Freq_LSI/2 ) / Freq_LSI; // ������       54525


    t32k += tmp2 + temp1; // ��������
    tmp += (t32k/65536) + temp2; // ��2s��
    temp += (tmp/43200);    // ����

    t32k %= 65536;  // ������
    tmp %= 43200;   // 2s��


    t = tmp * 2 + ((t32k < 0x8000) ? 0 : 1);

    *py = BEGYEAR;
    while(temp >= YearLength(*py))
    {
        temp -= YearLength(*py);
        (*py)++;
    }

    *pmon = 0;
    while(temp >= monthLength(IsLeapYear(*py), *pmon))
    {
        temp -= monthLength(IsLeapYear(*py), *pmon);
        (*pmon)++;
    }
    (*pmon)++;
    *pd = temp + 1;
    *ph = t / 3600;
    *pm = t % 3600 / 60;
    *ps = t % 60;
}

/*********************************************************************
 * @fn      RTC_SetCycleLSI
 *
 * @brief   ����LSIʱ�ӣ����õ�ǰRTC ������
 *
 * @param   cyc     - �������ڼ�����ֵ��MAX_CYC = 0xA8BFFFFF = 2831155199
 *
 * @return  none
 */
void RTC_SetCycleLSI(uint32_t cyc)
{
    volatile uint8_t clk_pin;

    do
    {
        clk_pin = (R8_LSI_CONFIG & RB_LSI_CLK_PIN);
    } while((clk_pin != (R8_LSI_CONFIG & RB_LSI_CLK_PIN)) || (!clk_pin));

    sys_safe_access_enable();
    R32_RTC_TRIG = cyc;
    R8_RTC_MODE_CTRL |= RB_RTC_LOAD_LO;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      RTC_GetCycleLSI
 *
 * @brief   ����LSIʱ�ӣ���ȡ��ǰRTC ������
 *
 * @param   none
 *
 * @return  ��ǰ��������MAX_CYC = 0xA8BFFFFF = 2831155199
 */
uint32_t RTC_GetCycleLSI(void)
{
    volatile uint32_t i;

    do
    {
        i = R32_RTC_CNT_LSI;
    } while(i != R32_RTC_CNT_LSI);

    return (i);
}

/*********************************************************************
 * @fn      RTC_TMRFunCfg
 *
 * @brief   RTC��ʱģʽ���ã�ע�ⶨʱ��׼�̶�Ϊ32768Hz��
 *
 * @param   t   - refer to RTC_TMRCycTypeDef
 *
 * @return  none
 */
void RTC_TMRFunCfg(RTC_TMRCycTypeDef t)
{
    sys_safe_access_enable();
    R8_RTC_MODE_CTRL &= ~(RB_RTC_TMR_EN | RB_RTC_TMR_MODE);
    sys_safe_access_disable();
    sys_safe_access_enable();
    R8_RTC_MODE_CTRL |= RB_RTC_TMR_EN | (t);
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      RTC_TRIGFunCfg
 *
 * @brief   RTCʱ�䴥��ģʽ����
 *
 * @param   cyc - ��Ե�ǰʱ��Ĵ������ʱ�䣬����LSIʱ��������
 *
 * @return  none
 */
void RTC_TRIGFunCfg(uint32_t cyc)
{
    uint32_t t;

    t = RTC_GetCycleLSI() + cyc;
    if(t > RTC_MAX_COUNT)
    {
        t -= RTC_MAX_COUNT;
    }

    sys_safe_access_enable();
    R32_RTC_TRIG = t;
    R8_RTC_MODE_CTRL |= RB_RTC_TRIG_EN;
    sys_safe_access_disable();
}

/*********************************************************************
 * @fn      RTC_ModeFunDisable
 *
 * @brief   RTC ģʽ���ܹر�
 *
 * @param   m   - ��Ҫ�رյĵ�ǰģʽ
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
 * @brief   ��ȡRTC�жϱ�־
 *
 * @param   f   - refer to RTC_EVENTTypeDef
 *
 * @return  �жϱ�־״̬
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
 * @brief   ���RTC�жϱ�־
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
