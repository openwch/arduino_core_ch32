/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH585_clk.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : head file(ch585/ch584)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __CH585_CLK_H__
#define __CH585_CLK_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  ϵͳ��Ƶ����
 */
typedef enum
{
    CLK_SOURCE_32KHz = 0xC0,

    CLK_SOURCE_HSI_16MHz = (0x100 | 0x80),
    CLK_SOURCE_HSI_8MHz = 0x02,
    CLK_SOURCE_HSI_5_3MHz = 0x03,
    CLK_SOURCE_HSI_4MHz = 0x04,
    CLK_SOURCE_HSI_2MHz = 0x08,
    CLK_SOURCE_HSI_1MHz = 0x10,

    CLK_SOURCE_HSE_32MHz = (0x100 | 0x200 | 0x80),
    CLK_SOURCE_HSE_16MHz = (0x200 | 0x02),
    CLK_SOURCE_HSE_8MHz = (0x200 | 0x04),
    CLK_SOURCE_HSE_6_4MHz = (0x200 | 0x05),
    CLK_SOURCE_HSE_4MHz = (0x200 | 0x08),
    CLK_SOURCE_HSE_2MHz = (0x200 | 0x10),

    CLK_SOURCE_HSI_PLL_78MHz = (0x100 | 0x40 | 4),
    CLK_SOURCE_HSI_PLL_62_4MHz = (0x100 | 0x40 | 5),
    CLK_SOURCE_HSI_PLL_52MHz = (0x100 | 0x40 | 6),
    CLK_SOURCE_HSI_PLL_39MHz = (0x100 | 0x40 | 8),
    CLK_SOURCE_HSI_PLL_26MHz = (0x100 | 0x40 | 12),
    CLK_SOURCE_HSI_PLL_24MHz = (0x100 | 0x40 | 13),
    CLK_SOURCE_HSI_PLL_19_5MHz = (0x100 | 0x40 | 16),
    CLK_SOURCE_HSI_PLL_13MHz = (0x100 | 0x40 | 24),

    CLK_SOURCE_HSE_PLL_78MHz = (0x300 | 0x40 | 4),
    CLK_SOURCE_HSE_PLL_62_4MHz = (0x300 | 0x40 | 5),
    CLK_SOURCE_HSE_PLL_52MHz = (0x300 | 0x40 | 6),
    CLK_SOURCE_HSE_PLL_39MHz = (0x300 | 0x40 | 8),
    CLK_SOURCE_HSE_PLL_26MHz = (0x300 | 0x40 | 12),
    CLK_SOURCE_HSE_PLL_24MHz = (0x300 | 0x40 | 13),
    CLK_SOURCE_HSE_PLL_19_5MHz = (0x300 | 0x40 | 16),
    CLK_SOURCE_HSE_PLL_13MHz = (0x300 | 0x40 | 24),

} SYS_CLKTypeDef;

/**
 * @brief  32Kʱ��ѡ��
 */
typedef enum
{
    Clk32K_LSI = 0,
    Clk32K_LSE,

} LClk32KTypeDef;

/**
 * @brief  32M���������λ
 */
typedef enum
{
    HSE_RCur_75 = 0,
    HSE_RCur_100,
    HSE_RCur_125,
    HSE_RCur_150

} HSECurrentTypeDef;

/**
 * @brief  32M�����ڲ����ݵ�λ
 */
typedef enum
{
    HSECap_10p = 0,
    HSECap_12p,
    HSECap_14p,
    HSECap_16p,
    HSECap_18p,
    HSECap_20p,
    HSECap_22p,
    HSECap_24p,
    HSECap_2p,
    HSECap_4p,
    HSECap_6p,
    HSECap_8p

} HSECapTypeDef;

/**
 * @brief  LSE32K���������λ
 */
typedef enum
{
    LSE_RCur_70 = 0,
    LSE_RCur_100,
    LSE_RCur_140,
    LSE_RCur_200

} LSECurrentTypeDef;

/**
 * @brief  LSI32K���������λ
 */
typedef enum
{
    LSI_RCur_70 = 0,
    LSI_RCur_100,
    LSI_RCur_140,
    LSI_RCur_200

} LSICurrentTypeDef;

/**
 * @brief  32K�����ڲ����ݵ�λ
 */
typedef enum
{
    LSECap_2p = 0,  // ���屣����ʵ����Ϊ12p
    LSECap_12p = 0,
    LSECap_13p,
    LSECap_14p,
    LSECap_15p,
    LSECap_16p,
    LSECap_17p,
    LSECap_18p,
    LSECap_19p,
    LSECap_20p,
    LSECap_21p,
    LSECap_22p,
    LSECap_23p,
    LSECap_24p,
    LSECap_25p,
    LSECap_26p,
    LSECap_27p

} LSECapTypeDef;

#define RTC_MAX_COUNT             0xA8C00000

#define MAX_DAY                   0x00004000
#define MAX_2_SEC                 0x0000A8C0
//#define	 MAX_SEC		0x545FFFFF

#define BEGYEAR                   2020
#define IsLeapYear(yr)            (!((yr) % 400) || (((yr) % 100) && !((yr) % 4)))
#define YearLength(yr)            (IsLeapYear(yr) ? 366 : 365)
#define monthLength(lpyr, mon)    (((mon) == 1) ? (28 + (lpyr)) : (((mon) > 6) ? (((mon) & 1) ? 31 : 30) : (((mon) & 1) ? 30 : 31)))

/**
 * @brief  rtc timer mode period define
 */
typedef enum
{
    Period_0_125_S = 0, // 0.125s ����
    Period_0_25_S,      // 0.25s ����
    Period_0_5_S,       // 0.5s ����
    Period_1_S,         // 1s ����
    Period_2_S,         // 2s ����
    Period_4_S,         // 4s ����
    Period_8_S,         // 8s ����
    Period_16_S,        // 16s ����
} RTC_TMRCycTypeDef;

/**
 * @brief  rtc interrupt event define
 */
typedef enum
{
    RTC_TRIG_EVENT = 0, // RTC �����¼�
    RTC_TMR_EVENT,      // RTC ���ڶ�ʱ�¼�

} RTC_EVENTTypeDef;

/**
 * @brief  rtc interrupt event define
 */
typedef enum
{
    RTC_TRIG_MODE = 0, // RTC ����ģʽ
    RTC_TMR_MODE,      // RTC ���ڶ�ʱģʽ

} RTC_MODETypeDef;

typedef enum
{
    /* У׼����Խ�ߣ���ʱԽ�� */
    Level_32 = 3, // ��ʱ 1.2ms
    Level_64,     // ��ʱ 2.2ms
    Level_128,    // ��ʱ 4.2ms
    Level_1024,    // ��ʱ 32.2ms

} Cali_LevelTypeDef;

/**
 * @brief   32K ��Ƶʱ����Դ
 *
 * @param   hc  - ѡ��32Kʹ���ڲ������ⲿ
 */
void LClk32K_Select(LClk32KTypeDef hc);

/**
 * @brief   HSE���� ƫ�õ�������
 *
 * @param   c   - 75%,100%,125%,150%
 */
void HSECFG_Current(HSECurrentTypeDef c);

/**
 * @brief   HSE���� ���ص�������
 *
 * @param   c   - refer to HSECapTypeDef
 */
void HSECFG_Capacitance(HSECapTypeDef c);

/**
 * @brief   LSI���� ƫ�õ�������
 *
 * @param   c   - 70%,100%,140%,200%
 */
void LSICFG_Current(LSICurrentTypeDef c);

/**
 * @brief   LSE���� ƫ�õ�������
 *
 * @param   c   - 70%,100%,140%,200%
 */
void LSECFG_Current(LSECurrentTypeDef c);

/**
 * @brief   LSE���� ���ص�������
 *
 * @param   c   - refer to LSECapTypeDef
 */
void LSECFG_Capacitance(LSECapTypeDef c);

void Calibration_LSI(Cali_LevelTypeDef cali_Lv); /* ����ƵУ׼�ڲ�32Kʱ�� */

/**
 * @brief   RTCʱ�ӳ�ʼ����ǰʱ��
 *
 * @param   y       - �����꣬MAX_Y = BEGYEAR + 44
 * @param   mon     - �����£�MAX_MON = 12
 * @param   d       - �����գ�MAX_D = 31
 * @param   h       - ����Сʱ��MAX_H = 23
 * @param   m       - ���÷��ӣ�MAX_M = 59
 * @param   s       - �����룬MAX_S = 59
 */
void RTC_InitTime(uint16_t y, uint16_t mon, uint16_t d, uint16_t h, uint16_t m, uint16_t s);

/**
 * @brief   ��ȡ��ǰʱ��
 *
 * @param   py      - ��ȡ�����꣬MAX_Y = BEGYEAR + 44
 * @param   pmon    - ��ȡ�����£�MAX_MON = 12
 * @param   pd      - ��ȡ�����գ�MAX_D = 31
 * @param   ph      - ��ȡ����Сʱ��MAX_H = 23
 * @param   pm      - ��ȡ���ķ��ӣ�MAX_M = 59
 * @param   ps      - ��ȡ�����룬MAX_S = 59
 */
void RTC_GetTime(uint16_t *py, uint16_t *pmon, uint16_t *pd, uint16_t *ph, uint16_t *pm, uint16_t *ps);

/**
 * @brief   ����LSE/LSIʱ�ӣ����õ�ǰRTC ������
 *
 * @param   cyc     - �������ڼ�����ֵ��MAX_CYC = 0xA8BFFFFF = 2831155199
 */
void RTC_SetCycle32k(uint32_t cyc);

/**
 * @brief   ����LSE/LSIʱ�ӣ���ȡ��ǰRTC ������
 *
 * @return  ��ǰ��������MAX_CYC = 0xA8BFFFFF = 2831155199
 */
uint32_t RTC_GetCycle32k(void);

/**
 * @brief   RTC��ʱģʽ���ã�ע�ⶨʱ��׼�̶�Ϊ32768Hz��
 *
 * @param   t   - refer to RTC_TMRCycTypeDef
 */
void RTC_TRIGFunCfg(uint32_t cyc);

/**
 * @brief   RTC��ʱģʽ���ã�ע�ⶨʱ��׼�̶�Ϊ32768Hz��
 *
 * @param   t   - refer to RTC_TMRCycTypeDef
 */
void RTC_TMRFunCfg(RTC_TMRCycTypeDef t);

/**
 * @brief   RTC ģʽ���ܹر�
 *
 * @param   m   - ��Ҫ�رյĵ�ǰģʽ
 */
void RTC_ModeFunDisable(RTC_MODETypeDef m);

/**
 * @brief   ��ȡRTC�жϱ�־
 *
 * @param   f   - refer to RTC_EVENTTypeDef
 *
 * @return  �жϱ�־״̬
 */
uint8_t RTC_GetITFlag(RTC_EVENTTypeDef f);

/**
 * @brief   ���RTC�жϱ�־
 *
 * @param   f   - refer to RTC_EVENTTypeDef
 */
void RTC_ClearITFlag(RTC_EVENTTypeDef f);

/**
 * @brief   32K ��Ƶʱ�ӵ�Դ����
 */
void LClk32K_Cfg(LClk32KTypeDef hc, FunctionalState s);


#ifdef __cplusplus
}
#endif

#endif // __CH585_CLK_H__
