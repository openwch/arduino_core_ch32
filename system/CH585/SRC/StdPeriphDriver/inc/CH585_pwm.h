/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH585_pwm.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : head file(ch585/ch584)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __CH585_PWM_H__
#define __CH585_PWM_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  channel of PWM define
 */
#define CH_PWM4     0x01  // PWM4 ͨ��
#define CH_PWM5     0x02  // PWM5 ͨ��
#define CH_PWM6     0x04  // PWM6 ͨ��
#define CH_PWM7     0x08  // PWM7 ͨ��
#define CH_PWM8     0x10  // PWM8 ͨ��
#define CH_PWM9     0x20  // PWM9 ͨ��
#define CH_PWM10    0x40  // PWM10 ͨ��
#define CH_PWM11    0x80  // PWM11 ͨ��

/**
 * @brief  channel of PWM define
 */
typedef enum
{
    High_Level = 0, // Ĭ�ϵ͵�ƽ���ߵ�ƽ��Ч
    Low_Level,      // Ĭ�ϸߵ�ƽ���͵�ƽ��Ч
} PWMX_PolarTypeDef;

/**
 * @brief  Configuration PWM4_11 Cycle size
 */
typedef enum
{
    PWMX_Cycle_256 = 0, // 256 ��PWMX����
    PWMX_Cycle_255,     // 255 ��PWMX����
    PWMX_Cycle_128,     // 128 ��PWMX����
    PWMX_Cycle_127,     // 127 ��PWMX����
    PWMX_Cycle_64,      // 64 ��PWMX����
    PWMX_Cycle_63,      // 63 ��PWMX����
} PWMX_CycleTypeDef;

/**
 * @brief   PWM4-PWM11 ͨ����׼ʱ������
 *
 * @param   d   - ͨ����׼ʱ�� = d*Tsys
 */
#define PWMX_CLKCfg(d)    (R8_PWM_CLOCK_DIV = d)

/**
 * @brief   PWM4-PWM11��������
 *
 * @param   cyc - refer to PWMX_CycleTypeDef
 */
void PWMX_CycleCfg(PWMX_CycleTypeDef cyc);

/**
 * @brief   PWM4-PWM9 16λ��������
 *
 * @param   cyc - 16λ����
 */
void PWMX_16bit_CycleCfg(uint16_t cyc);

/**
 * @brief   ���� PWM4 ��Ч��������
 *
 * @param   d   - ��Ч��������
 */
#define PWM4_ActDataWidth(d)     (R8_PWM4_DATA = d)

/**
 * @brief   ���� PWM5 ��Ч��������
 *
 * @param   d   - ��Ч��������
 */
#define PWM5_ActDataWidth(d)     (R8_PWM5_DATA = d)

/**
 * @brief   ���� PWM6 ��Ч��������
 *
 * @param   d   - ��Ч��������
 */
#define PWM6_ActDataWidth(d)     (R8_PWM6_DATA = d)

/**
 * @brief   ���� PWM7 ��Ч��������
 *
 * @param   d   - ��Ч��������
 */
#define PWM7_ActDataWidth(d)     (R8_PWM7_DATA = d)

/**
 * @brief   ���� PWM8 ��Ч��������
 *
 * @param   d   - ��Ч��������
 */
#define PWM8_ActDataWidth(d)     (R8_PWM8_DATA = d)

/**
 * @brief   ���� PWM9 ��Ч��������
 *
 * @param   d   - ��Ч��������
 */
#define PWM9_ActDataWidth(d)     (R8_PWM9_DATA = d)

/**
 * @brief   ���� PWM10 ��Ч��������
 *
 * @param   d   - ��Ч��������
 */
#define PWM10_ActDataWidth(d)    (R8_PWM10_DATA = d)

/**
 * @brief   ���� PWM11 ��Ч��������
 *
 * @param   d   - ��Ч��������
 */
#define PWM11_ActDataWidth(d)    (R8_PWM11_DATA = d)

/**
 * @brief   PWM4-PWM11ͨ�������������
 *
 * @param   ch      - select channel of pwm, refer to channel of PWM define
 * @param   da      - effective pulse width
 * @param   pr      - select wave polar, refer to PWMX_PolarTypeDef
 * @param   s       - control pwmx function, ENABLE or DISABLE
 */
void PWMX_ACTOUT(uint8_t ch, uint8_t da, PWMX_PolarTypeDef pr, FunctionalState s);

/**
 * @brief   PWM4-PWM9 ͨ��16λ�����������
 *
 * @param   ch      - select channel of pwm, refer to channel of PWM define
 * @param   da      - effective pulse width
 * @param   pr      - select wave polar, refer to PWMX_PolarTypeDef
 * @param   s       - control pwmx function, ENABLE or DISABLE
 */
void PWMX_16bit_ACTOUT(uint8_t ch, uint16_t da, PWMX_PolarTypeDef pr, FunctionalState s);

/**
 * @brief   PWM �������ģʽ����
 *
 * @param   ch      - select group of PWM alternate output
 *                    RB_PWM4_5_STAG_EN     -  PWM4 �� PWM5 ͨ���������
 *                    RB_PWM6_7_STAG_EN     -  PWM6 �� PWM7 ͨ���������
 *                    RB_PWM8_9_STAG_EN     -  PWM8 �� PWM9 ͨ���������
 *                    RB_PWM10_11_STAG_EN   -  PWM10 �� PWM11 ͨ���������
 * @param   s       - control pwmx function, ENABLE or DISABLE
 */
void PWMX_AlterOutCfg(uint8_t ch, FunctionalState s);

#ifdef __cplusplus
}
#endif

#endif // __CH585_PWM_H__
