/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH572_pwm.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : head file(ch572/ch570)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __CH572_PWM_H__
#define __CH572_PWM_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  channel of PWM define
 */
#define CH_PWM1     0x01  // PWM1   ͨ��
#define CH_PWM2     0x02  // PWM2   ͨ��
#define CH_PWM3     0x04  // PWM3   ͨ��
#define CH_PWM4     0x08  // PWM4   ͨ��
#define CH_PWM5     0x10  // PWM5   ͨ��
#define CH_PWM_ALL  0x1F  // PWM1-5 ͨ��

/**
 * @brief  DMA channel of PWM
 */
typedef enum
{
    Mode_DMACH1_3 = 0, // DMAѡ��1��2��3ͨ�����
    Mode_DMACH4_5,     // DMAѡ��4��5ͨ�����
    Mode_DMACH1_5,     // DMAѡ��1��2��3��4��5ͨ�����
} PWM_DMAChannel;

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
 * @brief  Configuration DMA mode
 */
typedef enum
{
    PWM_ModeSINGLE = 0, // ����ģʽ
    PWM_ModeLOOP,       // ѭ��ģʽ
} PWM_DMAModeTypeDef;

/**
 * @brief   PWM ͨ����׼ʱ������
 *
 * @param   d   - ͨ����׼ʱ�� = d*Tsys
 */
#define PWMX_CLKCfg(d)    (R16_PWM_CLOCK_DIV = d)

/**
 * @brief   PWM 8λ��������
 *
 * @param   cyc - refer to PWMX_CycleTypeDef
 */
void PWMX_CycleCfg(PWMX_CycleTypeDef cyc);

/**
 * @brief   PWM 16λ��������
 *
 * @param   ch  - select channel of pwm, refer to channel of PWM define
 *          cyc - 16λ����
 */
void PWMX_16bit_CycleCfg(uint8_t ch, uint16_t cyc);

/**
 * @brief   PWM 16λ����λ��ʹ��
 */
#define PWM_16bit_CycleEnable()  (R8_PWM_CONFIG |= (3 << 1))

/**
 * @brief   PWM 16λ����λ��ʧ��
 */
#define PWM_16bit_CycleDisable()  (R8_PWM_CONFIG &= ~(3 << 1))

/**
 * @brief   ���� PWM1 8λ��Ч��������
 *
 * @param   d   - ��Ч��������
 */
#define PWM1_ActDataWidth(d)     (R8_PWM1_DATA = d)

/**
 * @brief   ���� PWM2 8λ��Ч��������
 *
 * @param   d   - ��Ч��������
 */
#define PWM2_ActDataWidth(d)     (R8_PWM2_DATA = d)

/**
 * @brief   ���� PWM3 8λ��Ч��������
 *
 * @param   d   - ��Ч��������
 */
#define PWM3_ActDataWidth(d)     (R8_PWM3_DATA = d)

/**
 * @brief   ���� PWM4 8λ��Ч��������
 *
 * @param   d   - ��Ч��������
 */
#define PWM4_ActDataWidth(d)     (R8_PWM4_DATA = d)

/**
 * @brief   ���� PWM5 8λ��Ч��������
 *
 * @param   d   - ��Ч��������
 */
#define PWM5_ActDataWidth(d)     (R8_PWM5_DATA = d)

/**
 * @brief   ���� PWM1 16λ��Ч��������
 *
 * @param   d   - ��Ч��������
 */
#define PWM1_16bit_ActDataWidth(d)     (R16_PWM1_DATA = d)

/**
 * @brief   ���� PWM2 16λ��Ч��������
 *
 * @param   d   - ��Ч��������
 */
#define PWM2_16bit_ActDataWidth(d)     (R16_PWM2_DATA = d)

/**
 * @brief   ���� PWM3 16λ��Ч��������
 *
 * @param   d   - ��Ч��������
 */
#define PWM3_16bit_ActDataWidth(d)     (R16_PWM3_DATA = d)

/**
 * @brief   ���� PWM4 16λ��Ч��������
 *
 * @param   d   - ��Ч��������
 */
#define PWM4_16bit_ActDataWidth(d)     (R16_PWM4_DATA = d)

/**
 * @brief   ���� PWM5 16λ��Ч��������
 *
 * @param   d   - ��Ч��������
 */
#define PWM5_16bit_ActDataWidth(d)     (R16_PWM5_DATA = d)

/**
 * @brief   PWM 8λ�����������
 *
 * @param   ch      - select channel of pwm, refer to channel of PWM define
 * @param   da      - effective pulse width
 * @param   pr      - select wave polar, refer to PWMX_PolarTypeDef
 * @param   s       - control pwmx function, ENABLE or DISABLE
 */
void PWMX_ACTOUT(uint8_t ch, uint8_t da, PWMX_PolarTypeDef pr, FunctionalState s);

/**
 * @brief   PWM 16λ�����������
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
 * @param   s       - control pwmx function, ENABLE or DISABLE
 */
void PWMX_AlterOutCfg(uint8_t ch, FunctionalState s);

/**
 * @brief   PWM ͬ�����ģʽ����
 *
 * @param   s       - control pwmx function, ENABLE or DISABLE
 */
void PWMX_SyncOutCfg(FunctionalState s);

/**
 * @brief   ����PWM DMA����
 *
 * @param   s           - �Ƿ��DMA����
 * @param   startAddr   - DMA ��ʼ��ַ
 * @param   endAddr     - DMA ������ַ
 * @param   m           - ����DMAģʽ
 */
void PWM_DMACfg(uint8_t s, uint32_t startAddr, uint32_t endAddr, PWM_DMAModeTypeDef m, PWM_DMAChannel ch);

#ifdef __cplusplus
}
#endif

#endif // __CH572_PWM_H__
