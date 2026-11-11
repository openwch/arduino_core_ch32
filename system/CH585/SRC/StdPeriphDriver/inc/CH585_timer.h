/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH585_timer.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : head file(ch585/ch584)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __CH585_TIMER_H__
#define __CH585_TIMER_H__

#ifdef __cplusplus
extern "C" {
#endif

#define DataBit_25            (1 << 25)

/**
 * @brief  TMR0 interrupt bit define
 */

#define TMR0_3_IT_CYC_END     0x01  // ���ڽ�����־����׽-��ʱ����ʱ-���ڽ�����PWM-���ڽ���
#define TMR0_3_IT_DATA_ACT    0x02  // ������Ч��־����׽-�����ݣ�PWM-��Ч��ƽ����
#define TMR0_3_IT_FIFO_HF     0x04  // FIFO ʹ�ù��룺��׽- FIFO>=4�� PWM- FIFO<4
#define TMR0_3_IT_DMA_END     0x08  // DMA ������֧��TMR0-TMR3
#define TMR0_3_IT_FIFO_OV     0x10  // FIFO �������׽- FIFO���� PWM- FIFO��

/**
 * @brief  Configuration PWM effective level repeat times
 */
typedef enum
{
    PWM_Times_1 = 0, // PWM ��Ч����ظ�1����
    PWM_Times_4,     // PWM ��Ч����ظ�4����
    PWM_Times_8,     // PWM ��Ч����ظ�8����
    PWM_Times_16,    // PWM ��Ч����ظ�16����
} PWM_RepeatTsTypeDef;

/**
 * @brief  Configuration Cap mode
 */
typedef enum
{
    CAP_NULL = 0,         // ����׽ & ������
    Edge_To_Edge,         // �������֮��  &  �����������
    FallEdge_To_FallEdge, // �½��ص��½���  & �����½���
    RiseEdge_To_RiseEdge, // �����ص�������  &  ����������
} CapModeTypeDef;

/**
 * @brief  Configuration DMA mode
 */
typedef enum
{
    Mode_Single = 0, // ����ģʽ
    Mode_LOOP,       // ѭ��ģʽ
} DMAModeTypeDef;

/**
 * @brief   ��ʱ���ܳ�ʼ��
 *
 * @param   t       - ��ʱʱ�䣬���ڵ�ǰϵͳʱ��Tsys, ���ʱ���� 67108864
 */
void TMR0_TimerInit(uint32_t t);

/**
 * @brief   ��ȡ��ǰ��ʱ��ֵ�����67108864
 *
 * @return  ��ǰ��ʱ��ֵ
 */
#define TMR0_GetCurrentTimer()    R32_TMR0_COUNT

/**
 * @brief   ���ؼ������ܳ�ʼ��
 *
 * @param   cap     - �ɼ���������
 */
void TMR0_EXTSingleCounterInit(CapModeTypeDef cap);

/**
 * @brief   ���ü���ͳ�������С�����67108862
 *
 * @param   cyc     - ����ͳ�������С
 */
#define TMR0_CountOverflowCfg(cyc)    (R32_TMR0_CNT_END = (cyc + 2))

/**
 * @brief   ��ȡ��ǰ����ֵ�����67108862
 *
 * @return  ��ǰ����ֵ
 */
#define TMR0_GetCurrentCount()        R32_TMR0_COUNT

/**
 * @brief   PWM0 ͨ�����������������, ���67108864
 *
 * @param   cyc     - �����������
 */
#define TMR0_PWMCycleCfg(cyc)         (R32_TMR0_CNT_END = cyc)

/**
 * @brief   PWM �����ʼ��
 *
 * @param   pr      - select wave polar, refer to PWMX_PolarTypeDef
 * @param   ts      - set pwm repeat times, refer to PWM_RepeatTsTypeDef
 */
void TMR0_PWMInit(PWMX_PolarTypeDef pr, PWM_RepeatTsTypeDef ts);

/**
 * @brief   PWM0 ��Ч��������, ���67108864
 *
 * @param   d       - ��Ч��������
 */
#define TMR0_PWMActDataWidth(d)    (R32_TMR0_FIFO = d)

/**
 * @brief   CAP0 ��׽��ƽ��ʱ����, ���33554432
 *
 * @param   cyc     - ��׽��ƽ��ʱ
 */
#define TMR0_CAPTimeoutCfg(cyc)    (R32_TMR0_CNT_END = cyc)

/**
 * @brief   �ⲿ�źŲ�׽���ܳ�ʼ��
 *
 * @param   cap     - select capture mode, refer to CapModeTypeDef
 */
void TMR0_CapInit(CapModeTypeDef cap);

/**
 * @brief   ��ȡ��������
 *
 * @return  ��������
 */
#define TMR0_CAPGetData()        R32_TMR0_FIFO

/**
 * @brief   ��ȡ��ǰ�Ѳ������ݸ���
 *
 * @return  ��ǰ�Ѳ������ݸ���
 */
#define TMR0_CAPDataCounter()    R8_TMR0_FIFO_COUNT

/**
 * @brief   ����DMA����
 *
 * @param   s           - �Ƿ��DMA����
 * @param   startAddr   - DMA ��ʼ��ַ
 * @param   endAddr     - DMA ������ַ
 * @param   m           - ����DMAģʽ
 */
void TMR0_DMACfg(uint8_t s, uint32_t startAddr, uint32_t endAddr, DMAModeTypeDef m);

/**
 * @brief   �ر� TMR0 PWM���
 */
#define TMR0_PWMDisable()           (R8_TMR0_CTRL_MOD &= ~RB_TMR_OUT_EN)

/**
 * @brief   ���� TMR0 PWM���
 */
#define TMR0_PWMEnable()           (R8_TMR0_CTRL_MOD |= RB_TMR_OUT_EN)

/**
 * @brief   �ر� TMR0
 */
#define TMR0_Disable()           (R8_TMR0_CTRL_MOD &= ~RB_TMR_COUNT_EN)

/**
 * @brief   ���� TMR0
 */
#define TMR0_Enable()            (R8_TMR0_CTRL_MOD |= RB_TMR_COUNT_EN)

/**
 * @brief   �ж�����
 *
 * @param   s       - ʹ��/�ر�
 * @param   f       - refer to TMR interrupt bit define
 */
#define TMR0_ITCfg(s, f)         ((s) ? (R8_TMR0_INTER_EN |= f) : (R8_TMR0_INTER_EN &= ~f))

/**
 * @brief   ����жϱ�־
 *
 * @param   f       - refer to TMR interrupt bit define
 */
#define TMR0_ClearITFlag(f)      (R8_TMR0_INT_FLAG = f)

/**
 * @brief   ��ѯ�жϱ�־״̬
 *
 * @param   f       - refer to TMR interrupt bit define
 */
#define TMR0_GetITFlag(f)        (R8_TMR0_INT_FLAG & f)

/**
 * @brief   ��ʱ���ܳ�ʼ��
 *
 * @param   t       - ��ʱʱ�䣬���ڵ�ǰϵͳʱ��Tsys, ���ʱ���� 67108864
 */
void TMR1_TimerInit(uint32_t t);

/**
 * @brief   ��ȡ��ǰ��ʱ��ֵ�����67108864
 *
 * @return  ��ǰ��ʱ��ֵ
 */
#define TMR1_GetCurrentTimer()    R32_TMR1_COUNT

/**
 * @brief   ���ؼ������ܳ�ʼ��
 *
 * @param   cap     - �ɼ���������
 */
void TMR1_EXTSingleCounterInit(CapModeTypeDef cap);

/**
 * @brief   ���ü���ͳ�������С�����67108862
 *
 * @param   cyc     - ����ͳ�������С
 */
#define TMR1_CountOverflowCfg(cyc)    (R32_TMR1_CNT_END = (cyc + 2))

/**
 * @brief   ��ȡ��ǰ����ֵ�����67108862
 *
 * @return  ��ǰ����ֵ
 */
#define TMR1_GetCurrentCount()        R32_TMR1_COUNT

/**
 * @brief   PWM1 ͨ�����������������, ���67108864
 *
 * @param   cyc     - �����������
 */
#define TMR1_PWMCycleCfg(cyc)         (R32_TMR1_CNT_END = cyc)

/**
 * @brief   PWM �����ʼ��
 *
 * @param   pr      - select wave polar, refer to PWMX_PolarTypeDef
 * @param   ts      - set pwm repeat times, refer to PWM_RepeatTsTypeDef
 */
void TMR1_PWMInit(PWMX_PolarTypeDef pr, PWM_RepeatTsTypeDef ts);

/**
 * @brief   PWM1 ��Ч��������, ���67108864
 *
 * @param   d       - ��Ч��������
 */
#define TMR1_PWMActDataWidth(d)    (R32_TMR1_FIFO = d)

/**
 * @brief   CAP1 ��׽��ƽ��ʱ����, ���33554432
 *
 * @param   cyc     - ��׽��ƽ��ʱ
 */
#define TMR1_CAPTimeoutCfg(cyc)    (R32_TMR1_CNT_END = cyc)

/**
 * @brief   �ⲿ�źŲ�׽���ܳ�ʼ��
 *
 * @param   cap     - select capture mode, refer to CapModeTypeDef
 */
void TMR1_CapInit(CapModeTypeDef cap);

/**
 * @brief   ��ȡ��������
 *
 * @return  ��������
 */
#define TMR1_CAPGetData()        R32_TMR1_FIFO

/**
 * @brief   ��ȡ��ǰ�Ѳ������ݸ���
 *
 * @return  ��ǰ�Ѳ������ݸ���
 */
#define TMR1_CAPDataCounter()    R8_TMR1_FIFO_COUNT

/**
 * @brief   ����DMA����
 *
 * @param   s           - �Ƿ��DMA����
 * @param   startAddr   - DMA ��ʼ��ַ
 * @param   endAddr     - DMA ������ַ
 * @param   m           - ����DMAģʽ
 */
void TMR1_DMACfg(uint8_t s, uint32_t startAddr, uint32_t endAddr, DMAModeTypeDef m);

/**
 * @brief   �ر� TMR1 PWM���
 */
#define TMR1_PWMDisable()           (R8_TMR1_CTRL_MOD &= ~RB_TMR_OUT_EN)

/**
 * @brief   ���� TMR1 PWM���
 */
#define TMR1_PWMEnable()           (R8_TMR1_CTRL_MOD |= RB_TMR_OUT_EN)

/**
 * @brief   �ر� TMR1
 */
#define TMR1_Disable()         (R8_TMR1_CTRL_MOD &= ~RB_TMR_COUNT_EN)

/**
 * @brief   ���� TMR1
 */
#define TMR1_Enable()          (R8_TMR1_CTRL_MOD |= RB_TMR_COUNT_EN)

/**
 * @brief   �ж�����
 *
 * @param   s       - ʹ��/�ر�
 * @param   f       - refer to TMR interrupt bit define
 */
#define TMR1_ITCfg(s, f)       ((s) ? (R8_TMR1_INTER_EN |= f) : (R8_TMR1_INTER_EN &= ~f))

/**
 * @brief   ����жϱ�־
 *
 * @param   f       - refer to TMR interrupt bit define
 */
#define TMR1_ClearITFlag(f)    (R8_TMR1_INT_FLAG = f)

/**
 * @brief   ��ѯ�жϱ�־״̬
 *
 * @param   f       - refer to TMR interrupt bit define
 */
#define TMR1_GetITFlag(f)      (R8_TMR1_INT_FLAG & f)

/**
 * @brief   ��ʱ���ܳ�ʼ��
 *
 * @param   t       - ��ʱʱ�䣬���ڵ�ǰϵͳʱ��Tsys, ���ʱ���� 67108864
 */
void TMR2_TimerInit(uint32_t t);

/**
 * @brief   ��ȡ��ǰ��ʱ��ֵ�����67108864
 *
 * @return  ��ǰ��ʱ��ֵ
 */
#define TMR2_GetCurrentTimer()    R32_TMR2_COUNT

/**
 * @brief   ���ؼ������ܳ�ʼ��
 *
 * @param   cap     - �ɼ���������
 */
void TMR2_EXTSingleCounterInit(CapModeTypeDef cap);

/**
 * @brief   ���ü���ͳ�������С�����67108862
 *
 * @param   cyc     - ����ͳ�������С
 */
#define TMR2_CountOverflowCfg(cyc)    (R32_TMR2_CNT_END = (cyc + 2))

/**
 * @brief   ��ȡ��ǰ����ֵ�����67108862
 *
 * @return  ��ǰ����ֵ
 */
#define TMR2_GetCurrentCount()        R32_TMR2_COUNT

/**
 * @brief   PWM2 ͨ�����������������, ���67108864
 *
 * @param   cyc     - �����������
 */
#define TMR2_PWMCycleCfg(cyc)         (R32_TMR2_CNT_END = cyc)

/**
 * @brief   PWM �����ʼ��
 *
 * @param   pr      - select wave polar, refer to PWMX_PolarTypeDef
 * @param   ts      - set pwm repeat times, refer to PWM_RepeatTsTypeDef
 */
void TMR2_PWMInit(PWMX_PolarTypeDef pr, PWM_RepeatTsTypeDef ts);

/**
 * @brief   PWM2 ��Ч��������, ���67108864
 *
 * @param   d       - ��Ч��������
 */
#define TMR2_PWMActDataWidth(d)    (R32_TMR2_FIFO = d)

/**
 * @brief   CAP2 ��׽��ƽ��ʱ����, ���33554432
 *
 * @param   cyc     - ��׽��ƽ��ʱ
 */
#define TMR2_CAPTimeoutCfg(cyc)    (R32_TMR2_CNT_END = cyc)

/**
 * @brief   �ⲿ�źŲ�׽���ܳ�ʼ��
 *
 * @param   cap     - select capture mode, refer to CapModeTypeDef
 */
void TMR2_CapInit(CapModeTypeDef cap);

/**
 * @brief   ��ȡ��������
 *
 * @return  ��������
 */
#define TMR2_CAPGetData()        R32_TMR2_FIFO

/**
 * @brief   ��ȡ��ǰ�Ѳ������ݸ���
 *
 * @return  ��ǰ�Ѳ������ݸ���
 */
#define TMR2_CAPDataCounter()    R8_TMR2_FIFO_COUNT

/**
 * @brief   ����DMA����
 *
 * @param   s           - �Ƿ��DMA����
 * @param   startAddr   - DMA ��ʼ��ַ
 * @param   endAddr     - DMA ������ַ
 * @param   m           - ����DMAģʽ
 */
void TMR2_DMACfg(uint8_t s, uint32_t startAddr, uint32_t endAddr, DMAModeTypeDef m);

/**
 * @brief   �ر� TMR2 PWM���
 */
#define TMR2_PWMDisable()           (R8_TMR2_CTRL_MOD &= ~RB_TMR_OUT_EN)

/**
 * @brief   ���� TMR2 PWM���
 */
#define TMR2_PWMEnable()           (R8_TMR2_CTRL_MOD |= RB_TMR_OUT_EN)

/**
 * @brief   �ر� TMR2
 */
#define TMR2_Disable()         (R8_TMR2_CTRL_MOD &= ~RB_TMR_COUNT_EN)

/**
 * @brief   ���� TMR2
 */
#define TMR2_Enable()          (R8_TMR2_CTRL_MOD |= RB_TMR_COUNT_EN)

/**
 * @brief   �ж�����
 *
 * @param   s       - ʹ��/�ر�
 * @param   f       - refer to TMR interrupt bit define
 */
#define TMR2_ITCfg(s, f)       ((s) ? (R8_TMR2_INTER_EN |= f) : (R8_TMR2_INTER_EN &= ~f))

/**
 * @brief   ����жϱ�־
 *
 * @param   f       - refer to TMR interrupt bit define
 */
#define TMR2_ClearITFlag(f)    (R8_TMR2_INT_FLAG = f)

/**
 * @brief   ��ѯ�жϱ�־״̬
 *
 * @param   f       - refer to TMR interrupt bit define
 */
#define TMR2_GetITFlag(f)      (R8_TMR2_INT_FLAG & f)

/**
 * @brief   ��ʱ���ܳ�ʼ��
 *
 * @param   t       - ��ʱʱ�䣬���ڵ�ǰϵͳʱ��Tsys, ���ʱ���� 67108864
 */
void TMR3_TimerInit(uint32_t t);

/**
 * @brief   ��ȡ��ǰ��ʱ��ֵ�����67108864
 *
 * @return  ��ǰ��ʱ��ֵ
 */
#define TMR3_GetCurrentTimer()    R32_TMR3_COUNT

/**
 * @brief   ���ؼ������ܳ�ʼ��
 *
 * @param   cap     - �ɼ���������
 */
void TMR3_EXTSingleCounterInit(CapModeTypeDef cap);

/**
 * @brief   ���ü���ͳ�������С�����67108862
 *
 * @param   cyc     - ����ͳ�������С
 */
#define TMR3_CountOverflowCfg(cyc)    (R32_TMR3_CNT_END = (cyc + 2))

/**
 * @brief   ��ȡ��ǰ����ֵ�����67108862
 *
 * @return  ��ǰ����ֵ
 */
#define TMR3_GetCurrentCount()        R32_TMR3_COUNT

/**
 * @brief   PWM3 ͨ�����������������, ���67108864
 *
 * @param   cyc     - �����������
 */
#define TMR3_PWMCycleCfg(cyc)         (R32_TMR3_CNT_END = cyc)

/**
 * @brief   PWM �����ʼ��
 *
 * @param   pr      - select wave polar, refer to PWMX_PolarTypeDef
 * @param   ts      - set pwm repeat times, refer to PWM_RepeatTsTypeDef
 */
void TMR3_PWMInit(PWMX_PolarTypeDef pr, PWM_RepeatTsTypeDef ts);

/**
 * @brief   PWM3 ��Ч��������, ���67108864
 *
 * @param   d       - ��Ч��������
 */
#define TMR3_PWMActDataWidth(d)    (R32_TMR3_FIFO = d)

/**
 * @brief   CAP3 ��׽��ƽ��ʱ����, ���33554432
 *
 * @param   cyc     - ��׽��ƽ��ʱ
 */
#define TMR3_CAPTimeoutCfg(cyc)    (R32_TMR3_CNT_END = cyc)

/**
 * @brief   �ⲿ�źŲ�׽���ܳ�ʼ��
 *
 * @param   cap     - select capture mode, refer to CapModeTypeDef
 */
void TMR3_CapInit(CapModeTypeDef cap);

/**
 * @brief   ��ȡ��������
 *
 * @return  ��������
 */
#define TMR3_CAPGetData()        R32_TMR3_FIFO

/**
 * @brief   ��ȡ��ǰ�Ѳ������ݸ���
 *
 * @return  ��ǰ�Ѳ������ݸ���
 */
#define TMR3_CAPDataCounter()    R8_TMR3_FIFO_COUNT

/**
 * @brief   ����DMA����
 *
 * @param   s           - �Ƿ��DMA����
 * @param   startAddr   - DMA ��ʼ��ַ
 * @param   endAddr     - DMA ������ַ
 * @param   m           - ����DMAģʽ
 */
void TMR3_DMACfg(uint8_t s, uint32_t startAddr, uint32_t endAddr, DMAModeTypeDef m);

/**
 * @brief   �ر� TMR3 PWM���
 */
#define TMR3_PWMDisable()           (R8_TMR3_CTRL_MOD &= ~RB_TMR_OUT_EN)

/**
 * @brief   ���� TMR3 PWM���
 */
#define TMR3_PWMEnable()           (R8_TMR3_CTRL_MOD |= RB_TMR_OUT_EN)

/**
 * @brief   �ر� TMR3
 */
#define TMR3_Disable()           (R8_TMR3_CTRL_MOD &= ~RB_TMR_COUNT_EN)

/**
 * @brief   ���� TMR3
 */
#define TMR3_Enable()            (R8_TMR3_CTRL_MOD |= RB_TMR_COUNT_EN)

/**
 * @brief   �ж�����
 *
 * @param   s       - ʹ��/�ر�
 * @param   f       - refer to TMR interrupt bit define
 */
#define TMR3_ITCfg(s, f)         ((s) ? (R8_TMR3_INTER_EN |= f) : (R8_TMR3_INTER_EN &= ~f))

/**
 * @brief   ����жϱ�־
 *
 * @param   f       - refer to TMR interrupt bit define
 */
#define TMR3_ClearITFlag(f)      (R8_TMR3_INT_FLAG = f)

/**
 * @brief   ��ѯ�жϱ�־״̬
 *
 * @param   f       - refer to TMR interrupt bit define
 */
#define TMR3_GetITFlag(f)        (R8_TMR3_INT_FLAG & f)

#ifdef __cplusplus
}
#endif

#endif // __CH585_TIMER_H__
