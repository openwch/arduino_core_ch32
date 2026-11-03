/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH572_timer.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : head file(ch572/ch570)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __CH572_TIMER_H__
#define __CH572_TIMER_H__

#ifdef __cplusplus
extern "C" {
#endif

#define DataBit_25            (1 << 25)

/**
 * @brief  TMR interrupt bit define
 */
#define TMR_IT_CYC_END     0x01  // ���ڽ�����־����׽-��ʱ����ʱ-���ڽ�����PWM-���ڽ���
#define TMR_IT_DATA_ACT    0x02  // ������Ч��־����׽-�����ݣ�PWM-��Ч��ƽ����
#define TMR_IT_FIFO_HF     0x04  // FIFO ʹ�ù��룺��׽- FIFO>=4�� PWM- FIFO<4
#define TMR_IT_DMA_END     0x08  // DMA ������֧��TMR-TMR3
#define TMR_IT_FIFO_OV     0x10  // FIFO �������׽- FIFO���� PWM- FIFO��

/**
 * @brief  ENC interrupt bit define
 */
#define RB_IE_DIR_INC      0x01  // ǰ���ж�ʹ��
#define RB_IE_DIR_DEC      0x02  // �����ж�ʹ��

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
 * @brief  Configuration ENC mode
 */
typedef enum
{
    Mode_IDLE = 0,   // IDLEģʽ
    Mode_T2 ,        // T2���ؼ���ģʽ
    Mode_T1 ,        // T1���ؼ���ģʽ
    Mode_T1T2 ,      // T1��T2���ؼ���ģʽ
} ENCModeTypeDef;

/**
 * @brief   ��ʱ���ܳ�ʼ��
 *
 * @param   t       - ��ʱʱ�䣬���ڵ�ǰϵͳʱ��Tsys, ���ʱ���� 67108864
 */
void TMR_TimerInit(uint32_t t);

/**
 * @brief   ���ؼ������ܳ�ʼ��
 *
 * @param   cap     - �ɼ���������
 */
void TMR_EXTSingleCounterInit(CapModeTypeDef cap);

/**
 * @brief   ���ü���ͳ�������С�����67108863
 *
 * @param   cyc     - ����ͳ�������С
 */
#define TMR_CountOverflowCfg(cyc)    (R32_TMR_CNT_END = (cyc + 2))

/**
 * @brief   ��ȡ��ǰ����ֵ�����67108863
 *
 * @return  ��ǰ����ֵ
 */
#define TMR_GetCurrentCount()        R32_TMR_COUNT

/**
 * @brief   PWM0 ͨ�����������������, ���67108863
 *
 * @param   cyc     - �����������
 */
#define TMR_PWMCycleCfg(cyc)         (R32_TMR_CNT_END = cyc)

/**
 * @brief   PWM �����ʼ��
 *
 * @param   pr      - select wave polar, refer to PWMX_PolarTypeDef
 * @param   ts      - set pwm repeat times, refer to PWM_RepeatTsTypeDef
 */
void TMR_PWMInit(PWMX_PolarTypeDef pr, PWM_RepeatTsTypeDef ts);

/**
 * @brief   PWM0 ��Ч��������, ���67108864
 *
 * @param   d       - ��Ч��������
 */
#define TMR_PWMActDataWidth(d)    (R32_TMR_FIFO = d)

/**
 * @brief   CAP0 ��׽��ƽ��ʱ����, ���33554432
 *
 * @param   cyc     - ��׽��ƽ��ʱ
 */
#define TMR_CAPTimeoutCfg(cyc)    (R32_TMR_CNT_END = cyc)

/**
 * @brief   �ⲿ�źŲ�׽���ܳ�ʼ��
 *
 * @param   cap     - select capture mode, refer to CapModeTypeDef
 */
void TMR_CapInit(CapModeTypeDef cap);

/**
 * @brief   ��ȡ��������
 *
 * @return  ��������
 */
#define TMR_CAPGetData()        R32_TMR_FIFO

/**
 * @brief   ��ȡ��ǰ�Ѳ������ݸ���
 *
 * @return  ��ǰ�Ѳ������ݸ���
 */
#define TMR_CAPDataCounter()    R8_TMR_FIFO_COUNT

/**
 * @brief   ����DMA����
 *
 * @param   s           - �Ƿ��DMA����
 * @param   startAddr   - DMA ��ʼ��ַ
 * @param   endAddr     - DMA ������ַ
 * @param   m           - ����DMAģʽ
 */
void TMR_DMACfg(uint8_t s, uint32_t startAddr, uint32_t endAddr, DMAModeTypeDef m);

/**
 * @brief   ����ENC����
 *
 * @param   s           - �Ƿ�������������
 * @param   encReg      - ������ģʽ��ֵ(���ֵ0xFFFF)
 * @param   m           - ����ENCģʽ
 *
 * @return  none
 */
void ENC_Config(uint8_t s, uint32_t encReg, ENCModeTypeDef m);

/**
 * @brief   �ر� TMR PWM���
 */
#define TMR_PWMDisable()           (R8_TMR_CTRL_MOD &= ~RB_TMR_OUT_EN)

/**
 * @brief   ���� TMR PWM���
 */
#define TMR_PWMEnable()           (R8_TMR_CTRL_MOD |= RB_TMR_OUT_EN)

/**
 * @brief   �ر� TMR
 */
#define TMR_Disable()           (R8_TMR_CTRL_MOD &= ~RB_TMR_COUNT_EN)

/**
 * @brief   ���� TMR
 */
#define TMR_Enable()            (R8_TMR_CTRL_MOD |= RB_TMR_COUNT_EN)

/**
 * @brief   TMR�ж�����
 *
 * @param   s       - ʹ��/�ر�
 * @param   f       - refer to TMR interrupt bit define
 */
#define TMR_ITCfg(s, f)         ((s) ? (R8_TMR_INTER_EN |= f) : (R8_TMR_INTER_EN &= ~f))

/**
 * @brief   ���TMR�жϱ�־
 *
 * @param   f       - refer to TMR interrupt bit define
 */
#define TMR_ClearITFlag(f)      (R8_TMR_INT_FLAG = f)

/**
 * @brief   ��ѯ�жϱ�־״̬
 *
 * @param   f       - refer to TMR interrupt bit define
 */
#define TMR_GetITFlag(f)        (R8_TMR_INT_FLAG & f)

/**
 * @brief   ��ȡ��������ǰ����
 *
 * @return  ����ֵ  0:ǰ��  1:����
 */
#define ENC_GetCurrentDir       (R8_ENC_REG_CTRL>>5 & 0x01)

/**
 * @brief   ��ȡ��������ǰ����ֵ
 */
#define ENC_GetCurrentCount      R32_ENC_REG_CCNT

/**
 * @brief   ������ģʽ����������0
 */
#define ENC_GetCountandReset()  R8_ENC_REG_CTRL |= RB_RD_CLR_EN

/**
 * @brief   ENC�ж�����
 *
 * @param   s       - ʹ��/�ر�
 * @param   f       - refer to ENC interrupt bit define
 */
#define ENC_ITCfg(s, f)         ((s) ? (R8_ENC_INTER_EN |= f) : (R8_ENC_INTER_EN &= ~f))

/**
 * @brief   ���ENC�жϱ�־
 *
 * @param   f       - refer to ENC interrupt bit define
 */
#define ENC_ClearITFlag(f)      (R8_ENC_INT_FLAG = f)

/**
 * @brief   ��ѯ�жϱ�־״̬
 *
 * @param   f       - refer to ENC interrupt bit define
 */
#define ENC_GetITFlag(f)        (R8_ENC_INT_FLAG & f)

#ifdef __cplusplus
}
#endif

#endif // __CH572_TIMER_H__
