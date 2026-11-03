/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH572_timer0.c
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

/*********************************************************************
 * @fn      TMR_TimerInit
 *
 * @brief   ��ʱ���ܳ�ʼ��
 *
 * @param   t       - ��ʱʱ�䣬���ڵ�ǰϵͳʱ��Tsys, ���ʱ���� 67108864
 *
 * @return  none
 */
void TMR_TimerInit(uint32_t t)
{
    R32_TMR_CNT_END = t;
    R8_TMR_CTRL_MOD = RB_TMR_ALL_CLEAR;
    R8_TMR_CTRL_MOD = RB_TMR_COUNT_EN;
}

/*********************************************************************
 * @fn      TMR_EXTSingleCounterInit
 *
 * @brief   ���ؼ������ܳ�ʼ��
 *
 * @param   cap     - �ɼ���������
 *
 * @return  none
 */
void TMR_EXTSingleCounterInit(CapModeTypeDef cap)
{
    R8_TMR_CTRL_MOD = RB_TMR_ALL_CLEAR;
    R8_TMR_CTRL_MOD = RB_TMR_COUNT_EN | RB_TMR_CAP_COUNT | RB_TMR_MODE_IN | (cap << 6);
}

/*********************************************************************
 * @fn      TMR_PWMInit
 *
 * @brief   PWM �����ʼ��
 *
 * @param   pr      - select wave polar, refer to PWMX_PolarTypeDef
 * @param   ts      - set pwm repeat times, refer to PWM_RepeatTsTypeDef
 *
 * @return  none
 */
void TMR_PWMInit(PWMX_PolarTypeDef pr, PWM_RepeatTsTypeDef ts)
{
    R8_TMR_CTRL_MOD = RB_TMR_ALL_CLEAR;
    R8_TMR_CTRL_MOD = (pr << 4) | (ts << 6);
}

/*********************************************************************
 * @fn      TMR_CapInit
 *
 * @brief   �ⲿ�źŲ�׽���ܳ�ʼ��
 *
 * @param   cap     - select capture mode, refer to CapModeTypeDef
 *
 * @return  none
 */
void TMR_CapInit(CapModeTypeDef cap)
{
    R8_TMR_CTRL_MOD = RB_TMR_ALL_CLEAR;
    R8_TMR_CTRL_MOD = RB_TMR_COUNT_EN | RB_TMR_MODE_IN | (cap << 6);
}


/*********************************************************************
 * @fn      TMR_DMACfg
 *
 * @brief   ����DMA����
 *
 * @param   s           - �Ƿ��DMA����
 * @param   startAddr   - DMA ��ʼ��ַ
 * @param   endAddr     - DMA ������ַ
 * @param   m           - ����DMAģʽ
 *
 * @return  none
 */
void TMR_DMACfg(uint8_t s, uint32_t startAddr, uint32_t endAddr, DMAModeTypeDef m)
{
    if(s == DISABLE)
    {
        R8_TMR_CTRL_DMA = 0;
    }
    else
    {
        R16_TMR_DMA_BEG = startAddr & 0xFFFF;
        R16_TMR_DMA_END = endAddr & 0xFFFF;
        if(m)
            R8_TMR_CTRL_DMA = RB_TMR_DMA_LOOP | RB_TMR_DMA_ENABLE;
        else
            R8_TMR_CTRL_DMA = RB_TMR_DMA_ENABLE;
    }
}


/*********************************************************************
 * @fn      ENC_Config
 *
 * @brief   ���ñ���������
 *
 * @param   s           - �Ƿ�������������
 * @param   encReg      - ������ģʽ��ֵ(���ֵ0xFFFF)
 * @param   m           - ����ENCģʽ
 *
 * @return  none
 */
void ENC_Config(uint8_t s, uint32_t encReg, ENCModeTypeDef m)
{
    if(s == DISABLE)
    {
        R8_ENC_REG_CTRL &= ~(RB_START_ENC_EN);
    }
    else
    {
        R8_ENC_REG_CTRL |= (m << 1) | RB_START_ENC_EN;
        R32_ENC_REG_CEND |= encReg;
    }
}


