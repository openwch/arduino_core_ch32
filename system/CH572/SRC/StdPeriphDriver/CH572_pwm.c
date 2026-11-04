/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH572_pwm.c
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
 * @fn      PWMX_CycleCfg
 *
 * @brief   PWM 8λ��������
 *
 * @param   cyc     - refer to PWMX_CycleTypeDef
 *
 * @return  none
 */
void PWMX_CycleCfg(PWMX_CycleTypeDef cyc)
{
    switch(cyc)
    {
        case PWMX_Cycle_256:
            R8_PWM_CONFIG &= ~(0x07);
            break;

        case PWMX_Cycle_255:
            R8_PWM_CONFIG |= 0x01;
            break;

        case PWMX_Cycle_128:
            R8_PWM_CONFIG |= (1 << 1);
            break;

        case PWMX_Cycle_127:
            R8_PWM_CONFIG |= (1 << 1) | 0x01;
            break;

        case PWMX_Cycle_64:
            R8_PWM_CONFIG |= (2 << 1);
            break;

        case PWMX_Cycle_63:
            R8_PWM_CONFIG |= (2 << 1) | 0x01;
            break;

        default:
            break;
    }
}

/*********************************************************************
 * @fn      PWMX_16bit_CycleCfg
 *
 * @brief   PWM 16λ��������
 *
 * @param   cyc     - 16λ����
 *
 * @return  none
 */
void PWMX_16bit_CycleCfg(uint8_t ch, uint16_t cyc)
{
    if(ch & (CH_PWM1|CH_PWM2|CH_PWM3))
    {
        R16_PWM_CYC_VALUE = cyc;
    }
    if(ch & (CH_PWM4|CH_PWM5))
    {
        R16_PWM_CYC1_VALUE = cyc;
    }
}

/*********************************************************************
 * @fn      PWMX_16bit_ACTOUT
 *
 * @brief   PWM 16λ�����������
 *
 * @param   ch      - select channel of pwm, refer to channel of PWM define
 * @param   da      - effective pulse width
 * @param   pr      - select wave polar, refer to PWMX_PolarTypeDef
 * @param   s       - control pwmx function, ENABLE or DISABLE
 *
 * @return  none
 */
void PWMX_16bit_ACTOUT(uint8_t ch, uint16_t da, PWMX_PolarTypeDef pr, FunctionalState s)
{
    uint8_t i;

    if(s == DISABLE)
    {
        R8_PWM_OUT_EN &= ~(ch);
    }
    else
    {
        (pr) ? (R8_PWM_POLAR |= (ch)) : (R8_PWM_POLAR &= ~(ch));
        for(i = 0; i < 5; i++)
        {
            if((ch >> i) & 1)
            {
                if(i<3)
                {
                    *((volatile uint16_t *)((&R16_PWM1_DATA) + i)) = da;
                }
                else
                {
                    *((volatile uint16_t *)((&R16_PWM4_DATA) + (i-3))) = da;
                }
            }
        }
        R8_PWM_OUT_EN |= (ch);
    }
}

/*********************************************************************
 * @fn      PWMX_ACTOUT
 *
 * @brief   PWM 8λ�����������
 *
 * @param   ch      - select channel of pwm, refer to channel of PWM define
 * @param   da      - effective pulse width
 * @param   pr      - select wave polar, refer to PWMX_PolarTypeDef
 * @param   s       - control pwmx function, ENABLE or DISABLE
 *
 * @return  none
 */
void PWMX_ACTOUT(uint8_t ch, uint8_t da, PWMX_PolarTypeDef pr, FunctionalState s)
{
    uint8_t i;

    if(s == DISABLE)
    {
        R8_PWM_OUT_EN &= ~(ch);
    }
    else
    {
        (pr) ? (R8_PWM_POLAR |= (ch)) : (R8_PWM_POLAR &= ~(ch));
        switch(ch)
        {
            case CH_PWM1:
                *((volatile uint16_t *)((&R8_PWM1_DATA))) = da;
                break;
            case CH_PWM2:
                *((volatile uint16_t *)((&R8_PWM2_DATA))) = da;
                break;
            case CH_PWM3:
                *((volatile uint16_t *)((&R8_PWM3_DATA))) = da;
                break;
            case CH_PWM4:
                *((volatile uint16_t *)((&R8_PWM4_DATA))) = da;
                break;
            case CH_PWM5:
                *((volatile uint16_t *)((&R8_PWM5_DATA))) = da;
                break;
            default:
                break;
        }
        R8_PWM_OUT_EN |= (ch);
    }
}

/*********************************************************************
 * @fn      PWMX_AlterOutCfg
 *
 * @brief   PWM �������ģʽ����
 *
 * @param   ch      - select group of PWM alternate output
 *                    RB_PWM4_5_STAG_EN     -  PWM4 �� PWM5 ͨ���������
 * @param   s       - control pwmx function, ENABLE or DISABLE
 *
 * @return  none
 */
void PWMX_AlterOutCfg(uint8_t ch, FunctionalState s)
{
    if(s == DISABLE)
    {
        R8_PWM_CONFIG &= ~(ch);
    }
    else
    {
        R8_PWM_CONFIG |= (ch);
    }
}

/*********************************************************************
 * @fn      PWMX_SyncOutCfg
 *
 * @brief   PWM ͬ�����ģʽ����
 *
 * @param   s       - control pwmx function, ENABLE or DISABLE
 *
 * @return  none
 */
void PWMX_SyncOutCfg(FunctionalState s)
{
    if(s == DISABLE)
    {
        R8_PWM_CONFIG &= ~(RB_PWM_SYNC_EN);
    }
    else
    {
        R8_PWM_CONFIG |= RB_PWM_SYNC_EN;
        R8_PWM_CONFIG |= RB_PWM_SYNC_START;
    }
}

/*********************************************************************
 * @fn      PWM_DMACfg
 *
 * @brief   ����PWM DMA����
 *
 * @param   s           - �Ƿ��DMA����
 * @param   startAddr   - DMA ��ʼ��ַ
 * @param   endAddr     - DMA ������ַ
 * @param   m           - ����DMAģʽ
 * @param   ch          - ����DMA���ͨ��
 *
 * @return  none
 */
void PWM_DMACfg(uint8_t s, uint32_t startAddr, uint32_t endAddr, PWM_DMAModeTypeDef m, PWM_DMAChannel ch)
{
    if(s == DISABLE)
    {
        R8_PWM_DMA_CTRL = 0;
    }
    else
    {
        R32_PWM_DMA_BEG = startAddr & 0xFFFF;
        R32_PWM_DMA_END = endAddr & 0xFFFF;

        if(m)
            R8_PWM_DMA_CTRL = RB_DMA_ADDR_LOOP | RB_DMA_ENABLE;
        else
            R8_PWM_DMA_CTRL = RB_DMA_ENABLE;

        if(ch != Mode_DMACH4_5)
        {
           if(ch == Mode_DMACH1_3)  R8_PWM_DMA_CTRL |= RB_DMA_SEL;
           else   R8_PWM_CONFIG |= RB_PWM_SYNC_EN | RB_PWM_SYNC_START;
        }
    }
}
