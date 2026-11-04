/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH585_gpio.c
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
 * @fn      GPIOA_ModeCfg
 *
 * @brief   GPIOA�˿�����ģʽ����
 *
 * @param   pin     - PA0-PA15
 * @param   mode    - �����������
 *
 * @return  none
 */
void GPIOA_ModeCfg(uint32_t pin, GPIOModeTypeDef mode)
{
    switch(mode)
    {
        case GPIO_ModeIN_Floating:
            R32_PA_PD_DRV &= ~pin;
            R32_PA_PU &= ~pin;
            R32_PA_DIR &= ~pin;
            break;

        case GPIO_ModeIN_PU:
            R32_PA_PD_DRV &= ~pin;
            R32_PA_PU |= pin;
            R32_PA_DIR &= ~pin;
            break;

        case GPIO_ModeIN_PD:
            R32_PA_PD_DRV |= pin;
            R32_PA_PU &= ~pin;
            R32_PA_DIR &= ~pin;
            break;

        case GPIO_ModeOut_PP_5mA:
            R32_PA_PD_DRV &= ~pin;
            R32_PA_DIR |= pin;
            break;

        case GPIO_ModeOut_PP_20mA:
            R32_PA_PD_DRV |= pin;
            R32_PA_DIR |= pin;
            break;

        default:
            break;
    }
}

/*********************************************************************
 * @fn      GPIOB_ModeCfg
 *
 * @brief   GPIOB�˿�����ģʽ����
 *
 * @param   pin     - PB0-PB23
 * @param   mode    - �����������
 *
 * @return  none
 */
void GPIOB_ModeCfg(uint32_t pin, GPIOModeTypeDef mode)
{
    switch(mode)
    {
        case GPIO_ModeIN_Floating:
            R32_PB_PD_DRV &= ~pin;
            R32_PB_PU &= ~pin;
            R32_PB_DIR &= ~pin;
            break;

        case GPIO_ModeIN_PU:
            R32_PB_PD_DRV &= ~pin;
            R32_PB_PU |= pin;
            R32_PB_DIR &= ~pin;
            break;

        case GPIO_ModeIN_PD:
            R32_PB_PD_DRV |= pin;
            R32_PB_PU &= ~pin;
            R32_PB_DIR &= ~pin;
            break;

        case GPIO_ModeOut_PP_5mA:
            R32_PB_PD_DRV &= ~pin;
            R32_PB_DIR |= pin;
            break;

        case GPIO_ModeOut_PP_20mA:
            R32_PB_PD_DRV |= pin;
            R32_PB_DIR |= pin;
            break;

        default:
            break;
    }
}

/*********************************************************************
 * @fn      GPIOA_ITModeCfg
 *
 * @brief   GPIOA�����ж�ģʽ����
 *
 * @param   pin     - PA0-PA15
 * @param   mode    - ��������
 *
 * @return  none
 */
void GPIOA_ITModeCfg(uint32_t pin, GPIOITModeTpDef mode)
{
    switch(mode)
    {
        case GPIO_ITMode_LowLevel: // �͵�ƽ����
            R16_PA_INT_MODE &= ~pin;
            R32_PA_CLR |= pin;
            break;

        case GPIO_ITMode_HighLevel: // �ߵ�ƽ����
            R16_PA_INT_MODE &= ~pin;
            R32_PA_OUT |= pin;
            break;

        case GPIO_ITMode_FallEdge: // �½��ش���
            R16_PA_INT_MODE |= pin;
            R32_PA_CLR |= pin;
            break;

        case GPIO_ITMode_RiseEdge: // �����ش���
            R16_PA_INT_MODE |= pin;
            R32_PA_OUT |= pin;
            break;

        default:
            break;
    }
    R16_PA_INT_IF = pin;
    R16_PA_INT_EN |= pin;
}

/*********************************************************************
 * @fn      GPIOB_ITModeCfg
 *
 * @brief   GPIOB�����ж�ģʽ����
 *
 * @param   pin     - PB0-PB15,PB22-PB23,PB[9:8]��RB_PIN_INTXѡ���ӦPB[23:22]��PB[9:8]
 * @param   mode    - ��������
 *
 * @return  none
 */
void GPIOB_ITModeCfg(uint32_t pin, GPIOITModeTpDef mode)
{
    uint32_t Pin = pin | ((pin & (GPIO_Pin_22 | GPIO_Pin_23)) >> 14);
    switch(mode)
    {
        case GPIO_ITMode_LowLevel: // �͵�ƽ����
            R16_PB_INT_MODE &= ~Pin;
            R32_PB_CLR |= pin;
            break;

        case GPIO_ITMode_HighLevel: // �ߵ�ƽ����
            R16_PB_INT_MODE &= ~Pin;
            R32_PB_OUT |= pin;
            break;

        case GPIO_ITMode_FallEdge: // �½��ش���
            R16_PB_INT_MODE |= Pin;
            R32_PB_CLR |= pin;
            break;

        case GPIO_ITMode_RiseEdge: // �����ش���
            R16_PB_INT_MODE |= Pin;
            R32_PB_OUT |= pin;
            break;

        default:
            break;
    }
    R16_PB_INT_IF = Pin;
    R16_PB_INT_EN |= Pin;
}

/*********************************************************************
 * @fn      GPIOPinRemap
 *
 * @brief   ���蹦������ӳ��
 *
 * @param   s       - �Ƿ�ʹ��ӳ��
 * @param   perph   - RB_RF_ANT_SW_EN -  RF antenna switch control output on PA4/PA5/PA12/PA13/PA14/PA15
 *                    RB_PIN_U0_INV -  RXD0/RXD0_/TXD0/TXD0_ invert input/output
 *                    RB_PIN_INTX   -  INTX: INT24/INT25 PB8/PB9 -> INT24_/INT25_ PB22/PB23
 *                    RB_PIN_MODEM  -  MODEM: PA6/PA7 -> PB12/PB13
 *                    RB_PIN_I2C    -  I2C: PB14/PB15 -> PB14/PB15
 *                    RB_PIN_PWMX   -  PWMX: PA12/PA13/PB7/PB6/PB4 -> PA6/PA7/PB3/PB2/PB1
 *                    RB_PIN_SPI0   -  SPI0:  PA12/PA13/PA14/PA15 -> PB12/PB13/PB14/PB15
 *                    RB_PIN_UART3  -  UART3: PA4/PA5 ->  PA4/PA5
 *                    RB_PIN_UART2  -  UART2: PB22/PB23 ->  PA6/PA7
 *                    RB_PIN_UART1  -  UART1: PA8/PA9 ->  PB12/PB13
 *                    RB_PIN_UART0  -  UART0: PB4/PB7 ->  PA15/PA14
 *                    RB_PIN_TMR3   -  TMR2:  PB22 ->  PB22
 *                    RB_PIN_TMR2   -  TMR2:  PA11 ->  PB11
 *                    RB_PIN_TMR1   -  TMR1:  PA10 ->  PB10
 *                    RB_PIN_TMR0   -  TMR0:  PA9 ->  PB23
 *
 * @return  none
 */
void GPIOPinRemap(FunctionalState s, uint16_t perph)
{
    if(s)
    {
        R16_PIN_ALTERNATE |= perph;
    }
    else
    {
        R16_PIN_ALTERNATE &= ~perph;
    }
}

/*********************************************************************
 * @fn      GPIOAGPPCfg
 *
 * @brief   ģ������GPIO���Ź��ܿ���
 *
 * @param   s       -   ENABLE  - ��ģ�����蹦�ܣ��ر����ֹ���
 *                      DISABLE - �������ֹ��ܣ��ر�ģ�����蹦��
 * @param   perph   -   RB_DEBUG_PIN_SEL  - SWDIO/SWCLK alternate pin enable: 0=SWDIO/SWCLK on PA[8]/PA[9], 1=SWDIO/SWCLK on PA[14]/PA[15]
 *                      RB_PB16_8_SEL  - interupt pin select,0=PB8, 1=PB16
 *                      RB_PIN_USB2_EN  - USB high speed comunication pin select, 0=PB12/13 not use, 1=PB12/13 used
 *                      RB_UDP_PU_EN  - enable USB ud+ pin pulled upresistance , 0=control by RB_UC_DEV_PU_EN, 1=force pulled up
 *                      RB_PIN_USB_EN  - enable USB fast speed pin, 0=PB10/PB11 not used, 1=PB10/PB11 used
 *
 * @return  none
 */
void GPIOAGPPCfg(FunctionalState s, uint16_t perph)
{
    if(s)
    {
        R16_PIN_CONFIG |= perph;
    }
    else
    {
        R16_PIN_CONFIG &= ~perph;
    }
}


/*********************************************************************
 * @fn      GPIOADigitalCfg
 *
 * @brief   I/O pin���ֹ��ܿ���
 *
 * @param   s       - �Ƿ�򿪶�ӦI/O pin���ֹ���
 * @param   pin     - PA0-PA15
 */
void GPIOADigitalCfg(FunctionalState s, uint16_t pin)
{
    if(s)
    {
        R32_PIN_IN_DIS &= ~pin;
    }
    else
    {
        R32_PIN_IN_DIS |= pin;
    }
}

/*********************************************************************
 * @fn      GPIOADigitalCfg
 *
 * @brief   I/O pin���ֹ��ܿ���
 *
 * @param   s       - �Ƿ�򿪶�ӦI/O pin���ֹ���
 * @param   pin     - PB0-PB23
 */
void GPIOBDigitalCfg(FunctionalState s, uint32_t pin)
{
    uint32_t dis_pin;
    uint16_t config_pin;

    dis_pin = (pin << 16) & 0xffff0000;
    config_pin = (pin >> 8) & 0xff00;

    if(s)
    {
        if(dis_pin)
        {
            R32_PIN_IN_DIS &= ~dis_pin;
        }
        if(config_pin)
        {
            R16_PIN_CONFIG &= ~config_pin;
        }
    }
    else
    {
        if(dis_pin)
        {
            R32_PIN_IN_DIS |= dis_pin;
        }
        if(config_pin)
        {
            R16_PIN_CONFIG |= config_pin;
        }
    }
}
