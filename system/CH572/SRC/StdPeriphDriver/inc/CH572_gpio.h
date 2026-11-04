/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH572_gpio.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : head file(ch572/ch570)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __CH572_GPIO_H__
#define __CH572_GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	GPIO_pins_define
 */
#define GPIO_Pin_0      (0x00000001) /*!< Pin 0 selected */
#define GPIO_Pin_1      (0x00000002) /*!< Pin 1 selected */
#define GPIO_Pin_2      (0x00000004) /*!< Pin 2 selected */
#define GPIO_Pin_3      (0x00000008) /*!< Pin 3 selected */
#define GPIO_Pin_4      (0x00000010) /*!< Pin 4 selected */
#define GPIO_Pin_5      (0x00000020) /*!< Pin 5 selected */
#define GPIO_Pin_6      (0x00000040) /*!< Pin 6 selected */
#define GPIO_Pin_7      (0x00000080) /*!< Pin 7 selected */
#define GPIO_Pin_8      (0x00000100) /*!< Pin 8 selected */
#define GPIO_Pin_9      (0x00000200) /*!< Pin 9 selected */
#define GPIO_Pin_10     (0x00000400) /*!< Pin 10 selected */
#define GPIO_Pin_11     (0x00000800) /*!< Pin 11 selected */
#define GPIO_Pin_12     (0x00001000) /*!< Pin 12 selected */
#define GPIO_Pin_13     (0x00002000) /*!< Pin 13 selected */
#define GPIO_Pin_14     (0x00004000) /*!< Pin 14 selected */
#define GPIO_Pin_15     (0x00008000) /*!< Pin 15 selected */
#define GPIO_Pin_16     (0x00010000) /*!< Pin 16 selected */
#define GPIO_Pin_17     (0x00020000) /*!< Pin 17 selected */
#define GPIO_Pin_18     (0x00040000) /*!< Pin 18 selected */
#define GPIO_Pin_19     (0x00080000) /*!< Pin 19 selected */
#define GPIO_Pin_20     (0x00100000) /*!< Pin 20 selected */
#define GPIO_Pin_21     (0x00200000) /*!< Pin 21 selected */
#define GPIO_Pin_22     (0x00400000) /*!< Pin 22 selected */
#define GPIO_Pin_23     (0x00800000) /*!< Pin 23 selected */
#define GPIO_Pin_All    (0xFFFFFFFF) /*!< All pins selected */

/**
 * @brief   GPIO_pins_remap_define
 */
#define REMAP_RXD_PA2   0x00  /*!<Ĭ��ӳ�䣨RXD/PA2�� */
#define REMAP_RXD_PA3   0x01  /*!<��ӳ�䣨RXD/PA3�� */
#define REMAP_RXD_PA0   0x02  /*!<��ӳ�䣨RXD/PA0�� */
#define REMAP_RXD_PA1   0x03  /*!<��ӳ�䣨RXD/PA1�� */
#define REMAP_RXD_PA4   0x04  /*!<��ӳ�䣨RXD/PA4�� */
#define REMAP_RXD_PA9   0x05  /*!<��ӳ�䣨RXD/PA9�� */
#define REMAP_RXD_PA10  0x06  /*!<��ӳ�䣨RXD/PA10�� */
#define REMAP_RXD_PA11  0x07  /*!<��ӳ�䣨RXD/PA11�� */

#define REMAP_TXD_PA3   0x00  /*!<Ĭ��ӳ�䣨TXD/PA3�� */
#define REMAP_TXD_PA2   0x08  /*!<��ӳ�䣨TXD/PA2�� */
#define REMAP_TXD_PA1   0x10  /*!<��ӳ�䣨TXD/PA1�� */
#define REMAP_TXD_PA0   0x18  /*!<��ӳ�䣨TXD/PA0�� */
#define REMAP_TXD_PA7   0x20  /*!<��ӳ�䣨TXD/PA7�� */
#define REMAP_TXD_PA8   0x28  /*!<��ӳ�䣨TXD/PA8�� */
#define REMAP_TXD_PA11  0x30  /*!<��ӳ�䣨TXD/PA11�� */
#define REMAP_TXD_PA10  0x38  /*!<��ӳ�䣨TXD/PA10�� */

#define REMAP_TMR_DEFAULT   0x00   /*!<Ĭ��ӳ�䣨PWM0/PA7��CAP_IN1/PA7��CAP_IN2/PA2�� */
#define REMAP_TMR_MODE1     0x40   /*!<��ӳ��1 ��PWM0/PA2��CAP_IN1/PA2��CAP_IN2/PA7�� */
#define REMAP_TMR_MODE2     0x80   /*!<��ӳ��2��PWM0/PA4��CAP_IN1/PA4��CAP_IN2/PA9�� */
#define REMAP_TMR_MODE3     0xC0   /*!<��ӳ��3��PWM0/PA9��CAP_IN1/PA9��CAP_IN2/PA4�� */

#define REMAP_I2C_DEFAULT   0x00  /*!<Ĭ��ӳ�䣨SCL/PA8��SDA/PA9�� */
#define REMAP_I2C_MODE1     0x200 /*!<��ӳ��1 ��SCL/PA0��SDA/PA1�� */
#define REMAP_I2C_MODE2     0x400 /*!<��ӳ��2 ��SCL/PA3��SDA/PA2�� */
#define REMAP_I2C_MODE3     0x600 /*!<��ӳ��3 ��SCL/PA5��SDA/PA6�� */

/**
 * @brief  Configuration GPIO Mode
 */
typedef enum
{
    GPIO_ModeIN_Floating, //��������
    GPIO_ModeIN_PU,       //��������
    GPIO_ModeIN_PD,       //��������
    GPIO_ModeOut_PP_5mA,  //����������5mA
    GPIO_ModeOut_PP_20mA, //����������20mA

} GPIOModeTypeDef;

/**
 * @brief  Configuration GPIO IT Mode
 */
typedef enum
{
    GPIO_ITMode_LowLevel,  //�͵�ƽ����
    GPIO_ITMode_HighLevel, //�ߵ�ƽ����
    GPIO_ITMode_FallEdge,  //�½��ش���
    GPIO_ITMode_RiseEdge,  //�����ش���

} GPIOITModeTpDef;

/**
 * @brief   GPIOA�˿�����ģʽ����
 *
 * @param   pin     - PA0-PA15
 * @param   mode    - �����������
 */
void GPIOA_ModeCfg(uint32_t pin, GPIOModeTypeDef mode);

/**
 * @brief   GPIOA�˿���������õ�
 *
 * @param   pin     - PA0-PA15
 */
#define GPIOA_ResetBits(pin)      (R32_PA_CLR = pin)

/**
 * @brief   GPIOA�˿���������ø�
 *
 * @param   pin     - PA0-PA15
 */
#define GPIOA_SetBits(pin)        (R32_PA_SET = pin)

/**
 * @brief   GPIOA�˿����������ƽ��ת
 *
 * @param   pin     - PA0-PA15
 */
#define GPIOA_InverseBits(pin)    (R32_PA_OUT ^= pin)

/**
 * @brief   GPIOA�˿�32λ���ݷ��أ���16λ��Ч
 *
 * @return  GPIOA�˿�32λ����
 */
#define GPIOA_ReadPort()          (R32_PA_PIN)

/**
 * @brief   GPIOA�˿�����״̬��0-���ŵ͵�ƽ��(!0)-���Ÿߵ�ƽ
 *
 * @param   pin     - PA0-PA15
 *
 * @return  GPIOA�˿�����״̬
 */
#define GPIOA_ReadPortPin(pin)    (R32_PA_PIN & (pin))

/**
 * @brief   GPIOA�����ж�ģʽ����
 *
 * @param   pin     - PA0-PA15
 * @param   mode    - ��������
 */
void GPIOA_ITModeCfg(uint32_t pin, GPIOITModeTpDef mode);

/**
 * @brief   ��ȡGPIOA�˿��жϱ�־״̬
 *
 * @return  GPIOA�˿��жϱ�־״̬
 */
#define GPIOA_ReadITFlagPort()       (R16_PA_INT_IF)

/**
 * @brief   ��ȡGPIOA�˿������жϱ�־״̬
 *
 * @param   pin     - PA0-PA15
 *
 * @return  GPIOA�˿������жϱ�־״̬
 */
#define GPIOA_ReadITFlagBit(pin)     (R16_PA_INT_IF & (pin))

/**
 * @brief   ���GPIOA�˿������жϱ�־״̬
 *
 * @param   pin     - PA0-PA15
 */
#define GPIOA_ClearITFlagBit(pin)    (R16_PA_INT_IF = pin)

/**
 * @brief   ���蹦������ӳ��
 *
 * @param   s       - �Ƿ�ʹ��ӳ��
 * @param   perph   - д�����ӳ���ϵ�����GPIO_pins_remap_define
 */
void GPIOPinRemap(FunctionalState s, uint16_t perph);

/**
 * @brief   I/O pin���ֹ��ܿ���
 *
 * @param   s       - �Ƿ�򿪶�ӦI/O pin���ֹ���
 * @param   pin     - PA0-PA15
 */
void GPIOADigitalCfg(FunctionalState s, uint16_t pin);


#ifdef __cplusplus
}
#endif

#endif // __CH572_GPIO_H__
