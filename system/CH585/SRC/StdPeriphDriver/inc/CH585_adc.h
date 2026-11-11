/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH585_adc.h
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2021/11/17
 * Description        : head file(ch585/ch584)
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#ifndef __CH585_ADC_H__
#define __CH585_ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

#define ROM_CFG_TMP_25C    0x7F014

/**
 * @brief  adc single channel define
 */
typedef enum
{
    CH_EXTIN_0 = 0,   // ADC �ⲿģ��ͨ�� 0
    CH_EXTIN_1,       // ADC �ⲿģ��ͨ�� 1
    CH_EXTIN_2,       // ADC �ⲿģ��ͨ�� 2
    CH_EXTIN_3,       // ADC �ⲿģ��ͨ�� 3
    CH_EXTIN_4,       // ADC �ⲿģ��ͨ�� 4
    CH_EXTIN_5,       // ADC �ⲿģ��ͨ�� 5
    CH_EXTIN_6,       // ADC �ⲿģ��ͨ�� 6
    CH_EXTIN_7,       // ADC �ⲿģ��ͨ�� 7
    CH_EXTIN_8,       // ADC �ⲿģ��ͨ�� 8
    CH_EXTIN_9,       // ADC �ⲿģ��ͨ�� 9
    CH_EXTIN_10,      // ADC �ⲿģ��ͨ�� 10
    CH_EXTIN_11,      // ADC �ⲿģ��ͨ�� 11
    CH_EXTIN_12,      // ADC �ⲿģ��ͨ�� 12
    CH_EXTIN_13,      // ADC �ⲿģ��ͨ�� 13

    CH_INTE_VBAT = 14,  // ADC �ڲ���ؼ��ͨ��
    CH_INTE_VTEMP = 15, // ADC �ڲ��¶ȴ��������ͨ��
    CH_INTE_NFC = 16,   // NFC �ڲ��źż��ͨ��

} ADC_SingleChannelTypeDef;

/**
 * @brief  adc differential channel define
 */
typedef enum
{
    CH_DIFF_0_2 = 0, // ADC ���ͨ�� #0-#2
    CH_DIFF_1_3,     // ADC ���ͨ�� #1-#3

} ADC_DiffChannelTypeDef;

/**
 * @brief  adc sampling clock, depends on R16_CLK_SYS_CFG[9] = 1/0
 */
typedef enum
{
    SampleFreq_8 = 0,           // 8M ����Ƶ��   R16_CLK_SYS_CFG[9]=1ʱ������
    SampleFreq_8_or_4,          // 8/4M ����Ƶ��  R16_CLK_SYS_CFG[9]=1ʱΪ8M����֮Ϊ4M����ͬ
    SampleFreq_5_33_or_2_67,    // 5.33/2.67M ����Ƶ��
    SampleFreq_4_or_2,          // 4/2M ����Ƶ��
} ADC_SampClkTypeDef;

/**
 * @brief  adc signal PGA
 */
typedef enum
{
    ADC_PGA_1_4 = 0,    // -12dB, 1/4��
    ADC_PGA_1_2,        // -6dB, 1/2��
    ADC_PGA_0,          // 0dB, 1����������
    ADC_PGA_2,          // 6dB, 2��
    ADC_PGA_2_ = 0x10,  // 6dB, 2��
    ADC_PGA_4,          // 12dB, 4��
    ADC_PGA_8,          // 18dB, 8��
    ADC_PGA_16,         // 24dB, 16��
} ADC_SignalPGATypeDef;

/**
 * @brief  Configuration DMA mode
 */
typedef enum
{
    ADC_Mode_Single = 0, // ����ģʽ
    ADC_Mode_LOOP,       // ѭ��ģʽ
} ADC_DMAModeTypeDef;


/**
 * @brief   ���� ADC ����ͨ��
 *
 * @param   d   - refer to ADC_SingleChannelTypeDef
 */
#define ADC_ChannelCfg(d)      (R8_ADC_CHANNEL = d)

/**
 * @brief   ���� ADC ����ʱ��
 *
 * @param   d   - refer to ADC_SampClkTypeDef
 */
#define ADC_SampClkCfg(d)      (R8_ADC_CFG = R8_ADC_CFG & (~RB_ADC_CLK_DIV) | (d << 6))

/**
 * @brief   ���� ADC �ź�����
 *
 * @param   d   - refer to ADC_SignalPGATypeDef
 */
#define ADC_PGACfg(d)          (R8_ADC_CFG = R8_ADC_CFG & (~RB_ADC_PGA_GAIN) | (d << 4))

/**
 * @brief   �����ڲ��¶ȴ�����У׼ֵ
 *
 * @param   d   - У׼ֵ
 */
#define ADC_TempCalibCfg(d)    (R8_TEM_SENSOR = R8_TEM_SENSOR & (~RB_TEM_SEN_CALIB) | d)

/**
 * @brief   �ⲿ�źŵ�ͨ��������ʼ��
 *
 * @param   sp  - refer to ADC_SampClkTypeDef
 * @param   ga  - refer to ADC_SignalPGATypeDef
 */
void ADC_ExtSingleChSampInit(ADC_SampClkTypeDef sp, ADC_SignalPGATypeDef ga);

/**
 * @brief   �ⲿ�źŲ��ͨ��������ʼ��
 *
 * @param   sp  - refer to ADC_SampClkTypeDef
 * @param   ga  - refer to ADC_SignalPGATypeDef
 */
void ADC_ExtDiffChSampInit(ADC_SampClkTypeDef sp, ADC_SignalPGATypeDef ga);

/**
 * @brief   ��������ͨ��������ʼ��
 */
void TouchKey_ChSampInit(void);

/**
 * @brief   �ر�TouchKey��Դ
 */
#define TouchKey_DisableTSPower()    (R8_TKEY_CFG &= ~RB_TKEY_PWR_ON)

/**
 * @brief   �����¶ȴ�����������ʼ��
 */
void ADC_InterTSSampInit(void);

/**
 * @brief   �ر��¶ȴ�������Դ
 */
#define ADC_DisableTSPower()    (R8_TEM_SENSOR = 0)

/**
 * @brief   ���õ�ص�ѹ������ʼ��
 */
void ADC_InterBATSampInit(void);

/**
 * @brief   ADCִ�е���ת��
 *
 * @return  ADCת���������
 */
uint16_t ADC_ExcutSingleConver(void);

/**
 * @brief   �������ݴֵ�,��ȡƫ��ֵ,����������ADC����ô˺�����ȡУ׼ֵ
 *
 * @return  ƫ��
 */
signed short ADC_DataCalib_Rough(void);

/**
 * @brief   TouchKeyת��������
 *
 * @param   charg   - Touchkey���ʱ��,5bits��Ч, t=charg*Tadc
 * @param   disch   - Touchkey�ŵ�ʱ��,3bits��Ч, t=disch*Tadc
 *
 * @return  ��ǰTouchKey��Ч����
 */
uint16_t TouchKey_ExcutSingleConver(uint8_t charg, uint8_t disch);

/**
 * @brief   �������� ADC������
 *
 * @param   cycle   - ��λΪ 16��ϵͳʱ��
 */
void ADC_AutoConverCycle(uint8_t cycle);

/**
 * @brief   ����DMA����
 *
 * @param   s           - �Ƿ��DMA����
 * @param   startAddr   - DMA ��ʼ��ַ
 * @param   endAddr     - DMA ������ַ
 * @param   m           - ����DMAģʽ
 */
void ADC_DMACfg(uint8_t s, uint32_t startAddr, uint32_t endAddr, ADC_DMAModeTypeDef m);

/**
 * @brief   Convert ADC value to temperature(Celsius)
 *
 * @param   adc_val - adc value
 *
 * @return  temperature (Celsius)
 */
int adc_to_temperature_celsius(uint16_t adc_val);

/**
 * @brief   -12dB����ʱADC���˲���ֵת���ɵ�ѹ(mV)
 *
 * @param   adc_data - ADC����ֵ
 *
 * @return  ��ѹ(mV)
 */
int ADC_VoltConverSignalPGA_MINUS_12dB(uint16_t adc_data);


/**
 * @brief   -6dB����ʱADC���˲���ֵת���ɵ�ѹ(mV)
 *
 * @param   adc_data - ADC����ֵ
 *
 * @return  ��ѹ(mV)
 */
int ADC_VoltConverSignalPGA_MINUS_6dB(uint16_t adc_data);

/**
 * @brief   0dB����ʱADC���˲���ֵת���ɵ�ѹ(mV)
 *
 * @param   adc_data - ADC����ֵ
 *
 * @return  ��ѹ(mV)
 */
int ADC_VoltConverSignalPGA_0dB(uint16_t adc_data);

/**
 * @brief   6dB����ʱADC���˲���ֵת���ɵ�ѹ(mV)
 *
 * @param   adc_data - ADC����ֵ
 *
 * @return  ��ѹ(mV)
 */
int ADC_VoltConverSignalPGA_6dB(uint16_t adc_data);

/**
 * @brief   12dB����ʱADC���˲���ֵת���ɵ�ѹ(mV)
 *
 * @param   adc_data - ADC����ֵ
 *
 * @return  ��ѹ(mV)
 */
int ADC_VoltConverSignalPGA_12dB(uint16_t adc_data);

/**
 * @brief   18dB����ʱADC���˲���ֵת���ɵ�ѹ(mV)
 *
 * @param   adc_data - ADC����ֵ
 *
 * @return  ��ѹ(mV)
 */
int ADC_VoltConverSignalPGA_18dB(uint16_t adc_data);

/**
 * @brief   24dB����ʱADC���˲���ֵת���ɵ�ѹ(mV)
 *
 * @param   adc_data - ADC����ֵ
 *
 * @return  ��ѹ(mV)
 */
int ADC_VoltConverSignalPGA_24dB(uint16_t adc_data);

/**
 * @brief   -12dB����ʱADC��ֲ���ֵת���ɵ�ѹ(mV)
 *
 * @param   adc_data - ADC����ֵ
 *
 * @return  ��ѹ(mV)
 */
int ADC_VoltConverDiffPGA_MINUS_12dB(uint16_t adc_data);

/**
 * @brief   -6dB����ʱADC��ֲ���ֵת���ɵ�ѹ(mV)
 *
 * @param   adc_data - ADC����ֵ
 *
 * @return  ��ѹ(mV)
 */
int ADC_VoltConverDiffPGA_MINUS_6dB(uint16_t adc_data);

/**
 * @brief   0dB����ʱADC��ֲ���ֵת���ɵ�ѹ(mV)
 *
 * @param   adc_data - ADC����ֵ
 *
 * @return  ��ѹ(mV)
 */
int ADC_VoltConverDiffPGA_0dB(uint16_t adc_data);

/**
 * @brief   6dB����ʱADC��ֲ���ֵת���ɵ�ѹ(mV)
 *
 * @param   adc_data - ADC����ֵ
 *
 * @return  ��ѹ(mV)
 */
int ADC_VoltConverDiffPGA_6dB(uint16_t adc_data);

/**
 * @brief   12dB����ʱADC��ֲ���ֵת���ɵ�ѹ(mV)
 *
 * @param   adc_data - ADC����ֵ
 *
 * @return  ��ѹ(mV)
 */
int ADC_VoltConverDiffPGA_12dB(uint16_t adc_data);

/**
 * @brief   18dB����ʱADC��ֲ���ֵת���ɵ�ѹ(mV)
 *
 * @param   adc_data - ADC����ֵ
 *
 * @return  ��ѹ(mV)
 */
int ADC_VoltConverDiffPGA_18dB(uint16_t adc_data);

/**
 * @brief   24dB����ʱADC��ֲ���ֵת���ɵ�ѹ(mV)
 *
 * @param   adc_data - ADC����ֵ
 *
 * @return  ��ѹ(mV)
 */
int ADC_VoltConverDiffPGA_24dB(uint16_t adc_data);
/**
 * @brief   ��ȡADCת��ֵ
 *
 * @return  ADCת��ֵ
 */
#define ADC_ReadConverValue()     (R16_ADC_DATA)

/**
 * @brief   ADCִ�е���ת��
 */
#define ADC_StartUp()             (R8_ADC_CONVERT |= RB_ADC_START)

/**
 * @brief   ��ȡADC�ж�״̬
 */
#define ADC_GetITStatus()         (R8_ADC_INT_FLAG & RB_ADC_IF_EOC)

/**
 * @brief   ���ADC�жϱ�־
 */
#define ADC_ClearITFlag()         (R8_ADC_CONVERT = R8_ADC_CONVERT)

/**
 * @brief   ��ȡADC DMA���״̬
 */
#define ADC_GetDMAStatus()        (R8_ADC_DMA_IF & RB_ADC_IF_DMA_END)

/**
 * @brief   ���ADC DMA��ɱ�־
 */
#define ADC_ClearDMAFlag()        (R8_ADC_DMA_IF |= RB_ADC_IF_DMA_END)

/**
 * @brief   ʹ�ܶ�ʱ����Զ����� ADC,�Զ���ʼת��
 */
#define ADC_StartAutoDMA()        (R8_ADC_CTRL_DMA |= RB_ADC_AUTO_EN)

/**
 * @brief   �رն�ʱ����Զ����� ADC
 */
#define ADC_StopAutoDMA()         (R8_ADC_CTRL_DMA &= ~RB_ADC_AUTO_EN)

/**
 * @brief   ʹ������ת�� ADC,��ִ��ADC_StartUp��ʼת��
 */
#define ADC_StartContDMA()        (R8_ADC_CTRL_DMA |= RB_ADC_CONT_EN)

/**
 * @brief   �ر�����ת�� ADC
 */
#define ADC_StopContDMA()         (R8_ADC_CTRL_DMA &= ~RB_ADC_CONT_EN)

/**
 * @brief   �ر�ADC��Դ
 */
#define ADC_DisablePower()        (R8_ADC_CFG &= ~RB_ADC_POWER_ON)

#ifdef __cplusplus
}
#endif

#endif // __CH585_ADC_H__
