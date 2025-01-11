/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32x035_adc.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2023/04/06
 * Description        : This file contains all the functions prototypes for the
 *                      ADC firmware library.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32X035_ADC_H
#define __CH32X035_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ch32x035.h"

/* ADC Init structure definition */
typedef struct
{
    uint32_t ADC_Mode; /* Configures the ADC to operate in independent or
                          dual mode.
                          This parameter can be a value of @ref ADC_mode */

    FunctionalState ADC_ScanConvMode; /* Specifies whether the conversion is performed in
                                         Scan (multichannels) or Single (one channel) mode.
                                         This parameter can be set to ENABLE or DISABLE */

    FunctionalState ADC_ContinuousConvMode; /* Specifies whether the conversion is performed in
                                               Continuous or Single mode.
                                               This parameter can be set to ENABLE or DISABLE. */

    uint32_t ADC_ExternalTrigConv; /* Defines the external trigger used to start the analog
                                      to digital conversion of regular channels. This parameter
                                      can be a value of @ref ADC_external_trigger_sources_for_regular_channels_conversion */

    uint32_t ADC_DataAlign; /* Specifies whether the ADC data alignment is left or right.
                               This parameter can be a value of @ref ADC_data_align, Note:ADC_DataAlign_Left only applies to regular channels */

    uint8_t ADC_NbrOfChannel; /* Specifies the number of ADC channels that will be converted
                                   using the sequencer for regular channel group.
                                   This parameter must range from 1 to 16. */

    uint32_t ADC_OutputBuffer; /* Specifies whether the ADC channel output buffer is enabled or disabled.
                                    This parameter can be a value of @ref ADC_OutputBuffer */

    uint32_t ADC_Pga; /* Specifies the PGA gain multiple.
                           This parameter can be a value of @ref ADC_Pga */
} ADC_InitTypeDef;

/* ADC_mode */
#define ADC_Mode_Independent                           ((uint32_t)0x00000000)

/* ADC_external_trigger_sources_for_regular_channels_conversion */
#define ADC_ExternalTrigConv_T1_TRGO                   ((uint32_t)0x00000000)
#define ADC_ExternalTrigConv_T1_CC1                    ((uint32_t)0x00020000)
#define ADC_ExternalTrigConv_T1_CC2                    ((uint32_t)0x00040000)
#define ADC_ExternalTrigConv_T2_TRGO                   ((uint32_t)0x00060000)
#define ADC_ExternalTrigConv_T2_CC1                    ((uint32_t)0x00080000)
#define ADC_ExternalTrigConv_T3_CC1                    ((uint32_t)0x000A0000)
#define ADC_ExternalTrigConv_Ext_IT11                  ((uint32_t)0x000C0000)
#define ADC_ExternalTrigConv_None                      ((uint32_t)0x000E0000)

/* ADC_data_align */
#define ADC_DataAlign_Right                            ((uint32_t)0x00000000)
#define ADC_DataAlign_Left                             ((uint32_t)0x00000800)

/* ADC_channels */
#define ADC_Channel_0                                  ((uint8_t)0x00)
#define ADC_Channel_1                                  ((uint8_t)0x01)
#define ADC_Channel_2                                  ((uint8_t)0x02)
#define ADC_Channel_3                                  ((uint8_t)0x03)
#define ADC_Channel_4                                  ((uint8_t)0x04)
#define ADC_Channel_5                                  ((uint8_t)0x05)
#define ADC_Channel_6                                  ((uint8_t)0x06)
#define ADC_Channel_7                                  ((uint8_t)0x07)
#define ADC_Channel_8                                  ((uint8_t)0x08)
#define ADC_Channel_9                                  ((uint8_t)0x09)
#define ADC_Channel_10                                 ((uint8_t)0x0A)
#define ADC_Channel_11                                 ((uint8_t)0x0B)
#define ADC_Channel_12                                 ((uint8_t)0x0C)
#define ADC_Channel_13                                 ((uint8_t)0x0D)
#define ADC_Channel_14                                 ((uint8_t)0x0E)
#define ADC_Channel_15                                 ((uint8_t)0x0F)

#define ADC_Channel_Vrefint                            ((uint8_t)ADC_Channel_15)

/* ADC_sampling_time */
#define ADC_SampleTime_4Cycles                         ((uint8_t)0x00)
#define ADC_SampleTime_5Cycles                         ((uint8_t)0x01)
#define ADC_SampleTime_6Cycles                         ((uint8_t)0x02)
#define ADC_SampleTime_7Cycles                         ((uint8_t)0x03)
#define ADC_SampleTime_8Cycles                         ((uint8_t)0x04)
#define ADC_SampleTime_9Cycles                         ((uint8_t)0x05)
#define ADC_SampleTime_10Cycles                        ((uint8_t)0x06)
#define ADC_SampleTime_11Cycles                        ((uint8_t)0x07)

/* ADC_external_trigger_sources_for_injected_channels_conversion */
#define ADC_ExternalTrigInjecConv_T1_CC3               ((uint32_t)0x00000000)
#define ADC_ExternalTrigInjecConv_T1_CC4               ((uint32_t)0x00001000)
#define ADC_ExternalTrigInjecConv_T2_CC3               ((uint32_t)0x00002000)
#define ADC_ExternalTrigInjecConv_T2_CC4               ((uint32_t)0x00003000)
#define ADC_ExternalTrigInjecConv_T2_CC2               ((uint32_t)0x00004000)
#define ADC_ExternalTrigInjecConv_T3_CC2               ((uint32_t)0x00005000)
#define ADC_ExternalTrigInjecConv_Ext_IT15             ((uint32_t)0x00006000)
#define ADC_ExternalTrigInjecConv_None                 ((uint32_t)0x00007000)

/* ADC_injected_channel_selection */
#define ADC_InjectedChannel_1                          ((uint8_t)0x14)
#define ADC_InjectedChannel_2                          ((uint8_t)0x18)
#define ADC_InjectedChannel_3                          ((uint8_t)0x1C)
#define ADC_InjectedChannel_4                          ((uint8_t)0x20)

/* ADC_analog_watchdog_selection */
#define ADC_AnalogWatchdog_SingleRegEnable             ((uint32_t)0x00800200)
#define ADC_AnalogWatchdog_SingleInjecEnable           ((uint32_t)0x00400200)
#define ADC_AnalogWatchdog_SingleRegOrInjecEnable      ((uint32_t)0x00C00200)
#define ADC_AnalogWatchdog_AllRegEnable                ((uint32_t)0x00800000)
#define ADC_AnalogWatchdog_AllInjecEnable              ((uint32_t)0x00400000)
#define ADC_AnalogWatchdog_AllRegAllInjecEnable        ((uint32_t)0x00C00000)
#define ADC_AnalogWatchdog_None                        ((uint32_t)0x00000000)

/* ADC_interrupts_definition */
#define ADC_IT_EOC                                     ((uint16_t)0x0220)
#define ADC_IT_AWD                                     ((uint16_t)0x0140)
#define ADC_IT_JEOC                                    ((uint16_t)0x0480)

/* ADC_flags_definition */
#define ADC_FLAG_AWD                                   ((uint8_t)0x01)
#define ADC_FLAG_EOC                                   ((uint8_t)0x02)
#define ADC_FLAG_JEOC                                  ((uint8_t)0x04)
#define ADC_FLAG_JSTRT                                 ((uint8_t)0x08)
#define ADC_FLAG_STRT                                  ((uint8_t)0x10)

/* ADC_analog_watchdog_reset_enable_selection */
#define ADC_AnalogWatchdog_0_RST_EN                    ((uint32_t)0x00001000)
#define ADC_AnalogWatchdog_1_RST_EN                    ((uint32_t)0x00002000)
#define ADC_AnalogWatchdog_2_RST_EN                    ((uint32_t)0x00004000)
#define ADC_AnalogWatchdog_3_RST_EN                    ((uint32_t)0x00008000)

/* ADC_analog_watchdog_reset_flags_definition */
#define ADC_AnalogWatchdogResetFLAG_0                  ((uint32_t)0x00010000)
#define ADC_AnalogWatchdogResetFLAG_1                  ((uint32_t)0x00020000)
#define ADC_AnalogWatchdogResetFLAG_2                  ((uint32_t)0x00040000)
#define ADC_AnalogWatchdogResetFLAG_3                  ((uint32_t)0x00080000)

/* ADC_clock */
#define ADC_CLK_Div4                                   ((uint32_t)0x00000013)
#define ADC_CLK_Div5                                   ((uint32_t)0x00000014)
#define ADC_CLK_Div6                                   ((uint32_t)0x00000025)
#define ADC_CLK_Div7                                   ((uint32_t)0x00000026)
#define ADC_CLK_Div8                                   ((uint32_t)0x00000037)
#define ADC_CLK_Div9                                   ((uint32_t)0x00000038)
#define ADC_CLK_Div10                                  ((uint32_t)0x00000049)
#define ADC_CLK_Div11                                  ((uint32_t)0x0000004A)
#define ADC_CLK_Div12                                  ((uint32_t)0x0000005B)
#define ADC_CLK_Div13                                  ((uint32_t)0x0000005C)
#define ADC_CLK_Div14                                  ((uint32_t)0x0000006D)
#define ADC_CLK_Div15                                  ((uint32_t)0x0000006E)
#define ADC_CLK_Div16                                  ((uint32_t)0x0000007F)


void       ADC_DeInit(ADC_TypeDef *ADCx);
void       ADC_Init(ADC_TypeDef *ADCx, ADC_InitTypeDef *ADC_InitStruct);
void       ADC_StructInit(ADC_InitTypeDef *ADC_InitStruct);
void       ADC_Cmd(ADC_TypeDef *ADCx, FunctionalState NewState);
void       ADC_DMACmd(ADC_TypeDef *ADCx, FunctionalState NewState);
void       ADC_ITConfig(ADC_TypeDef *ADCx, uint16_t ADC_IT, FunctionalState NewState);
void       ADC_SoftwareStartConvCmd(ADC_TypeDef *ADCx, FunctionalState NewState);
FlagStatus ADC_GetSoftwareStartConvStatus(ADC_TypeDef *ADCx);
void       ADC_DiscModeChannelCountConfig(ADC_TypeDef *ADCx, uint8_t Number);
void       ADC_DiscModeCmd(ADC_TypeDef *ADCx, FunctionalState NewState);
void       ADC_RegularChannelConfig(ADC_TypeDef *ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime);
void       ADC_ExternalTrigConvCmd(ADC_TypeDef *ADCx, FunctionalState NewState);
uint16_t   ADC_GetConversionValue(ADC_TypeDef *ADCx);
uint32_t   ADC_GetDualModeConversionValue(void);
void       ADC_AutoInjectedConvCmd(ADC_TypeDef *ADCx, FunctionalState NewState);
void       ADC_InjectedDiscModeCmd(ADC_TypeDef *ADCx, FunctionalState NewState);
void       ADC_ExternalTrigInjectedConvConfig(ADC_TypeDef *ADCx, uint32_t ADC_ExternalTrigInjecConv);
void       ADC_ExternalTrigInjectedConvCmd(ADC_TypeDef *ADCx, FunctionalState NewState);
void       ADC_SoftwareStartInjectedConvCmd(ADC_TypeDef *ADCx, FunctionalState NewState);
FlagStatus ADC_GetSoftwareStartInjectedConvCmdStatus(ADC_TypeDef *ADCx);
void       ADC_InjectedChannelConfig(ADC_TypeDef *ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime);
void       ADC_InjectedSequencerLengthConfig(ADC_TypeDef *ADCx, uint8_t Length);
void       ADC_SetInjectedOffset(ADC_TypeDef *ADCx, uint8_t ADC_InjectedChannel, uint16_t Offset);
uint16_t   ADC_GetInjectedConversionValue(ADC_TypeDef *ADCx, uint8_t ADC_InjectedChannel);
void       ADC_AnalogWatchdogCmd(ADC_TypeDef *ADCx, uint32_t ADC_AnalogWatchdog);
void       ADC_AnalogWatchdogThresholdsConfig(ADC_TypeDef *ADCx, uint16_t HighThreshold, uint16_t LowThreshold);
void       ADC_AnalogWatchdog1ThresholdsConfig(ADC_TypeDef *ADCx, uint16_t HighThreshold, uint16_t LowThreshold);
void       ADC_AnalogWatchdog2ThresholdsConfig(ADC_TypeDef *ADCx, uint16_t HighThreshold, uint16_t LowThreshold);
void       ADC_AnalogWatchdog3ThresholdsConfig(ADC_TypeDef *ADCx, uint16_t HighThreshold, uint16_t LowThreshold);
void       ADC_AnalogWatchdogSingleChannelConfig(ADC_TypeDef *ADCx, uint8_t ADC_Channel);
FlagStatus ADC_GetFlagStatus(ADC_TypeDef *ADCx, uint8_t ADC_FLAG);
void       ADC_ClearFlag(ADC_TypeDef *ADCx, uint8_t ADC_FLAG);
ITStatus   ADC_GetITStatus(ADC_TypeDef *ADCx, uint16_t ADC_IT);
void       ADC_ClearITPendingBit(ADC_TypeDef *ADCx, uint16_t ADC_IT);
void       ADC_AnalogWatchdogResetCmd(ADC_TypeDef *ADCx, uint32_t ADC_AnalogWatchdog_x, FunctionalState NewState);
void       ADC_AnalogWatchdogScanCmd(ADC_TypeDef *ADCx, FunctionalState NewState);
void       ADC_CLKConfig(ADC_TypeDef *ADCx, uint32_t ADC_CLK_Div_x);


#ifdef __cplusplus
}
#endif

#endif
