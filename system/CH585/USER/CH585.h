// added by deqing for compatibility 

#ifndef __CH585_H__
#define __CH585_H__


#include "CH585_common.h"

/* General Purpose I/O */
typedef struct
{
    __IO uint32_t DIR;
    __IO uint32_t PIN;
    __IO uint32_t OUT;
    __IO uint32_t CLR;
    __IO uint32_t PU;
    __IO uint32_t PD_DRV;
    __IO uint32_t SET;
} GPIO_TypeDef;

/* Configuration Mode enumeration */
typedef enum
{
    GPIO_Mode_AIN = 0x0,
    GPIO_Mode_IN_FLOATING = 0x04,
    GPIO_Mode_IPD = 0x28,
    GPIO_Mode_IPU = 0x48,
    GPIO_Mode_Out_OD = 0x14,
    GPIO_Mode_Out_PP = 0x10,
    GPIO_Mode_AF_OD = 0x1C,
    GPIO_Mode_AF_PP = 0x18,
} GPIOMode_TypeDef;

/* Bit_SET and Bit_RESET enumeration */
typedef enum
{
    Bit_RESET = 0,
    Bit_SET
} BitAction;

/* EXTI mode enumeration */
typedef enum
{
    // EXTI_Mode_Interrupt = 0x00,
    // EXTI_Mode_Event = 0x04
    DUMMY_EXTIMode_TypeDef
} EXTIMode_TypeDef;


typedef enum
{
    // EXTI_Trigger_Rising = 0x08,
    // EXTI_Trigger_Falling = 0x0C,
    // EXTI_Trigger_Rising_Falling = 0x10
    DUMMY_EXTITrigger_TypeDef
} EXTITrigger_TypeDef;


/*********************************************************************
 * @fn      NVIC_EnableIRQ
 *
 * @brief   Disable Interrupt
 *
 * @param   IRQn - Interrupt Numbers
 *
 * @return  none
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  PFIC_EnableIRQ(IRQn);
}

/*********************************************************************
 * @fn      NVIC_DisableIRQ
 *
 * @brief   Disable Interrupt
 *
 * @param   IRQn - Interrupt Numbers
 *
 * @return  none
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  PFIC_DisableIRQ(IRQn);
}

/*********************************************************************
 * @fn      NVIC_SetPriority
 *
 * @brief   Set Interrupt Priority
 *
 * @param   IRQn - Interrupt Numbers
 *          priority -bit[7] - Pre-emption Priority
 *                    bit[6:4] - Subpriority
 * @return  None
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE void NVIC_SetPriority(IRQn_Type IRQn, uint8_t priority)
{
  PFIC_SetPriority(IRQn, priority);
}

//todo: need more check
/* EXTI_Lines */
#define EXTI_Line0     ((uint32_t)0x00001) /* External interrupt line 0 */
#define EXTI_Line1     ((uint32_t)0x00002) /* External interrupt line 1 */
#define EXTI_Line2     ((uint32_t)0x00004) /* External interrupt line 2 */
#define EXTI_Line3     ((uint32_t)0x00008) /* External interrupt line 3 */
#define EXTI_Line4     ((uint32_t)0x00010) /* External interrupt line 4 */
#define EXTI_Line5     ((uint32_t)0x00020) /* External interrupt line 5 */
#define EXTI_Line6     ((uint32_t)0x00040) /* External interrupt line 6 */
#define EXTI_Line7     ((uint32_t)0x00080) /* External interrupt line 7 */
#define EXTI_Line8     ((uint32_t)0x00100) /* External interrupt line 8 Connected to the PVD Output */
#define EXTI_Line9     ((uint32_t)0x00200) /* External interrupt line 9 Connected to the PWR Auto Wake-up event*/
#define EXTI_Line10    ((uint32_t)0x00400) /* External interrupt line 10 */
#define EXTI_Line11    ((uint32_t)0x00800) /* External interrupt line 11 */
#define EXTI_Line12    ((uint32_t)0x01000) /* External interrupt line 12 */
#define EXTI_Line13    ((uint32_t)0x02000) /* External interrupt line 13 */
#define EXTI_Line14    ((uint32_t)0x04000) /* External interrupt line 14 */
#define EXTI_Line15    ((uint32_t)0x08000) /* External interrupt line 15 */

/* EXTI Init Structure definition */
typedef struct
{
    uint32_t EXTI_Line; /* Specifies the EXTI lines to be enabled or disabled.
                           This parameter can be any combination of @ref EXTI_Lines */

    EXTIMode_TypeDef EXTI_Mode; /* Specifies the mode for the EXTI lines.
                                   This parameter can be a value of @ref EXTIMode_TypeDef */

    EXTITrigger_TypeDef EXTI_Trigger; /* Specifies the trigger signal active edge for the EXTI lines.
                                         This parameter can be a value of @ref EXTIMode_TypeDef */

    FunctionalState EXTI_LineCmd; /* Specifies the new state of the selected EXTI lines.
                                     This parameter can be set either to ENABLE or DISABLE */
} EXTI_InitTypeDef;


#define GPIOA_BASE                              (BA_PA)
#define GPIOB_BASE                              (BA_PB)

#define GPIOA                                   ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB                                   ((GPIO_TypeDef *)GPIOB_BASE)

/* Output Maximum frequency selection */
typedef enum
{
    GPIO_Speed_50MHz = 1
} GPIOSpeed_TypeDef;

typedef struct
{
    uint32_t GPIO_Pin; /* Specifies the GPIO pins to be configured.
                          This parameter can be any value of @ref GPIO_pins_define */

    GPIOSpeed_TypeDef GPIO_Speed; /* Specifies the speed for the selected pins.
                                     This parameter can be a value of @ref GPIOSpeed_TypeDef */

    GPIOMode_TypeDef GPIO_Mode; /* Specifies the operating mode for the selected pins.
                                   This parameter can be a value of @ref GPIOMode_TypeDef */
} GPIO_InitTypeDef;

void GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_InitStruct);

/* GPIO_Port_Sources */
#define GPIO_PortSourceGPIOA            ((uint8_t)0x00)
#define GPIO_PortSourceGPIOB            ((uint8_t)0x01)

void EXTI_ClearITPendingBit(uint32_t EXTI_Line);

ITStatus EXTI_GetITStatus(uint32_t EXTI_Line);
void EXTI_Init(EXTI_InitTypeDef *EXTI_InitStruct);
void GPIO_EXTILineConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource);


uint8_t  GPIO_ReadInputDataBit(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin);
// uint16_t GPIO_ReadInputData(GPIO_TypeDef *GPIOx);
// uint8_t  GPIO_ReadOutputDataBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
// uint16_t GPIO_ReadOutputData(GPIO_TypeDef *GPIOx);
void     GPIO_SetBits(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin);
void     GPIO_ResetBits(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin);
// void     GPIO_WriteBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, BitAction BitVal);
// void     GPIO_Write(GPIO_TypeDef *GPIOx, uint16_t PortVal);


/* Arduino-compatible ADC register view (BLE AUX ADC @ 0x40001058) */
typedef struct
{
    __IO uint8_t ADC_CHANNEL;
    __IO uint8_t ADC_CFG;
    __IO uint8_t ADC_CONVERT;
    __IO uint8_t TEM_SENSOR;
    __IO uint16_t ADC_DATA;
    __IO uint8_t ADC_INT_FLAG;
    __IO uint32_t ADC_DMA_CTRL;
    __IO uint8_t ADC_CTRL_DMA;
    __IO uint8_t ADC_DMA_IF;
    __IO uint8_t ADC_AUTO_CYCLE;
    __IO uint16_t ADC_DMA_NOW;
    __IO uint16_t ADC_DMA_BEG;
    __IO uint16_t ADC_DMA_END;
} ADC_TypeDef;

#define ADC1 ((ADC_TypeDef *)0x40001058)

#define ADC_Channel_0                               ((uint8_t)0x00)
#define ADC_Channel_1                               ((uint8_t)0x01)
#define ADC_Channel_2                               ((uint8_t)0x02)
#define ADC_Channel_3                               ((uint8_t)0x03)
#define ADC_Channel_4                               ((uint8_t)0x04)
#define ADC_Channel_5                               ((uint8_t)0x05)
#define ADC_Channel_6                               ((uint8_t)0x06)
#define ADC_Channel_7                               ((uint8_t)0x07)
#define ADC_Channel_8                               ((uint8_t)0x08)
#define ADC_Channel_9                               ((uint8_t)0x09)
#define ADC_Channel_10                              ((uint8_t)0x0A)
#define ADC_Channel_11                              ((uint8_t)0x0B)
#define ADC_Channel_12                              ((uint8_t)0x0C)
#define ADC_Channel_13                              ((uint8_t)0x0D)
#define ADC_Channel_14                              ((uint8_t)0x0E)
#define ADC_Channel_15                              ((uint8_t)0x0F)

#define ADC_Channel_TempSensor                      ((uint8_t)ADC_Channel_15)
#define ADC_Channel_Vbat                            ((uint8_t)ADC_Channel_14)
#define ADC_Channel_VBAT                            ((uint8_t)ADC_Channel_14)
#define ADC_CHANNEL_TEMPSENSOR                      ADC_Channel_TempSensor
#define ADC_CHANNEL_VBAT                            ADC_Channel_Vbat

/* ADC Init structure definition */
typedef struct
{
    uint32_t ADC_Mode;
    FunctionalState ADC_ScanConvMode;
    FunctionalState ADC_ContinuousConvMode;
    uint32_t ADC_ExternalTrigConv;
    uint32_t ADC_DataAlign;
    uint8_t ADC_NbrOfChannel;
    uint32_t ADC_OutputBuffer;
    uint32_t ADC_Pga;
} ADC_InitTypeDef;

/* CH585 BLE ADC has no classic sample-time field used by Arduino path */
#define ADC_SAMPLINGTIME 0
#define ADC_SAMPLINGTIME_INTERNAL 0

#define ADC_Mode_Independent                           ((uint32_t)0x00000000)
#define ADC_ExternalTrigConv_None                      ((uint32_t)0x000E0000)

#define ADC_FLAG_EOC                                   RB_ADC_IF_EOC

#ifdef __cplusplus
extern "C" {
#endif

void ADC_Cmd(ADC_TypeDef *ADCx, FunctionalState NewState);
void ADC_DeInit(ADC_TypeDef *ADCx);
void ADC_Init(ADC_TypeDef *ADCx, ADC_InitTypeDef *ADC_InitStruct);
void ADC_RegularChannelConfig(ADC_TypeDef *ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime);
void ADC_SoftwareStartConvCmd(ADC_TypeDef *ADCx, FunctionalState NewState);
FlagStatus ADC_GetFlagStatus(ADC_TypeDef *ADCx, uint8_t ADC_FLAG);
uint16_t ADC_ConvertPAGValueCH585(void);

#ifdef __cplusplus
}
#endif


#endif  // __CH585_H__
