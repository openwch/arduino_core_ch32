// added by deqing for compatibility 

#ifndef __CH573_H__
#define __CH573_H__


#include "CH573_common.h"

/* General Purpose I/O */
typedef struct
{
    __IO uint32_t DIR;
    __IO uint32_t PIN;
    __IO uint32_t OUT;
    __IO uint32_t CLR;
    __IO uint32_t PU;
    __IO uint32_t PD_DRV;
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

typedef struct
{
    __IO uint16_t CTLR1;
    uint16_t      RESERVED0;
    __IO uint16_t CTLR2;
    uint16_t      RESERVED1;
    __IO uint16_t SMCFGR;
    uint16_t      RESERVED2;
    __IO uint16_t DMAINTENR;
    uint16_t      RESERVED3;
    __IO uint16_t INTFR;
    uint16_t      RESERVED4;
    __IO uint16_t SWEVGR;
    uint16_t      RESERVED5;
    __IO uint16_t CHCTLR1;
    uint16_t      RESERVED6;
    __IO uint16_t CHCTLR2;
    uint16_t      RESERVED7;
    __IO uint16_t CCER;
    uint16_t      RESERVED8;
    __IO uint16_t CNT;
    uint16_t      RESERVED9;
    __IO uint16_t PSC;
    uint16_t      RESERVED10;
    __IO uint16_t ATRLR;
    uint16_t      RESERVED11;
    __IO uint16_t RPTCR;
    uint16_t      RESERVED12;
    __IO uint32_t CH1CVR;
    __IO uint32_t CH2CVR;
    __IO uint32_t CH3CVR;
    __IO uint32_t CH4CVR;
    __IO uint16_t BDTR;
    uint16_t      RESERVED13;
    __IO uint16_t DMACFGR;
    uint16_t      RESERVED14;
    __IO uint16_t DMAADR;
    uint16_t      RESERVED15;
} TIM_TypeDef;


typedef struct
{
    uint16_t TIM_Prescaler; /* Specifies the prescaler value used to divide the TIM clock.
                               This parameter can be a number between 0x0000 and 0xFFFF */

    uint16_t TIM_CounterMode; /* Specifies the counter mode.
                                 This parameter can be a value of @ref TIM_Counter_Mode */

    uint16_t TIM_Period; /* Specifies the period value to be loaded into the active
                            Auto-Reload Register at the next update event.
                            This parameter must be a number between 0x0000 and 0xFFFF.  */

    uint16_t TIM_ClockDivision; /* Specifies the clock division.
                                  This parameter can be a value of @ref TIM_Clock_Division_CKD */

    uint8_t TIM_RepetitionCounter; /* Specifies the repetition counter value. Each time the RCR downcounter
                                      reaches zero, an update event is generated and counting restarts
                                      from the RCR value (N).
                                      This means in PWM mode that (N+1) corresponds to:
                                         - the number of PWM periods in edge-aligned mode
                                         - the number of half PWM period in center-aligned mode
                                      This parameter must be a number between 0x00 and 0xFF.
                                      @note This parameter is valid only for TIM1 and TIM8. */
} TIM_TimeBaseInitTypeDef;



/* TIM Output Compare Init structure definition */
typedef struct
{
    // uint16_t TIM_OCMode; /* Specifies the TIM mode.
    //                         This parameter can be a value of @ref TIM_Output_Compare_and_PWM_modes */

    // uint16_t TIM_OutputState; /* Specifies the TIM Output Compare state.
    //                              This parameter can be a value of @ref TIM_Output_Compare_state */

    // uint16_t TIM_OutputNState; /* Specifies the TIM complementary Output Compare state.
    //                               This parameter can be a value of @ref TIM_Output_Compare_N_state
    //                               @note This parameter is valid only for TIM1 and TIM8. */

    // uint16_t TIM_Pulse; /* Specifies the pulse value to be loaded into the Capture Compare Register.
    //                        This parameter can be a number between 0x0000 and 0xFFFF */

    // uint16_t TIM_OCPolarity; /* Specifies the output polarity.
    //                             This parameter can be a value of @ref TIM_Output_Compare_Polarity */

    // uint16_t TIM_OCNPolarity; /* Specifies the complementary output polarity.
    //                              This parameter can be a value of @ref TIM_Output_Compare_N_Polarity
    //                              @note This parameter is valid only for TIM1 and TIM8. */

    // uint16_t TIM_OCIdleState; /* Specifies the TIM Output Compare pin state during Idle state.
    //                              This parameter can be a value of @ref TIM_Output_Compare_Idle_State
    //                              @note This parameter is valid only for TIM1 and TIM8. */

    // uint16_t TIM_OCNIdleState; /* Specifies the TIM Output Compare pin state during Idle state.
    //                               This parameter can be a value of @ref TIM_Output_Compare_N_Idle_State
    //                               @note This parameter is valid only for TIM1 and TIM8. */
} TIM_OCInitTypeDef;


/* TIM Input Capture Init structure definition */
typedef struct
{
    // uint16_t TIM_Channel; /* Specifies the TIM channel.
    //                          This parameter can be a value of @ref TIM_Channel */

    // uint16_t TIM_ICPolarity; /* Specifies the active edge of the input signal.
    //                             This parameter can be a value of @ref TIM_Input_Capture_Polarity */

    // uint16_t TIM_ICSelection; /* Specifies the input.
    //                              This parameter can be a value of @ref TIM_Input_Capture_Selection */

    // uint16_t TIM_ICPrescaler; /* Specifies the Input Capture Prescaler.
    //                              This parameter can be a value of @ref TIM_Input_Capture_Prescaler */

    // uint16_t TIM_ICFilter; /* Specifies the input capture filter.
    //                           This parameter can be a number between 0x0 and 0xF */
} TIM_ICInitTypeDef;

/* Universal Synchronous Asynchronous Receiver Transmitter */
typedef struct
{
    __IO uint8_t MCR;
    __IO uint8_t IER;
    __IO uint8_t FCR;
    __IO uint8_t LCR;
    __IO uint8_t IIR;
    __IO uint8_t LSR;
    __IO uint8_t MSR;
    __IO uint8_t RBR;
    __IO uint8_t THR;
    __IO uint8_t RFC;
    __IO uint8_t TFC;
    __IO uint16_t DL;
    __IO uint8_t DIV;
    __IO uint8_t ADR;   //only exist for UART0
} USART_TypeDef;

/* USART Init Structure definition */
typedef struct
{
    uint32_t USART_BaudRate; /* This member configures the USART communication baud rate.
                                The baud rate is computed using the following formula:
                                 - IntegerDivider = ((PCLKx) / (16 * (USART_InitStruct->USART_BaudRate)))
                                 - FractionalDivider = ((IntegerDivider - ((u32) IntegerDivider)) * 16) + 0.5 */

    uint16_t USART_WordLength; /* Specifies the number of data bits transmitted or received in a frame.
                                  This parameter can be a value of @ref USART_Word_Length */

    uint16_t USART_StopBits; /* Specifies the number of stop bits transmitted.
                                This parameter can be a value of @ref USART_Stop_Bits */

    uint16_t USART_Parity; /* Specifies the parity mode.
                              This parameter can be a value of @ref USART_Parity
                              @note When parity is enabled, the computed parity is inserted
                                    at the MSB position of the transmitted data (9th bit when
                                    the word length is set to 9 data bits; 8th bit when the
                                    word length is set to 8 data bits). */

    uint16_t USART_Mode; /* Specifies wether the Receive or Transmit mode is enabled or disabled.
                            This parameter can be a value of @ref USART_Mode */

    uint16_t USART_HardwareFlowControl; /* Specifies wether the hardware flow control mode is enabled
                                           or disabled.
                                           This parameter can be a value of @ref USART_Hardware_Flow_Control */
} USART_InitTypeDef;



uint8_t  GPIO_ReadInputDataBit(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin);
// uint16_t GPIO_ReadInputData(GPIO_TypeDef *GPIOx);
// uint8_t  GPIO_ReadOutputDataBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
// uint16_t GPIO_ReadOutputData(GPIO_TypeDef *GPIOx);
void     GPIO_SetBits(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin);
void     GPIO_ResetBits(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin);
// void     GPIO_WriteBit(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, BitAction BitVal);
// void     GPIO_Write(GPIO_TypeDef *GPIOx, uint16_t PortVal);



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

#define NonMaskableInt_IRQn NMI_IRQn

// /* Interrupt Number Definition, according to the selected device */
// typedef enum IRQn
// {
//     /******  RISC-V Processor Exceptions Numbers *******************************************************/
//     NonMaskableInt_IRQn = 2, /* 2 Non Maskable Interrupt                             */
//     EXC_IRQn = 3,            /* 4 Exception Interrupt                                */
//     SysTicK_IRQn = 12,       /* 12 System timer Interrupt                            */
//     Software_IRQn = 14,      /* 14 software Interrupt                                */

//     /******  RISC-V specific Interrupt Numbers *********************************************************/
//     WWDG_IRQn = 16,          /* Window WatchDog Interrupt                            */
//     PVD_IRQn = 17,           /* PVD through EXTI Line detection Interrupt            */
//     TAMPER_IRQn = 18,        /* Tamper Interrupt                                     */
//     RTC_IRQn = 19,           /* RTC global Interrupt                                 */
//     FLASH_IRQn = 20,         /* FLASH global Interrupt                               */
//     RCC_IRQn = 21,           /* RCC global Interrupt                                 */
//     EXTI0_IRQn = 22,         /* EXTI Line0 Interrupt                                 */
//     EXTI1_IRQn = 23,         /* EXTI Line1 Interrupt                                 */
//     EXTI2_IRQn = 24,         /* EXTI Line2 Interrupt                                 */
//     EXTI3_IRQn = 25,         /* EXTI Line3 Interrupt                                 */
//     EXTI4_IRQn = 26,         /* EXTI Line4 Interrupt                                 */
//     DMA1_Channel1_IRQn = 27, /* DMA1 Channel 1 global Interrupt                      */
//     DMA1_Channel2_IRQn = 28, /* DMA1 Channel 2 global Interrupt                      */
//     DMA1_Channel3_IRQn = 29, /* DMA1 Channel 3 global Interrupt                      */
//     DMA1_Channel4_IRQn = 30, /* DMA1 Channel 4 global Interrupt                      */
//     DMA1_Channel5_IRQn = 31, /* DMA1 Channel 5 global Interrupt                      */
//     DMA1_Channel6_IRQn = 32, /* DMA1 Channel 6 global Interrupt                      */
//     DMA1_Channel7_IRQn = 33, /* DMA1 Channel 7 global Interrupt                      */
//     ADC_IRQn = 34,           /* ADC1 global Interrupt                                */
//     EXTI9_5_IRQn = 39,       /* External Line[9:5] Interrupts                        */
//     TIM1_BRK_IRQn = 40,      /* TIM1 Break Interrupt                                 */
//     TIM1_UP_IRQn = 41,       /* TIM1 Update Interrupt                                */
//     TIM1_TRG_COM_IRQn = 42,  /* TIM1 Trigger and Commutation Interrupt               */
//     TIM1_CC_IRQn = 43,       /* TIM1 Capture Compare Interrupt                       */
//     TIM2_IRQn = 44,          /* TIM2 global Interrupt                                */
//     TIM3_IRQn = 45,          /* TIM3 global Interrupt                                */
//     TIM4_IRQn = 46,          /* TIM4 global Interrupt                                */
//     I2C1_EV_IRQn = 47,       /* I2C1 Event Interrupt                                 */
//     I2C1_ER_IRQn = 48,       /* I2C1 Error Interrupt                                 */
//     I2C2_EV_IRQn = 49,       /* I2C2 Event Interrupt                                 */
//     I2C2_ER_IRQn = 50,       /* I2C2 Error Interrupt                                 */
//     SPI1_IRQn = 51,          /* SPI1 global Interrupt                                */
//     SPI2_IRQn = 52,          /* SPI2 global Interrupt                                */
//     USART1_IRQn = 53,        /* USART1 global Interrupt                              */
//     USART2_IRQn = 54,        /* USART2 global Interrupt                              */
//     USART3_IRQn = 55,        /* USART3 global Interrupt                              */
//     EXTI15_10_IRQn = 56,     /* External Line[15:10] Interrupts                      */
//     RTCAlarm_IRQn = 57,      /* RTC Alarm through EXTI Line Interrupt                */
//     USBWakeUp_IRQn = 58,     /* USB WakeUp from suspend through EXTI Line Interrupt  */
//     USBHD_IRQn = 59,         /* USBHD Interrupt                                      */

// } IRQn_Type;

//UART IRQn
#define USART1_IRQn                         (26)
#define USART2_IRQn                         (27)
#define USART3_IRQn                         (33)
#define USART4_IRQn                         (34)

/* TIM_Channel */
#define TIM_Channel_1                      ((uint16_t)0x0000)
#define TIM_Channel_2                      ((uint16_t)0x0004)
#define TIM_Channel_3                      ((uint16_t)0x0008)
#define TIM_Channel_4                      ((uint16_t)0x000C)

//UART0 related registers
#define USART1_BASE                             (BA_UART0)
#define USART1                                  ((USART_TypeDef *)USART1_BASE)

//UART1
#define USART2_BASE                         (BA_UART1)
#define USART2                              ((USART_TypeDef *)USART2_BASE)

//UART2
#define USART3_BASE                         (BA_UART2)
#define USART3                              ((USART_TypeDef *)USART3_BASE)

//UART3
#define USART4_BASE                         (BA_UART3)
#define USART4                              ((USART_TypeDef *)USART4_BASE)



/* USART_Hardware_Flow_Control */
#define USART_HardwareFlowControl_None       ((uint16_t)0x0000)
#define USART_HardwareFlowControl_RTS        ((uint16_t)0x0100)
#define USART_HardwareFlowControl_CTS        ((uint16_t)0x0200)
#define USART_HardwareFlowControl_RTS_CTS    ((uint16_t)0x0300)

/* USART_Word_Length */
#define USART_WordLength_5b                  ((uint16_t)0x0000)
#define USART_WordLength_6b                  ((uint16_t)0x0001)
#define USART_WordLength_7b                  ((uint16_t)0x0002)
#define USART_WordLength_8b                  ((uint16_t)0x0003)

/* USART_Stop_Bits */
#define USART_StopBits_1                     ((uint16_t)0x0000)
#define USART_StopBits_0_5                   ((uint16_t)0x0000)
#define USART_StopBits_2                     ((uint16_t)RB_LCR_STOP_BIT)
#define USART_StopBits_1_5                   ((uint16_t)RB_LCR_STOP_BIT)

/* USART_Parity */
#define USART_Parity_No                      ((uint16_t)0x0000)
#define USART_Parity_Even                    ((uint16_t)(RB_LCR_PAR_EN|0x10))
#define USART_Parity_Odd                     ((uint16_t)(RB_LCR_PAR_EN|0x00))

/* USART_Mode */
#define USART_Mode_Rx                        ((uint16_t)0x0004)
#define USART_Mode_Tx                        ((uint16_t)0x0008)

/* USART_Flags */
#define USART_FLAG_RX_BUSY                   ((uint16_t)0x0400)
#define USART_FLAG_CTS                       ((uint16_t)0x0200)
#define USART_FLAG_LBD                       ((uint16_t)0x0100)
#define USART_FLAG_TXE                       ((uint16_t)0x0080)
#define USART_FLAG_TC                        ((uint16_t)0x0040)
#define USART_FLAG_RXNE                      ((uint16_t)0x0020)
#define USART_FLAG_IDLE                      ((uint16_t)0x0010)
#define USART_FLAG_ORE                       ((uint16_t)0x0008)
#define USART_FLAG_NE                        ((uint16_t)0x0004)
#define USART_FLAG_FE                        ((uint16_t)0x0002)
#define USART_FLAG_PE                        ((uint16_t)0x0001)


#define RCC_APB1Periph_USART2          ((uint32_t)0x00020000)

#define TIM2_BASE                               (BA_TMR1)
#define TIM3_BASE                               (BA_TMR2)
#define TIM4_BASE                               (BA_TMR3)

#define TIM2                                    ((TIM_TypeDef *)TIM2_BASE)
#define TIM3                                    ((TIM_TypeDef *)TIM3_BASE)
#define TIM4                                    ((TIM_TypeDef *)TIM4_BASE)

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

#define ADC_Channel_0                     (CH_EXTIN_0)
#define ADC_Channel_1                     (CH_EXTIN_1)
#define ADC_Channel_2                     (CH_EXTIN_2)
#define ADC_Channel_3                     (CH_EXTIN_3)
#define ADC_Channel_4                     (CH_EXTIN_4)
#define ADC_Channel_5                     (CH_EXTIN_5)   
#define ADC_Channel_6                     (0)   
#define ADC_Channel_7                     (0)   


void TIM_DeInit(TIM_TypeDef *TIMx);
void TIM_CCxCmd(TIM_TypeDef *TIMx, uint16_t TIM_Channel, uint16_t TIM_CCx);


//does not exist in CH573, modify later
#define TIM_MOE 0


//may not exist in CH573, modify later
/* TIM_Capture_Compare_state */
#define TIM_CCx_Enable                     ((uint16_t)0x0001)
#define TIM_CCx_Disable                    ((uint16_t)0x0000)


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

void GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_InitStruct);

/* GPIO_Port_Sources */
#define GPIO_PortSourceGPIOA            ((uint8_t)0x00)
#define GPIO_PortSourceGPIOB            ((uint8_t)0x01)

void EXTI_ClearITPendingBit(uint32_t EXTI_Line);

ITStatus EXTI_GetITStatus(uint32_t EXTI_Line);
void EXTI_Init(EXTI_InitTypeDef *EXTI_InitStruct);
void GPIO_EXTILineConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource);


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
#define ADC_Channel_VBAT                            ((uint8_t)ADC_Channel_14)

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
                               This parameter can be a value of @ref ADC_data_align */

    uint8_t ADC_NbrOfChannel; /* Specifies the number of ADC channels that will be converted
                                   using the sequencer for regular channel group.
                                   This parameter must range from 1 to 16. */

    uint32_t ADC_OutputBuffer; /* Specifies whether the ADC channel output buffer is enabled or disabled.
                                    This parameter can be a value of @ref ADC_OutputBuffer */

    uint32_t ADC_Pga; /* Specifies the PGA gain multiple.
                           This parameter can be a value of @ref ADC_Pga */
} ADC_InitTypeDef;

//CH573 does not have sampling time setting
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
uint16_t ADC_ConvertPAGValueCH573();

#ifdef __cplusplus
}   // extern "C"
#endif

#endif  // __CH573_H__
