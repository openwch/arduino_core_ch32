/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32x035.h
 * Author             : WCH
 * Version            : V1.0.1
 * Date               : 2024/10/28
 * Description        : CH32X035 Device Peripheral Access Layer Header File.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32X035_H
#define __CH32X035_H

#ifdef __cplusplus
extern "C" {
#endif

#define __MPU_PRESENT             0                   /* Other CH32 devices does not provide an MPU */
#define __Vendor_SysTickConfig    0                   /* Set to 1 if different SysTick Config is used */

#define HSI_VALUE              ((uint32_t)48000000) /* Value of the Internal oscillator in Hz */

/* Standard Peripheral Library version number */
#define __STDPERIPH_VERSION_MAIN   (0x01) /* [15:8] main version */
#define __STDPERIPH_VERSION_SUB    (0x08) /* [7:0] sub version */
#define __STDPERIPH_VERSION        ((__STDPERIPH_VERSION_MAIN << 8)\
                                    |(__STDPERIPH_VERSION_SUB << 0))


/* Interrupt Number Definition, according to the selected device */
typedef enum IRQn
{
    /******  RISC-V Processor Exceptions Numbers *******************************************************/
    NonMaskableInt_IRQn = 2,   /* 2 Non Maskable Interrupt                             */
    EXC_IRQn = 3,              /* 3 Exception Interrupt                                */
    Ecall_M_Mode_IRQn = 5,     /* 5 Ecall M Mode Interrupt                             */
    Ecall_U_Mode_IRQn = 8,     /* 8 Ecall U Mode Interrupt                             */
    Break_Point_IRQn = 9,      /* 9 Break Point Interrupt                              */
    SysTick_IRQn = 12,         /* 12 System timer Interrupt                            */
    Software_IRQn = 14,        /* 14 software Interrupt                                */

    /******  RISC-V specific Interrupt Numbers *********************************************************/
    WWDG_IRQn = 16,            /* Window WatchDog Interrupt                            */
    PVD_IRQn = 17,             /* PVD through EXTI Line detection Interrupt            */
    FLASH_IRQn = 18,           /* FLASH global Interrupt                               */
    EXTI7_0_IRQn = 20,         /* External Line[7:0] Interrupts                        */
    AWU_IRQn = 21,             /* AWU global Interrupt                                 */
    DMA1_Channel1_IRQn = 22,   /* DMA1 Channel 1 global Interrupt                      */
    DMA1_Channel2_IRQn = 23,   /* DMA1 Channel 2 global Interrupt                      */
    DMA1_Channel3_IRQn = 24,   /* DMA1 Channel 3 global Interrupt                      */
    DMA1_Channel4_IRQn = 25,   /* DMA1 Channel 4 global Interrupt                      */
    DMA1_Channel5_IRQn = 26,   /* DMA1 Channel 5 global Interrupt                      */
    DMA1_Channel6_IRQn = 27,   /* DMA1 Channel 6 global Interrupt                      */
    DMA1_Channel7_IRQn = 28,   /* DMA1 Channel 7 global Interrupt                      */
    ADC1_IRQn = 29,            /* ADC1 global Interrupt                                */
    I2C1_EV_IRQn = 30,         /* I2C1 Event Interrupt                                 */
    I2C1_ER_IRQn = 31,         /* I2C1 Error Interrupt                                 */
    USART1_IRQn = 32,          /* USART1 global Interrupt                              */
    SPI1_IRQn = 33,            /* SPI1 global Interrupt                                */
    TIM1_BRK_IRQn = 34,        /* TIM1 Break Interrupt                                 */
    TIM1_UP_IRQn = 35,         /* TIM1 Update Interrupt                                */
    TIM1_TRG_COM_IRQn = 36,    /* TIM1 Trigger and Commutation Interrupt               */
    TIM1_CC_IRQn = 37,         /* TIM1 Capture Compare Interrupt                       */
    TIM2_UP_IRQn = 38,         /* TIM2 Update Interrupt                                */
    USART2_IRQn = 39,          /* USART2 global Interrupt                              */
    EXTI15_8_IRQn = 40,        /* External Line[15:8] Interrupts                       */
    EXTI25_16_IRQn = 41,       /* External Line[25:16] Interrupts                      */
    USART3_IRQn = 42,          /* USART3 global Interrupt                              */
    USART4_IRQn = 43,          /* USART4 global Interrupt                              */
    DMA1_Channel8_IRQn = 44,   /* DMA1 Channel 8 global Interrupt                      */
    USBFS_IRQn = 45,           /* USBFS Host/Device global Interrupt                   */
    USBFSWakeUp_IRQn = 46,     /* USBFS Host/Device WakeUp Interrupt                   */
    PIOC_IRQn = 47,            /* PIOC global Interrupt                                */
    OPA_IRQn = 48,             /* OPA global Interrupt                                 */
    USBPD_IRQn = 49,           /* USBPD global Interrupt                               */
    USBPDWakeUp_IRQn = 50,     /* USBPD WakeUp Interrupt                               */
    TIM2_CC_IRQn = 51,         /* TIM2 Capture Compare Interrupt                       */
    TIM2_TRG_COM_IRQn = 52,    /* TIM2 Trigger and Commutation Interrupt               */
    TIM2_BRK_IRQn = 53,        /* TIM2 Break Interrupt                                 */
    TIM3_IRQn = 54,            /* TIM3 global Interrupt                                */
} IRQn_Type;

#define HardFault_IRQn    EXC_IRQn
#define SysTicK_IRQn      SysTick_IRQn

#include <stdint.h>
#include "core_riscv.h"
#include "system_ch32x035.h"

/* Standard Peripheral Library old definitions (maintained for legacy purpose) */
#define HSI_Value             HSI_VALUE

/* Analog to Digital Converter */
typedef struct
{
    __IO uint32_t STATR;
    __IO uint32_t CTLR1;
    __IO uint32_t CTLR2;
    __IO uint32_t SAMPTR1;
    __IO uint32_t SAMPTR2;
    __IO uint32_t IOFR1;
    __IO uint32_t IOFR2;
    __IO uint32_t IOFR3;
    __IO uint32_t IOFR4;
    __IO uint32_t WDHTR;
    __IO uint32_t WDLTR;
    __IO uint32_t RSQR1;
    __IO uint32_t RSQR2;
    __IO uint32_t RSQR3;
    __IO uint32_t ISQR;
    __IO uint32_t IDATAR1;
    __IO uint32_t IDATAR2;
    __IO uint32_t IDATAR3;
    __IO uint32_t IDATAR4;
    __IO uint32_t RDATAR;
    __IO uint32_t CTLR3;
    __IO uint32_t WDTR1;
    __IO uint32_t WDTR2;
    __IO uint32_t WDTR3;
} ADC_TypeDef;

/* DMA Channel Controller */
typedef struct
{
    __IO uint32_t CFGR;
    __IO uint32_t CNTR;
    __IO uint32_t PADDR;
    __IO uint32_t MADDR;
} DMA_Channel_TypeDef;

/* DMA Controller */
typedef struct
{
    __IO uint32_t INTFR;
    __IO uint32_t INTFCR;
} DMA_TypeDef;

/* External Interrupt/Event Controller */
typedef struct
{
    __IO uint32_t INTENR;
    __IO uint32_t EVENR;
    __IO uint32_t RTENR;
    __IO uint32_t FTENR;
    __IO uint32_t SWIEVR;
    __IO uint32_t INTFR;
} EXTI_TypeDef;

/* FLASH Registers */
typedef struct
{
    __IO uint32_t ACTLR;
    __IO uint32_t KEYR;
    __IO uint32_t OBKEYR;
    __IO uint32_t STATR;
    __IO uint32_t CTLR;
    __IO uint32_t ADDR;
    uint32_t      RESERVED;
    __IO uint32_t OBR;
    __IO uint32_t WPR;
    __IO uint32_t MODEKEYR;
    __IO uint32_t BOOT_MODEKEYR;
} FLASH_TypeDef;

/* Option Bytes Registers */
typedef struct
{
    __IO uint16_t RDPR;
    __IO uint16_t USER;
    __IO uint16_t Data0;
    __IO uint16_t Data1;
    __IO uint16_t WRPR0;
    __IO uint16_t WRPR1;
    __IO uint16_t WRPR2;
    __IO uint16_t WRPR3;
} OB_TypeDef;

/* General Purpose I/O */
typedef struct
{
    __IO uint32_t CFGLR;
    __IO uint32_t CFGHR;
    __IO uint32_t INDR;
    __IO uint32_t OUTDR;
    __IO uint32_t BSHR;
    __IO uint32_t BCR;
    __IO uint32_t LCKR;
    __IO uint32_t CFGXR;
    __IO uint32_t BSXR;
} GPIO_TypeDef;

/* Alternate Function I/O */
typedef struct
{
    uint32_t      RESERVED0;
    __IO uint32_t PCFR1;
    __IO uint32_t EXTICR[2];
    uint32_t      RESERVED1;
    uint32_t      RESERVED2;
    __IO uint32_t CTLR;
} AFIO_TypeDef;

/* Inter Integrated Circuit Interface */
typedef struct
{
    __IO uint16_t CTLR1;
    uint16_t      RESERVED0;
    __IO uint16_t CTLR2;
    uint16_t      RESERVED1;
    __IO uint16_t OADDR1;
    uint16_t      RESERVED2;
    __IO uint16_t OADDR2;
    uint16_t      RESERVED3;
    __IO uint16_t DATAR;
    uint16_t      RESERVED4;
    __IO uint16_t STAR1;
    uint16_t      RESERVED5;
    __IO uint16_t STAR2;
    uint16_t      RESERVED6;
    __IO uint16_t CKCFGR;
    uint16_t      RESERVED7;
} I2C_TypeDef;

/* Independent WatchDog */
typedef struct
{
    __IO uint32_t CTLR;
    __IO uint32_t PSCR;
    __IO uint32_t RLDR;
    __IO uint32_t STATR;
} IWDG_TypeDef;

/* Power Control */
typedef struct
{
    __IO uint32_t CTLR;
    __IO uint32_t CSR;
} PWR_TypeDef;

/* Reset and Clock Control */
typedef struct
{
    __IO uint32_t CTLR;
    __IO uint32_t CFGR0;
    __IO uint32_t RESERVED0;
    __IO uint32_t APB2PRSTR;
    __IO uint32_t APB1PRSTR;
    __IO uint32_t AHBPCENR;
    __IO uint32_t APB2PCENR;
    __IO uint32_t APB1PCENR;
    __IO uint32_t RESERVED1;
    __IO uint32_t RSTSCKR;
    __IO uint32_t AHBRSTR;
} RCC_TypeDef;

/* Serial Peripheral Interface */
typedef struct
{
    __IO uint16_t CTLR1;
    uint16_t      RESERVED0;
    __IO uint16_t CTLR2;
    uint16_t      RESERVED1;
    __IO uint16_t STATR;
    uint16_t      RESERVED2;
    __IO uint16_t DATAR;
    uint16_t      RESERVED3;
    __IO uint16_t CRCR;
    uint16_t      RESERVED4;
    __IO uint16_t RCRCR;
    uint16_t      RESERVED5;
    __IO uint16_t TCRCR;
    uint16_t      RESERVED6;
    uint32_t      RESERVED7;
    uint32_t      RESERVED8;
    __IO uint16_t HSCR;
    uint16_t      RESERVED9;
} SPI_TypeDef;

/* TIM */
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
    union
    {
        __IO uint32_t CH1CVR_R32;
        struct
        {
            __IO uint16_t CH1CVR;
            uint16_t      RESERVED13;
        };
    };
    union
    {
        __IO uint32_t CH2CVR__R32;
        struct
        {
            __IO uint16_t CH2CVR;
            uint16_t      RESERVED14;
        };
    };
    union
    {
        __IO uint32_t CH3CVR__R32;
        struct
        {
            __IO uint16_t CH3CVR;
            uint16_t      RESERVED15;
        };
    };
    union
    {
        __IO uint32_t CH4CVR__R32;
        struct
        {
            __IO uint16_t CH4CVR;
            uint16_t      RESERVED16;
        };
    };
    __IO uint16_t BDTR;
    uint16_t      RESERVED17;
    __IO uint16_t DMACFGR;
    uint16_t      RESERVED18;
    __IO uint16_t DMAADR;
    uint16_t      RESERVED19;
    __IO uint16_t SPEC;
    uint16_t      RESERVED20;
} TIM_TypeDef;

/* Universal Synchronous Asynchronous Receiver Transmitter */
typedef struct
{
    __IO uint16_t STATR;
    uint16_t      RESERVED0;
    __IO uint16_t DATAR;
    uint16_t      RESERVED1;
    __IO uint16_t BRR;
    uint16_t      RESERVED2;
    __IO uint16_t CTLR1;
    uint16_t      RESERVED3;
    __IO uint16_t CTLR2;
    uint16_t      RESERVED4;
    __IO uint16_t CTLR3;
    uint16_t      RESERVED5;
    __IO uint16_t GPR;
    uint16_t      RESERVED6;
} USART_TypeDef;

/* Window WatchDog */
typedef struct
{
    __IO uint32_t CTLR;
    __IO uint32_t CFGR;
    __IO uint32_t STATR;
} WWDG_TypeDef;

/* OPA Registers */
typedef struct
{
    __IO uint16_t CFGR1;
    __IO uint16_t CFGR2;
    __IO uint32_t CTLR1;
    __IO uint32_t CTLR2;
    __IO uint32_t OPAKEY;
    __IO uint32_t CMPKEY;
    __IO uint32_t POLLKEY;
} OPA_TypeDef;

/* AWU Registers */
typedef struct
{
    __IO uint32_t CSR;
    __IO uint32_t WR;
    __IO uint32_t PSC;
} AWU_TypeDef;

/* PD Registers */

typedef struct
{
    union
    {
        __IO uint32_t USBPD_CONFIG;
        struct
        {
            __IO uint16_t CONFIG;
            __IO uint16_t BMC_CLK_CNT;
        };
    };
    union
    {
        __IO uint32_t USBPD_CONTROL;
        struct
        {
            union
            {
                __IO uint16_t R16_CONTROL;
                struct
                {
                    __IO uint8_t  CONTROL;
                    __IO uint8_t  TX_SEL;
                };
            };
            __IO uint16_t BMC_TX_SZ;
        };
    };
    union
    {
        __IO uint32_t USBPD_STATUS;
        struct
        {
            union
            {
                __IO uint16_t R16_STATUS;
                struct
                {
                    __IO uint8_t  DATA_BUF;
                    __IO uint8_t  STATUS;
                };
            };
            __IO uint16_t BMC_BYTE_CNT;
        };
    };
    union
    {
        __IO uint32_t USBPD_PORT;
        struct
        {
            __IO uint16_t PORT_CC1;
            __IO uint16_t PORT_CC2;
        };
    };
    union
    {
        __IO uint32_t USBPD_DMA;
        struct
        {
            __IO uint16_t DMA;
            __IO uint16_t RESERVED;
        };
    };
} USBPD_TypeDef;

/* USBFS Registers */
typedef struct
{
    __IO uint8_t  BASE_CTRL;
    __IO uint8_t  UDEV_CTRL;
    __IO uint8_t  INT_EN;
    __IO uint8_t  DEV_ADDR;
    uint8_t       RESERVED0;
    __IO uint8_t  MIS_ST;
    __IO uint8_t  INT_FG;
    __IO uint8_t  INT_ST;
    __IO uint32_t RX_LEN;
    __IO uint8_t  UEP4_1_MOD;
    __IO uint8_t  UEP2_3_MOD;
    __IO uint8_t  UEP567_MOD;
    uint8_t       RESERVED1;
    __IO uint32_t UEP0_DMA;
    __IO uint32_t UEP1_DMA;
    __IO uint32_t UEP2_DMA;
    __IO uint32_t UEP3_DMA;
    union{
        __IO uint32_t  UEP0_CTRL;
        struct{
            __IO uint16_t  UEP0_TX_LEN;
            __IO uint16_t  UEP0_CTRL_H;
        };
    };
    union{
        __IO uint32_t  UEP1_CTRL;
        struct{
            __IO uint16_t  UEP1_TX_LEN;
            __IO uint16_t  UEP1_CTRL_H;
        };
    };
    union{
        __IO uint32_t  UEP2_CTRL;
        struct{
            __IO uint16_t  UEP2_TX_LEN;
            __IO uint16_t  UEP2_CTRL_H;
        };
    };
    union{
        __IO uint32_t  UEP3_CTRL;
        struct{
            __IO uint16_t  UEP3_TX_LEN;
            __IO uint16_t  UEP3_CTRL_H;
        };
    };
    union{
        __IO uint32_t  UEP4_CTRL;
        struct{
            __IO uint16_t  UEP4_TX_LEN;
            __IO uint16_t  UEP4_CTRL_H;
        };
    };
    uint32_t      RESERVED2;
    uint32_t      RESERVED3;
    uint32_t      RESERVED4;
    uint32_t      RESERVED5;
    uint32_t      RESERVED6;
    uint32_t      RESERVED7;
    uint32_t      RESERVED8;
    uint32_t      RESERVED9;
    __IO uint32_t UEP5_DMA;
    __IO uint32_t UEP6_DMA;
    __IO uint32_t UEP7_DMA;
    uint32_t      RESERVED10;
    union{
        __IO uint32_t  UEP5_CTRL;
        struct{
            __IO uint16_t  UEP5_TX_LEN;
            __IO uint16_t  UEP5_CTRL_H;
        };
    };
    union{
        __IO uint32_t  UEP6_CTRL;
        struct{
            __IO uint16_t  UEP6_TX_LEN;
            __IO uint16_t  UEP6_CTRL_H;
        };
    };
    union{
        __IO uint32_t  UEP7_CTRL;
        struct{
            __IO uint16_t  UEP7_TX_LEN;
            __IO uint16_t  UEP7_CTRL_H;
        };
    };
    __IO uint32_t UEPX_MOD;
} USBFSD_TypeDef;

typedef struct
{
    __IO uint8_t   BASE_CTRL;
    __IO uint8_t   HOST_CTRL;
    __IO uint8_t   INT_EN;
    __IO uint8_t   DEV_ADDR;
    uint8_t        RESERVED0;
    __IO uint8_t   MIS_ST;
    __IO uint8_t   INT_FG;
    __IO uint8_t   INT_ST;
    __IO uint16_t  RX_LEN;
    uint16_t       RESERVED1;
    uint8_t        RESERVED2;
    __IO uint8_t   HOST_EP_MOD;
    uint16_t       RESERVED3;
    uint32_t       RESERVED4;
    uint32_t       RESERVED5;
    __IO uint16_t  HOST_RX_DMA;
    uint16_t       RESERVED6;
    __IO uint16_t  HOST_TX_DMA;
    uint16_t       RESERVED7;
    uint32_t       RESERVED8;
    uint16_t       RESERVED9;
    __IO uint8_t   HOST_SETUP;
    uint8_t        RESERVED10;
    __IO uint8_t   HOST_EP_PID;
    uint8_t        RESERVED11;
    __IO uint8_t   HOST_RX_CTRL;
    uint8_t        RESERVED12;
    __IO uint8_t   HOST_TX_LEN;
    uint8_t        RESERVED13;
    __IO uint8_t   HOST_TX_CTRL;
    uint8_t        RESERVED14;
} USBFSH_TypeDef;

/* Peripheral memory map */
#define FLASH_BASE                              ((uint32_t)0x08000000) /* FLASH base address in the alias region */
#define SRAM_BASE                               ((uint32_t)0x20000000) /* SRAM base address in the alias region */
#define PERIPH_BASE                             ((uint32_t)0x40000000) /* Peripheral base address in the alias region */

#define APB1PERIPH_BASE                         (PERIPH_BASE)
#define APB2PERIPH_BASE                         (PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE                          (PERIPH_BASE + 0x20000)

#define TIM2_BASE                               (APB1PERIPH_BASE + 0x0000)
#define TIM3_BASE                               (APB1PERIPH_BASE + 0x0400)
#define WWDG_BASE                               (APB1PERIPH_BASE + 0x2C00)
#define IWDG_BASE                               (APB1PERIPH_BASE + 0x3000)
#define USART2_BASE                             (APB1PERIPH_BASE + 0x4400)
#define USART3_BASE                             (APB1PERIPH_BASE + 0x4800)
#define USART4_BASE                             (APB1PERIPH_BASE + 0x4C00)
#define I2C1_BASE                               (APB1PERIPH_BASE + 0x5400)
#define PWR_BASE                                (APB1PERIPH_BASE + 0x7000)

#define AFIO_BASE                               (APB2PERIPH_BASE + 0x0000)
#define EXTI_BASE                               (APB2PERIPH_BASE + 0x0400)
#define GPIOA_BASE                              (APB2PERIPH_BASE + 0x0800)
#define GPIOB_BASE                              (APB2PERIPH_BASE + 0x0C00)
#define GPIOC_BASE                              (APB2PERIPH_BASE + 0x1000)
#define ADC1_BASE                               (APB2PERIPH_BASE + 0x2400)
#define TIM1_BASE                               (APB2PERIPH_BASE + 0x2C00)
#define SPI1_BASE                               (APB2PERIPH_BASE + 0x3000)
#define USART1_BASE                             (APB2PERIPH_BASE + 0x3800)

#define DMA1_BASE                               (AHBPERIPH_BASE + 0x0000)
#define DMA1_Channel1_BASE                      (AHBPERIPH_BASE + 0x0008)
#define DMA1_Channel2_BASE                      (AHBPERIPH_BASE + 0x001C)
#define DMA1_Channel3_BASE                      (AHBPERIPH_BASE + 0x0030)
#define DMA1_Channel4_BASE                      (AHBPERIPH_BASE + 0x0044)
#define DMA1_Channel5_BASE                      (AHBPERIPH_BASE + 0x0058)
#define DMA1_Channel6_BASE                      (AHBPERIPH_BASE + 0x006C)
#define DMA1_Channel7_BASE                      (AHBPERIPH_BASE + 0x0080)
#define DMA1_Channel8_BASE                      (AHBPERIPH_BASE + 0x0094)
#define RCC_BASE                                (AHBPERIPH_BASE + 0x1000)
#define FLASH_R_BASE                            (AHBPERIPH_BASE + 0x2000)
#define USBFS_BASE                              (AHBPERIPH_BASE + 0x3400)
#define OPA_BASE                                (AHBPERIPH_BASE + 0x6000)
#define AWU_BASE                                (AHBPERIPH_BASE + 0x6400)
#define PIOC_BASE                               (AHBPERIPH_BASE + 0x6C00)
#define USBPD_BASE                              (AHBPERIPH_BASE + 0x7000)

#define OB_BASE                                 ((uint32_t)0x1FFFF800)

/* Peripheral declaration */
#define TIM2                                    ((TIM_TypeDef *)TIM2_BASE)
#define TIM3                                    ((TIM_TypeDef *)TIM3_BASE)
#define WWDG                                    ((WWDG_TypeDef *)WWDG_BASE)
#define IWDG                                    ((IWDG_TypeDef *)IWDG_BASE)
#define USART2                                  ((USART_TypeDef *)USART2_BASE)
#define USART3                                  ((USART_TypeDef *)USART3_BASE)
#define USART4                                  ((USART_TypeDef *)USART4_BASE)
#define I2C1                                    ((I2C_TypeDef *)I2C1_BASE)
#define PWR                                     ((PWR_TypeDef *)PWR_BASE)

#define AFIO                                    ((AFIO_TypeDef *)AFIO_BASE)
#define EXTI                                    ((EXTI_TypeDef *)EXTI_BASE)
#define GPIOA                                   ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB                                   ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC                                   ((GPIO_TypeDef *)GPIOC_BASE)
#define ADC1                                    ((ADC_TypeDef *)ADC1_BASE)
#define TKey1                                   ((ADC_TypeDef *)ADC1_BASE)
#define TIM1                                    ((TIM_TypeDef *)TIM1_BASE)
#define SPI1                                    ((SPI_TypeDef *)SPI1_BASE)
#define USART1                                  ((USART_TypeDef *)USART1_BASE)

#define DMA1                                    ((DMA_TypeDef *)DMA1_BASE)
#define DMA1_Channel1                           ((DMA_Channel_TypeDef *)DMA1_Channel1_BASE)
#define DMA1_Channel2                           ((DMA_Channel_TypeDef *)DMA1_Channel2_BASE)
#define DMA1_Channel3                           ((DMA_Channel_TypeDef *)DMA1_Channel3_BASE)
#define DMA1_Channel4                           ((DMA_Channel_TypeDef *)DMA1_Channel4_BASE)
#define DMA1_Channel5                           ((DMA_Channel_TypeDef *)DMA1_Channel5_BASE)
#define DMA1_Channel6                           ((DMA_Channel_TypeDef *)DMA1_Channel6_BASE)
#define DMA1_Channel7                           ((DMA_Channel_TypeDef *)DMA1_Channel7_BASE)
#define DMA1_Channel8                           ((DMA_Channel_TypeDef *)DMA1_Channel8_BASE)
#define RCC                                     ((RCC_TypeDef *)RCC_BASE)
#define FLASH                                   ((FLASH_TypeDef *)FLASH_R_BASE)
#define USBFSD                                  ((USBFSD_TypeDef *)USBFS_BASE)
#define USBFSH                                  ((USBFSH_TypeDef *)USBFS_BASE)
#define OPA                                     ((OPA_TypeDef *)OPA_BASE)
#define AWU                                     ((AWU_TypeDef *)AWU_BASE)
#define USBPD                                   ((USBPD_TypeDef *)USBPD_BASE)

#define OB                                      ((OB_TypeDef *)OB_BASE)


/******************************************************************************/
/*                         Peripheral Registers Bits Definition               */
/******************************************************************************/

/******************************************************************************/
/*                        Analog to Digital Converter                         */
/******************************************************************************/

/********************  Bit definition for ADC_STATR register  ********************/
#define ADC_AWD                                 ((uint8_t)0x01) /* Analog watchdog flag */
#define ADC_EOC                                 ((uint8_t)0x02) /* End of conversion */
#define ADC_JEOC                                ((uint8_t)0x04) /* Injected channel end of conversion */
#define ADC_JSTRT                               ((uint8_t)0x08) /* Injected channel Start flag */
#define ADC_STRT                                ((uint8_t)0x10) /* Regular channel Start flag */

/*******************  Bit definition for ADC_CTLR1 register  ********************/
#define ADC_AWDCH                               ((uint32_t)0x0000000F) /* AWDCH[3:0] bits (Analog watchdog channel select bits) */
#define ADC_AWDCH_0                             ((uint32_t)0x00000001) /* Bit 0 */
#define ADC_AWDCH_1                             ((uint32_t)0x00000002) /* Bit 1 */
#define ADC_AWDCH_2                             ((uint32_t)0x00000004) /* Bit 2 */
#define ADC_AWDCH_3                             ((uint32_t)0x00000008) /* Bit 3 */

#define ADC_EOCIE                               ((uint32_t)0x00000020) /* Interrupt enable for EOC */
#define ADC_AWDIE                               ((uint32_t)0x00000040) /* Analog Watchdog interrupt enable */
#define ADC_JEOCIE                              ((uint32_t)0x00000080) /* Interrupt enable for injected channels */
#define ADC_SCAN                                ((uint32_t)0x00000100) /* Scan mode */
#define ADC_AWDSGL                              ((uint32_t)0x00000200) /* Enable the watchdog on a single channel in scan mode */
#define ADC_JAUTO                               ((uint32_t)0x00000400) /* Automatic injected group conversion */
#define ADC_DISCEN                              ((uint32_t)0x00000800) /* Discontinuous mode on regular channels */
#define ADC_JDISCEN                             ((uint32_t)0x00001000) /* Discontinuous mode on injected channels */

#define ADC_DISCNUM                             ((uint32_t)0x0000E000) /* DISCNUM[2:0] bits (Discontinuous mode channel count) */
#define ADC_DISCNUM_0                           ((uint32_t)0x00002000) /* Bit 0 */
#define ADC_DISCNUM_1                           ((uint32_t)0x00004000) /* Bit 1 */
#define ADC_DISCNUM_2                           ((uint32_t)0x00008000) /* Bit 2 */

#define ADC_JAWDEN                              ((uint32_t)0x00400000) /* Analog watchdog enable on injected channels */
#define ADC_AWDEN                               ((uint32_t)0x00800000) /* Analog watchdog enable on regular channels */
#define ADC_TKENABLE                            ((uint32_t)0x01000000) /* TKEN mode enable */

/*******************  Bit definition for ADC_CTLR2 register  ********************/
#define ADC_ADON                                ((uint32_t)0x00000001) /* A/D Converter ON / OFF */
#define ADC_CONT                                ((uint32_t)0x00000002) /* Continuous Conversion */
#define ADC_DMA                                 ((uint32_t)0x00000100) /* Direct Memory access mode */
#define ADC_ALIGN                               ((uint32_t)0x00000800) /* Data Alignment */

#define ADC_JEXTSEL                             ((uint32_t)0x00007000) /* JEXTSEL[2:0] bits (External event select for injected group) */
#define ADC_JEXTSEL_0                           ((uint32_t)0x00001000) /* Bit 0 */
#define ADC_JEXTSEL_1                           ((uint32_t)0x00002000) /* Bit 1 */
#define ADC_JEXTSEL_2                           ((uint32_t)0x00004000) /* Bit 2 */

#define ADC_JEXTTRIG                            ((uint32_t)0x00008000) /* External Trigger Conversion mode for injected channels */

#define ADC_EXTSEL                              ((uint32_t)0x000E0000) /* EXTSEL[2:0] bits (External Event Select for regular group) */
#define ADC_EXTSEL_0                            ((uint32_t)0x00020000) /* Bit 0 */
#define ADC_EXTSEL_1                            ((uint32_t)0x00040000) /* Bit 1 */
#define ADC_EXTSEL_2                            ((uint32_t)0x00080000) /* Bit 2 */

#define ADC_EXTTRIG                             ((uint32_t)0x00100000) /* External Trigger Conversion mode for regular channels */
#define ADC_JSWSTART                            ((uint32_t)0x00200000) /* Start Conversion of injected channels */
#define ADC_SWSTART                             ((uint32_t)0x00400000) /* Start Conversion of regular channels */

/******************  Bit definition for ADC_SAMPTR1 register  *******************/
#define ADC_SMP10                               ((uint32_t)0x00000007) /* SMP10[2:0] bits (Channel 10 Sample time selection) */
#define ADC_SMP10_0                             ((uint32_t)0x00000001) /* Bit 0 */
#define ADC_SMP10_1                             ((uint32_t)0x00000002) /* Bit 1 */
#define ADC_SMP10_2                             ((uint32_t)0x00000004) /* Bit 2 */

#define ADC_SMP11                               ((uint32_t)0x00000038) /* SMP11[2:0] bits (Channel 11 Sample time selection) */
#define ADC_SMP11_0                             ((uint32_t)0x00000008) /* Bit 0 */
#define ADC_SMP11_1                             ((uint32_t)0x00000010) /* Bit 1 */
#define ADC_SMP11_2                             ((uint32_t)0x00000020) /* Bit 2 */

#define ADC_SMP12                               ((uint32_t)0x000001C0) /* SMP12[2:0] bits (Channel 12 Sample time selection) */
#define ADC_SMP12_0                             ((uint32_t)0x00000040) /* Bit 0 */
#define ADC_SMP12_1                             ((uint32_t)0x00000080) /* Bit 1 */
#define ADC_SMP12_2                             ((uint32_t)0x00000100) /* Bit 2 */

#define ADC_SMP13                               ((uint32_t)0x00000E00) /* SMP13[2:0] bits (Channel 13 Sample time selection) */
#define ADC_SMP13_0                             ((uint32_t)0x00000200) /* Bit 0 */
#define ADC_SMP13_1                             ((uint32_t)0x00000400) /* Bit 1 */
#define ADC_SMP13_2                             ((uint32_t)0x00000800) /* Bit 2 */

#define ADC_SMP14                               ((uint32_t)0x00007000) /* SMP14[2:0] bits (Channel 14 Sample time selection) */
#define ADC_SMP14_0                             ((uint32_t)0x00001000) /* Bit 0 */
#define ADC_SMP14_1                             ((uint32_t)0x00002000) /* Bit 1 */
#define ADC_SMP14_2                             ((uint32_t)0x00004000) /* Bit 2 */

#define ADC_SMP15                               ((uint32_t)0x00038000) /* SMP15[2:0] bits (Channel 15 Sample time selection) */
#define ADC_SMP15_0                             ((uint32_t)0x00008000) /* Bit 0 */
#define ADC_SMP15_1                             ((uint32_t)0x00010000) /* Bit 1 */
#define ADC_SMP15_2                             ((uint32_t)0x00020000) /* Bit 2 */

#define ADC_SMP16                               ((uint32_t)0x001C0000) /* SMP16[2:0] bits (Channel 16 Sample time selection) */
#define ADC_SMP16_0                             ((uint32_t)0x00040000) /* Bit 0 */
#define ADC_SMP16_1                             ((uint32_t)0x00080000) /* Bit 1 */
#define ADC_SMP16_2                             ((uint32_t)0x00100000) /* Bit 2 */

#define ADC_SMP17                               ((uint32_t)0x00E00000) /* SMP17[2:0] bits (Channel 17 Sample time selection) */
#define ADC_SMP17_0                             ((uint32_t)0x00200000) /* Bit 0 */
#define ADC_SMP17_1                             ((uint32_t)0x00400000) /* Bit 1 */
#define ADC_SMP17_2                             ((uint32_t)0x00800000) /* Bit 2 */

/******************  Bit definition for ADC_SAMPTR2 register  *******************/
#define ADC_SMP0                                ((uint32_t)0x00000007) /* SMP0[2:0] bits (Channel 0 Sample time selection) */
#define ADC_SMP0_0                              ((uint32_t)0x00000001) /* Bit 0 */
#define ADC_SMP0_1                              ((uint32_t)0x00000002) /* Bit 1 */
#define ADC_SMP0_2                              ((uint32_t)0x00000004) /* Bit 2 */

#define ADC_SMP1                                ((uint32_t)0x00000038) /* SMP1[2:0] bits (Channel 1 Sample time selection) */
#define ADC_SMP1_0                              ((uint32_t)0x00000008) /* Bit 0 */
#define ADC_SMP1_1                              ((uint32_t)0x00000010) /* Bit 1 */
#define ADC_SMP1_2                              ((uint32_t)0x00000020) /* Bit 2 */

#define ADC_SMP2                                ((uint32_t)0x000001C0) /* SMP2[2:0] bits (Channel 2 Sample time selection) */
#define ADC_SMP2_0                              ((uint32_t)0x00000040) /* Bit 0 */
#define ADC_SMP2_1                              ((uint32_t)0x00000080) /* Bit 1 */
#define ADC_SMP2_2                              ((uint32_t)0x00000100) /* Bit 2 */

#define ADC_SMP3                                ((uint32_t)0x00000E00) /* SMP3[2:0] bits (Channel 3 Sample time selection) */
#define ADC_SMP3_0                              ((uint32_t)0x00000200) /* Bit 0 */
#define ADC_SMP3_1                              ((uint32_t)0x00000400) /* Bit 1 */
#define ADC_SMP3_2                              ((uint32_t)0x00000800) /* Bit 2 */

#define ADC_SMP4                                ((uint32_t)0x00007000) /* SMP4[2:0] bits (Channel 4 Sample time selection) */
#define ADC_SMP4_0                              ((uint32_t)0x00001000) /* Bit 0 */
#define ADC_SMP4_1                              ((uint32_t)0x00002000) /* Bit 1 */
#define ADC_SMP4_2                              ((uint32_t)0x00004000) /* Bit 2 */

#define ADC_SMP5                                ((uint32_t)0x00038000) /* SMP5[2:0] bits (Channel 5 Sample time selection) */
#define ADC_SMP5_0                              ((uint32_t)0x00008000) /* Bit 0 */
#define ADC_SMP5_1                              ((uint32_t)0x00010000) /* Bit 1 */
#define ADC_SMP5_2                              ((uint32_t)0x00020000) /* Bit 2 */

#define ADC_SMP6                                ((uint32_t)0x001C0000) /* SMP6[2:0] bits (Channel 6 Sample time selection) */
#define ADC_SMP6_0                              ((uint32_t)0x00040000) /* Bit 0 */
#define ADC_SMP6_1                              ((uint32_t)0x00080000) /* Bit 1 */
#define ADC_SMP6_2                              ((uint32_t)0x00100000) /* Bit 2 */

#define ADC_SMP7                                ((uint32_t)0x00E00000) /* SMP7[2:0] bits (Channel 7 Sample time selection) */
#define ADC_SMP7_0                              ((uint32_t)0x00200000) /* Bit 0 */
#define ADC_SMP7_1                              ((uint32_t)0x00400000) /* Bit 1 */
#define ADC_SMP7_2                              ((uint32_t)0x00800000) /* Bit 2 */

#define ADC_SMP8                                ((uint32_t)0x07000000) /* SMP8[2:0] bits (Channel 8 Sample time selection) */
#define ADC_SMP8_0                              ((uint32_t)0x01000000) /* Bit 0 */
#define ADC_SMP8_1                              ((uint32_t)0x02000000) /* Bit 1 */
#define ADC_SMP8_2                              ((uint32_t)0x04000000) /* Bit 2 */

#define ADC_SMP9                                ((uint32_t)0x38000000) /* SMP9[2:0] bits (Channel 9 Sample time selection) */
#define ADC_SMP9_0                              ((uint32_t)0x08000000) /* Bit 0 */
#define ADC_SMP9_1                              ((uint32_t)0x10000000) /* Bit 1 */
#define ADC_SMP9_2                              ((uint32_t)0x20000000) /* Bit 2 */

/******************  Bit definition for ADC_IOFR1 register  *******************/
#define ADC_JOFFSET1                            ((uint16_t)0x0FFF) /* Data offset for injected channel 1 */

/******************  Bit definition for ADC_IOFR2 register  *******************/
#define ADC_JOFFSET2                            ((uint16_t)0x0FFF) /* Data offset for injected channel 2 */

/******************  Bit definition for ADC_IOFR3 register  *******************/
#define ADC_JOFFSET3                            ((uint16_t)0x0FFF) /* Data offset for injected channel 3 */

/******************  Bit definition for ADC_IOFR4 register  *******************/
#define ADC_JOFFSET4                            ((uint16_t)0x0FFF) /* Data offset for injected channel 4 */

/*******************  Bit definition for ADC_WDHTR register  ********************/
#define ADC_HT                                  ((uint16_t)0x0FFF) /* Analog watchdog high threshold */

/*******************  Bit definition for ADC_WDLTR register  ********************/
#define ADC_LT                                  ((uint16_t)0x0FFF) /* Analog watchdog low threshold */

/*******************  Bit definition for ADC_RSQR1 register  *******************/
#define ADC_SQ13                                ((uint32_t)0x0000001F) /* SQ13[4:0] bits (13th conversion in regular sequence) */
#define ADC_SQ13_0                              ((uint32_t)0x00000001) /* Bit 0 */
#define ADC_SQ13_1                              ((uint32_t)0x00000002) /* Bit 1 */
#define ADC_SQ13_2                              ((uint32_t)0x00000004) /* Bit 2 */
#define ADC_SQ13_3                              ((uint32_t)0x00000008) /* Bit 3 */
#define ADC_SQ13_4                              ((uint32_t)0x00000010) /* Bit 4 */

#define ADC_SQ14                                ((uint32_t)0x000003E0) /* SQ14[4:0] bits (14th conversion in regular sequence) */
#define ADC_SQ14_0                              ((uint32_t)0x00000020) /* Bit 0 */
#define ADC_SQ14_1                              ((uint32_t)0x00000040) /* Bit 1 */
#define ADC_SQ14_2                              ((uint32_t)0x00000080) /* Bit 2 */
#define ADC_SQ14_3                              ((uint32_t)0x00000100) /* Bit 3 */
#define ADC_SQ14_4                              ((uint32_t)0x00000200) /* Bit 4 */

#define ADC_SQ15                                ((uint32_t)0x00007C00) /* SQ15[4:0] bits (15th conversion in regular sequence) */
#define ADC_SQ15_0                              ((uint32_t)0x00000400) /* Bit 0 */
#define ADC_SQ15_1                              ((uint32_t)0x00000800) /* Bit 1 */
#define ADC_SQ15_2                              ((uint32_t)0x00001000) /* Bit 2 */
#define ADC_SQ15_3                              ((uint32_t)0x00002000) /* Bit 3 */
#define ADC_SQ15_4                              ((uint32_t)0x00004000) /* Bit 4 */

#define ADC_SQ16                                ((uint32_t)0x000F8000) /* SQ16[4:0] bits (16th conversion in regular sequence) */
#define ADC_SQ16_0                              ((uint32_t)0x00008000) /* Bit 0 */
#define ADC_SQ16_1                              ((uint32_t)0x00010000) /* Bit 1 */
#define ADC_SQ16_2                              ((uint32_t)0x00020000) /* Bit 2 */
#define ADC_SQ16_3                              ((uint32_t)0x00040000) /* Bit 3 */
#define ADC_SQ16_4                              ((uint32_t)0x00080000) /* Bit 4 */

#define ADC_L                                   ((uint32_t)0x00F00000) /* L[3:0] bits (Regular channel sequence length) */
#define ADC_L_0                                 ((uint32_t)0x00100000) /* Bit 0 */
#define ADC_L_1                                 ((uint32_t)0x00200000) /* Bit 1 */
#define ADC_L_2                                 ((uint32_t)0x00400000) /* Bit 2 */
#define ADC_L_3                                 ((uint32_t)0x00800000) /* Bit 3 */

/*******************  Bit definition for ADC_RSQR2 register  *******************/
#define ADC_SQ7                                 ((uint32_t)0x0000001F) /* SQ7[4:0] bits (7th conversion in regular sequence) */
#define ADC_SQ7_0                               ((uint32_t)0x00000001) /* Bit 0 */
#define ADC_SQ7_1                               ((uint32_t)0x00000002) /* Bit 1 */
#define ADC_SQ7_2                               ((uint32_t)0x00000004) /* Bit 2 */
#define ADC_SQ7_3                               ((uint32_t)0x00000008) /* Bit 3 */
#define ADC_SQ7_4                               ((uint32_t)0x00000010) /* Bit 4 */

#define ADC_SQ8                                 ((uint32_t)0x000003E0) /* SQ8[4:0] bits (8th conversion in regular sequence) */
#define ADC_SQ8_0                               ((uint32_t)0x00000020) /* Bit 0 */
#define ADC_SQ8_1                               ((uint32_t)0x00000040) /* Bit 1 */
#define ADC_SQ8_2                               ((uint32_t)0x00000080) /* Bit 2 */
#define ADC_SQ8_3                               ((uint32_t)0x00000100) /* Bit 3 */
#define ADC_SQ8_4                               ((uint32_t)0x00000200) /* Bit 4 */

#define ADC_SQ9                                 ((uint32_t)0x00007C00) /* SQ9[4:0] bits (9th conversion in regular sequence) */
#define ADC_SQ9_0                               ((uint32_t)0x00000400) /* Bit 0 */
#define ADC_SQ9_1                               ((uint32_t)0x00000800) /* Bit 1 */
#define ADC_SQ9_2                               ((uint32_t)0x00001000) /* Bit 2 */
#define ADC_SQ9_3                               ((uint32_t)0x00002000) /* Bit 3 */
#define ADC_SQ9_4                               ((uint32_t)0x00004000) /* Bit 4 */

#define ADC_SQ10                                ((uint32_t)0x000F8000) /* SQ10[4:0] bits (10th conversion in regular sequence) */
#define ADC_SQ10_0                              ((uint32_t)0x00008000) /* Bit 0 */
#define ADC_SQ10_1                              ((uint32_t)0x00010000) /* Bit 1 */
#define ADC_SQ10_2                              ((uint32_t)0x00020000) /* Bit 2 */
#define ADC_SQ10_3                              ((uint32_t)0x00040000) /* Bit 3 */
#define ADC_SQ10_4                              ((uint32_t)0x00080000) /* Bit 4 */

#define ADC_SQ11                                ((uint32_t)0x01F00000) /* SQ11[4:0] bits (11th conversion in regular sequence) */
#define ADC_SQ11_0                              ((uint32_t)0x00100000) /* Bit 0 */
#define ADC_SQ11_1                              ((uint32_t)0x00200000) /* Bit 1 */
#define ADC_SQ11_2                              ((uint32_t)0x00400000) /* Bit 2 */
#define ADC_SQ11_3                              ((uint32_t)0x00800000) /* Bit 3 */
#define ADC_SQ11_4                              ((uint32_t)0x01000000) /* Bit 4 */

#define ADC_SQ12                                ((uint32_t)0x3E000000) /* SQ12[4:0] bits (12th conversion in regular sequence) */
#define ADC_SQ12_0                              ((uint32_t)0x02000000) /* Bit 0 */
#define ADC_SQ12_1                              ((uint32_t)0x04000000) /* Bit 1 */
#define ADC_SQ12_2                              ((uint32_t)0x08000000) /* Bit 2 */
#define ADC_SQ12_3                              ((uint32_t)0x10000000) /* Bit 3 */
#define ADC_SQ12_4                              ((uint32_t)0x20000000) /* Bit 4 */

/*******************  Bit definition for ADC_RSQR3 register  *******************/
#define ADC_SQ1                                 ((uint32_t)0x0000001F) /* SQ1[4:0] bits (1st conversion in regular sequence) */
#define ADC_SQ1_0                               ((uint32_t)0x00000001) /* Bit 0 */
#define ADC_SQ1_1                               ((uint32_t)0x00000002) /* Bit 1 */
#define ADC_SQ1_2                               ((uint32_t)0x00000004) /* Bit 2 */
#define ADC_SQ1_3                               ((uint32_t)0x00000008) /* Bit 3 */
#define ADC_SQ1_4                               ((uint32_t)0x00000010) /* Bit 4 */

#define ADC_SQ2                                 ((uint32_t)0x000003E0) /* SQ2[4:0] bits (2nd conversion in regular sequence) */
#define ADC_SQ2_0                               ((uint32_t)0x00000020) /* Bit 0 */
#define ADC_SQ2_1                               ((uint32_t)0x00000040) /* Bit 1 */
#define ADC_SQ2_2                               ((uint32_t)0x00000080) /* Bit 2 */
#define ADC_SQ2_3                               ((uint32_t)0x00000100) /* Bit 3 */
#define ADC_SQ2_4                               ((uint32_t)0x00000200) /* Bit 4 */

#define ADC_SQ3                                 ((uint32_t)0x00007C00) /* SQ3[4:0] bits (3rd conversion in regular sequence) */
#define ADC_SQ3_0                               ((uint32_t)0x00000400) /* Bit 0 */
#define ADC_SQ3_1                               ((uint32_t)0x00000800) /* Bit 1 */
#define ADC_SQ3_2                               ((uint32_t)0x00001000) /* Bit 2 */
#define ADC_SQ3_3                               ((uint32_t)0x00002000) /* Bit 3 */
#define ADC_SQ3_4                               ((uint32_t)0x00004000) /* Bit 4 */

#define ADC_SQ4                                 ((uint32_t)0x000F8000) /* SQ4[4:0] bits (4th conversion in regular sequence) */
#define ADC_SQ4_0                               ((uint32_t)0x00008000) /* Bit 0 */
#define ADC_SQ4_1                               ((uint32_t)0x00010000) /* Bit 1 */
#define ADC_SQ4_2                               ((uint32_t)0x00020000) /* Bit 2 */
#define ADC_SQ4_3                               ((uint32_t)0x00040000) /* Bit 3 */
#define ADC_SQ4_4                               ((uint32_t)0x00080000) /* Bit 4 */

#define ADC_SQ5                                 ((uint32_t)0x01F00000) /* SQ5[4:0] bits (5th conversion in regular sequence) */
#define ADC_SQ5_0                               ((uint32_t)0x00100000) /* Bit 0 */
#define ADC_SQ5_1                               ((uint32_t)0x00200000) /* Bit 1 */
#define ADC_SQ5_2                               ((uint32_t)0x00400000) /* Bit 2 */
#define ADC_SQ5_3                               ((uint32_t)0x00800000) /* Bit 3 */
#define ADC_SQ5_4                               ((uint32_t)0x01000000) /* Bit 4 */

#define ADC_SQ6                                 ((uint32_t)0x3E000000) /* SQ6[4:0] bits (6th conversion in regular sequence) */
#define ADC_SQ6_0                               ((uint32_t)0x02000000) /* Bit 0 */
#define ADC_SQ6_1                               ((uint32_t)0x04000000) /* Bit 1 */
#define ADC_SQ6_2                               ((uint32_t)0x08000000) /* Bit 2 */
#define ADC_SQ6_3                               ((uint32_t)0x10000000) /* Bit 3 */
#define ADC_SQ6_4                               ((uint32_t)0x20000000) /* Bit 4 */

/*******************  Bit definition for ADC_ISQR register  *******************/
#define ADC_JSQ1                                ((uint32_t)0x0000001F) /* JSQ1[4:0] bits (1st conversion in injected sequence) */
#define ADC_JSQ1_0                              ((uint32_t)0x00000001) /* Bit 0 */
#define ADC_JSQ1_1                              ((uint32_t)0x00000002) /* Bit 1 */
#define ADC_JSQ1_2                              ((uint32_t)0x00000004) /* Bit 2 */
#define ADC_JSQ1_3                              ((uint32_t)0x00000008) /* Bit 3 */
#define ADC_JSQ1_4                              ((uint32_t)0x00000010) /* Bit 4 */

#define ADC_JSQ2                                ((uint32_t)0x000003E0) /* JSQ2[4:0] bits (2nd conversion in injected sequence) */
#define ADC_JSQ2_0                              ((uint32_t)0x00000020) /* Bit 0 */
#define ADC_JSQ2_1                              ((uint32_t)0x00000040) /* Bit 1 */
#define ADC_JSQ2_2                              ((uint32_t)0x00000080) /* Bit 2 */
#define ADC_JSQ2_3                              ((uint32_t)0x00000100) /* Bit 3 */
#define ADC_JSQ2_4                              ((uint32_t)0x00000200) /* Bit 4 */

#define ADC_JSQ3                                ((uint32_t)0x00007C00) /* JSQ3[4:0] bits (3rd conversion in injected sequence) */
#define ADC_JSQ3_0                              ((uint32_t)0x00000400) /* Bit 0 */
#define ADC_JSQ3_1                              ((uint32_t)0x00000800) /* Bit 1 */
#define ADC_JSQ3_2                              ((uint32_t)0x00001000) /* Bit 2 */
#define ADC_JSQ3_3                              ((uint32_t)0x00002000) /* Bit 3 */
#define ADC_JSQ3_4                              ((uint32_t)0x00004000) /* Bit 4 */

#define ADC_JSQ4                                ((uint32_t)0x000F8000) /* JSQ4[4:0] bits (4th conversion in injected sequence) */
#define ADC_JSQ4_0                              ((uint32_t)0x00008000) /* Bit 0 */
#define ADC_JSQ4_1                              ((uint32_t)0x00010000) /* Bit 1 */
#define ADC_JSQ4_2                              ((uint32_t)0x00020000) /* Bit 2 */
#define ADC_JSQ4_3                              ((uint32_t)0x00040000) /* Bit 3 */
#define ADC_JSQ4_4                              ((uint32_t)0x00080000) /* Bit 4 */

#define ADC_JL                                  ((uint32_t)0x00300000) /* JL[1:0] bits (Injected Sequence length) */
#define ADC_JL_0                                ((uint32_t)0x00100000) /* Bit 0 */
#define ADC_JL_1                                ((uint32_t)0x00200000) /* Bit 1 */

/*******************  Bit definition for ADC_IDATAR1 register  *******************/
#define ADC_IDATAR1_JDATA                       ((uint16_t)0xFFFF) /* Injected data */

/*******************  Bit definition for ADC_IDATAR2 register  *******************/
#define ADC_IDATAR2_JDATA                       ((uint16_t)0xFFFF) /* Injected data */

/*******************  Bit definition for ADC_IDATAR3 register  *******************/
#define ADC_IDATAR3_JDATA                       ((uint16_t)0xFFFF) /* Injected data */

/*******************  Bit definition for ADC_IDATAR4 register  *******************/
#define ADC_IDATAR4_JDATA                       ((uint16_t)0xFFFF) /* Injected data */

/********************  Bit definition for ADC_RDATAR register  ********************/
#define ADC_RDATAR_DATA                         ((uint32_t)0x0000FFFF) /* Regular data */

/********************  Bit definition for ADC_CTLR3 register  ********************/
#define ADC_CTLR3_CLK_DIV                       ((uint32_t)0x0000000F) /* CLK_DIVL[3:0] bits */
#define ADC_CTLR3_CLK_DIV_0                     ((uint32_t)0x00000001) /* Bit 0 */
#define ADC_CTLR3_CLK_DIV_1                     ((uint32_t)0x00000002) /* Bit 1 */
#define ADC_CTLR3_CLK_DIV_2                     ((uint32_t)0x00000004) /* Bit 2 */
#define ADC_CTLR3_CLK_DIV_3                     ((uint32_t)0x00000008) /* Bit 3 */

#define ADC_CTLR3_AWD_SCAN                      ((uint32_t)0x00000200) /* Analog watchdog Scan enable */
#define ADC_CTLR3_AWD0_RST_EN                   ((uint32_t)0x00001000) /* Watchdog0 Reset Enable */
#define ADC_CTLR3_AWD1_RST_EN                   ((uint32_t)0x00002000) /* Watchdog1 Reset Enable */
#define ADC_CTLR3_AWD2_RST_EN                   ((uint32_t)0x00004000) /* Watchdog2 Reset Enable */
#define ADC_CTLR3_AWD3_RST_EN                   ((uint32_t)0x00008000) /* Watchdog3 Reset Enable */

/********************  Bit definition for ADC_WDTR1 register  ********************/
#define ADC_WDTR1_LTR1                          ((uint32_t)0x00000FFF) /* Analog watchdog1 low threshold */
#define ADC_WDTR1_HTR1                          ((uint32_t)0x0FFF0000) /* Analog watchdog1 high threshold */

/********************  Bit definition for ADC_WDTR2 register  ********************/
#define ADC_WDTR2_LTR2                          ((uint32_t)0x00000FFF) /* Analog watchdog2 low threshold */
#define ADC_WDTR2_HTR2                          ((uint32_t)0x0FFF0000) /* Analog watchdog2 high threshold */

/********************  Bit definition for ADC_WDTR3 register  ********************/
#define ADC_WDTR3_LTR3                          ((uint32_t)0x00000FFF) /* Analog watchdog3 low threshold */
#define ADC_WDTR3_HTR3                          ((uint32_t)0x0FFF0000) /* Analog watchdog3 high threshold */

/******************************************************************************/
/*                             DMA Controller                                 */
/******************************************************************************/

/*******************  Bit definition for DMA_INTFR register  ********************/
#define DMA_GIF1                                ((uint32_t)0x00000001) /* Channel 1 Global interrupt flag */
#define DMA_TCIF1                               ((uint32_t)0x00000002) /* Channel 1 Transfer Complete flag */
#define DMA_HTIF1                               ((uint32_t)0x00000004) /* Channel 1 Half Transfer flag */
#define DMA_TEIF1                               ((uint32_t)0x00000008) /* Channel 1 Transfer Error flag */
#define DMA_GIF2                                ((uint32_t)0x00000010) /* Channel 2 Global interrupt flag */
#define DMA_TCIF2                               ((uint32_t)0x00000020) /* Channel 2 Transfer Complete flag */
#define DMA_HTIF2                               ((uint32_t)0x00000040) /* Channel 2 Half Transfer flag */
#define DMA_TEIF2                               ((uint32_t)0x00000080) /* Channel 2 Transfer Error flag */
#define DMA_GIF3                                ((uint32_t)0x00000100) /* Channel 3 Global interrupt flag */
#define DMA_TCIF3                               ((uint32_t)0x00000200) /* Channel 3 Transfer Complete flag */
#define DMA_HTIF3                               ((uint32_t)0x00000400) /* Channel 3 Half Transfer flag */
#define DMA_TEIF3                               ((uint32_t)0x00000800) /* Channel 3 Transfer Error flag */
#define DMA_GIF4                                ((uint32_t)0x00001000) /* Channel 4 Global interrupt flag */
#define DMA_TCIF4                               ((uint32_t)0x00002000) /* Channel 4 Transfer Complete flag */
#define DMA_HTIF4                               ((uint32_t)0x00004000) /* Channel 4 Half Transfer flag */
#define DMA_TEIF4                               ((uint32_t)0x00008000) /* Channel 4 Transfer Error flag */
#define DMA_GIF5                                ((uint32_t)0x00010000) /* Channel 5 Global interrupt flag */
#define DMA_TCIF5                               ((uint32_t)0x00020000) /* Channel 5 Transfer Complete flag */
#define DMA_HTIF5                               ((uint32_t)0x00040000) /* Channel 5 Half Transfer flag */
#define DMA_TEIF5                               ((uint32_t)0x00080000) /* Channel 5 Transfer Error flag */
#define DMA_GIF6                                ((uint32_t)0x00100000) /* Channel 6 Global interrupt flag */
#define DMA_TCIF6                               ((uint32_t)0x00200000) /* Channel 6 Transfer Complete flag */
#define DMA_HTIF6                               ((uint32_t)0x00400000) /* Channel 6 Half Transfer flag */
#define DMA_TEIF6                               ((uint32_t)0x00800000) /* Channel 6 Transfer Error flag */
#define DMA_GIF7                                ((uint32_t)0x01000000) /* Channel 7 Global interrupt flag */
#define DMA_TCIF7                               ((uint32_t)0x02000000) /* Channel 7 Transfer Complete flag */
#define DMA_HTIF7                               ((uint32_t)0x04000000) /* Channel 7 Half Transfer flag */
#define DMA_TEIF7                               ((uint32_t)0x08000000) /* Channel 7 Transfer Error flag */
#define DMA_GIF8                                ((uint32_t)0x10000000) /* Channel 8 Global interrupt flag */
#define DMA_TCIF8                               ((uint32_t)0x20000000) /* Channel 8 Transfer Complete flag */
#define DMA_HTIF8                               ((uint32_t)0x40000000) /* Channel 8 Half Transfer flag */
#define DMA_TEIF8                               ((uint32_t)0x80000000) /* Channel 8 Transfer Error flag */

/*******************  Bit definition for DMA_INTFCR register  *******************/
#define DMA_CGIF1                               ((uint32_t)0x00000001) /* Channel 1 Global interrupt clear */
#define DMA_CTCIF1                              ((uint32_t)0x00000002) /* Channel 1 Transfer Complete clear */
#define DMA_CHTIF1                              ((uint32_t)0x00000004) /* Channel 1 Half Transfer clear */
#define DMA_CTEIF1                              ((uint32_t)0x00000008) /* Channel 1 Transfer Error clear */
#define DMA_CGIF2                               ((uint32_t)0x00000010) /* Channel 2 Global interrupt clear */
#define DMA_CTCIF2                              ((uint32_t)0x00000020) /* Channel 2 Transfer Complete clear */
#define DMA_CHTIF2                              ((uint32_t)0x00000040) /* Channel 2 Half Transfer clear */
#define DMA_CTEIF2                              ((uint32_t)0x00000080) /* Channel 2 Transfer Error clear */
#define DMA_CGIF3                               ((uint32_t)0x00000100) /* Channel 3 Global interrupt clear */
#define DMA_CTCIF3                              ((uint32_t)0x00000200) /* Channel 3 Transfer Complete clear */
#define DMA_CHTIF3                              ((uint32_t)0x00000400) /* Channel 3 Half Transfer clear */
#define DMA_CTEIF3                              ((uint32_t)0x00000800) /* Channel 3 Transfer Error clear */
#define DMA_CGIF4                               ((uint32_t)0x00001000) /* Channel 4 Global interrupt clear */
#define DMA_CTCIF4                              ((uint32_t)0x00002000) /* Channel 4 Transfer Complete clear */
#define DMA_CHTIF4                              ((uint32_t)0x00004000) /* Channel 4 Half Transfer clear */
#define DMA_CTEIF4                              ((uint32_t)0x00008000) /* Channel 4 Transfer Error clear */
#define DMA_CGIF5                               ((uint32_t)0x00010000) /* Channel 5 Global interrupt clear */
#define DMA_CTCIF5                              ((uint32_t)0x00020000) /* Channel 5 Transfer Complete clear */
#define DMA_CHTIF5                              ((uint32_t)0x00040000) /* Channel 5 Half Transfer clear */
#define DMA_CTEIF5                              ((uint32_t)0x00080000) /* Channel 5 Transfer Error clear */
#define DMA_CGIF6                               ((uint32_t)0x00100000) /* Channel 6 Global interrupt clear */
#define DMA_CTCIF6                              ((uint32_t)0x00200000) /* Channel 6 Transfer Complete clear */
#define DMA_CHTIF6                              ((uint32_t)0x00400000) /* Channel 6 Half Transfer clear */
#define DMA_CTEIF6                              ((uint32_t)0x00800000) /* Channel 6 Transfer Error clear */
#define DMA_CGIF7                               ((uint32_t)0x01000000) /* Channel 7 Global interrupt clear */
#define DMA_CTCIF7                              ((uint32_t)0x02000000) /* Channel 7 Transfer Complete clear */
#define DMA_CHTIF7                              ((uint32_t)0x04000000) /* Channel 7 Half Transfer clear */
#define DMA_CTEIF7                              ((uint32_t)0x08000000) /* Channel 7 Transfer Error clear */
#define DMA_CGIF8                               ((uint32_t)0x10000000) /* Channel 8 Global interrupt clear */
#define DMA_CTCIF8                              ((uint32_t)0x20000000) /* Channel 8 Transfer Complete clear */
#define DMA_CHTIF8                              ((uint32_t)0x40000000) /* Channel 8 Half Transfer clear */
#define DMA_CTEIF8                              ((uint32_t)0x80000000) /* Channel 8 Transfer Error clear */

/*******************  Bit definition for DMA_CFGR1 register  *******************/
#define DMA_CFGR1_EN                            ((uint16_t)0x0001) /* Channel enable*/
#define DMA_CFGR1_TCIE                          ((uint16_t)0x0002) /* Transfer complete interrupt enable */
#define DMA_CFGR1_HTIE                          ((uint16_t)0x0004) /* Half Transfer interrupt enable */
#define DMA_CFGR1_TEIE                          ((uint16_t)0x0008) /* Transfer error interrupt enable */
#define DMA_CFGR1_DIR                           ((uint16_t)0x0010) /* Data transfer direction */
#define DMA_CFGR1_CIRC                          ((uint16_t)0x0020) /* Circular mode */
#define DMA_CFGR1_PINC                          ((uint16_t)0x0040) /* Peripheral increment mode */
#define DMA_CFGR1_MINC                          ((uint16_t)0x0080) /* Memory increment mode */

#define DMA_CFGR1_PSIZE                         ((uint16_t)0x0300) /* PSIZE[1:0] bits (Peripheral size) */
#define DMA_CFGR1_PSIZE_0                       ((uint16_t)0x0100) /* Bit 0 */
#define DMA_CFGR1_PSIZE_1                       ((uint16_t)0x0200) /* Bit 1 */

#define DMA_CFGR1_MSIZE                         ((uint16_t)0x0C00) /* MSIZE[1:0] bits (Memory size) */
#define DMA_CFGR1_MSIZE_0                       ((uint16_t)0x0400) /* Bit 0 */
#define DMA_CFGR1_MSIZE_1                       ((uint16_t)0x0800) /* Bit 1 */

#define DMA_CFGR1_PL                            ((uint16_t)0x3000) /* PL[1:0] bits(Channel Priority level) */
#define DMA_CFGR1_PL_0                          ((uint16_t)0x1000) /* Bit 0 */
#define DMA_CFGR1_PL_1                          ((uint16_t)0x2000) /* Bit 1 */

#define DMA_CFGR1_MEM2MEM                       ((uint16_t)0x4000) /* Memory to memory mode */

/*******************  Bit definition for DMA_CFGR2 register  *******************/
#define DMA_CFGR2_EN                            ((uint16_t)0x0001) /* Channel enable */
#define DMA_CFGR2_TCIE                          ((uint16_t)0x0002) /* Transfer complete interrupt enable */
#define DMA_CFGR2_HTIE                          ((uint16_t)0x0004) /* Half Transfer interrupt enable */
#define DMA_CFGR2_TEIE                          ((uint16_t)0x0008) /* Transfer error interrupt enable */
#define DMA_CFGR2_DIR                           ((uint16_t)0x0010) /* Data transfer direction */
#define DMA_CFGR2_CIRC                          ((uint16_t)0x0020) /* Circular mode */
#define DMA_CFGR2_PINC                          ((uint16_t)0x0040) /* Peripheral increment mode */
#define DMA_CFGR2_MINC                          ((uint16_t)0x0080) /* Memory increment mode */

#define DMA_CFGR2_PSIZE                         ((uint16_t)0x0300) /* PSIZE[1:0] bits (Peripheral size) */
#define DMA_CFGR2_PSIZE_0                       ((uint16_t)0x0100) /* Bit 0 */
#define DMA_CFGR2_PSIZE_1                       ((uint16_t)0x0200) /* Bit 1 */

#define DMA_CFGR2_MSIZE                         ((uint16_t)0x0C00) /* MSIZE[1:0] bits (Memory size) */
#define DMA_CFGR2_MSIZE_0                       ((uint16_t)0x0400) /* Bit 0 */
#define DMA_CFGR2_MSIZE_1                       ((uint16_t)0x0800) /* Bit 1 */

#define DMA_CFGR2_PL                            ((uint16_t)0x3000) /* PL[1:0] bits (Channel Priority level) */
#define DMA_CFGR2_PL_0                          ((uint16_t)0x1000) /* Bit 0 */
#define DMA_CFGR2_PL_1                          ((uint16_t)0x2000) /* Bit 1 */

#define DMA_CFGR2_MEM2MEM                       ((uint16_t)0x4000) /* Memory to memory mode */

/*******************  Bit definition for DMA_CFGR3 register  *******************/
#define DMA_CFGR3_EN                            ((uint16_t)0x0001) /* Channel enable */
#define DMA_CFGR3_TCIE                          ((uint16_t)0x0002) /* Transfer complete interrupt enable */
#define DMA_CFGR3_HTIE                          ((uint16_t)0x0004) /* Half Transfer interrupt enable */
#define DMA_CFGR3_TEIE                          ((uint16_t)0x0008) /* Transfer error interrupt enable */
#define DMA_CFGR3_DIR                           ((uint16_t)0x0010) /* Data transfer direction */
#define DMA_CFGR3_CIRC                          ((uint16_t)0x0020) /* Circular mode */
#define DMA_CFGR3_PINC                          ((uint16_t)0x0040) /* Peripheral increment mode */
#define DMA_CFGR3_MINC                          ((uint16_t)0x0080) /* Memory increment mode */

#define DMA_CFGR3_PSIZE                         ((uint16_t)0x0300) /* PSIZE[1:0] bits (Peripheral size) */
#define DMA_CFGR3_PSIZE_0                       ((uint16_t)0x0100) /* Bit 0 */
#define DMA_CFGR3_PSIZE_1                       ((uint16_t)0x0200) /* Bit 1 */

#define DMA_CFGR3_MSIZE                         ((uint16_t)0x0C00) /* MSIZE[1:0] bits (Memory size) */
#define DMA_CFGR3_MSIZE_0                       ((uint16_t)0x0400) /* Bit 0 */
#define DMA_CFGR3_MSIZE_1                       ((uint16_t)0x0800) /* Bit 1 */

#define DMA_CFGR3_PL                            ((uint16_t)0x3000) /* PL[1:0] bits (Channel Priority level) */
#define DMA_CFGR3_PL_0                          ((uint16_t)0x1000) /* Bit 0 */
#define DMA_CFGR3_PL_1                          ((uint16_t)0x2000) /* Bit 1 */

#define DMA_CFGR3_MEM2MEM                       ((uint16_t)0x4000) /* Memory to memory mode */

/*******************  Bit definition for DMA_CFG4 register  *******************/
#define DMA_CFG4_EN                             ((uint16_t)0x0001) /* Channel enable */
#define DMA_CFG4_TCIE                           ((uint16_t)0x0002) /* Transfer complete interrupt enable */
#define DMA_CFG4_HTIE                           ((uint16_t)0x0004) /* Half Transfer interrupt enable */
#define DMA_CFG4_TEIE                           ((uint16_t)0x0008) /* Transfer error interrupt enable */
#define DMA_CFG4_DIR                            ((uint16_t)0x0010) /* Data transfer direction */
#define DMA_CFG4_CIRC                           ((uint16_t)0x0020) /* Circular mode */
#define DMA_CFG4_PINC                           ((uint16_t)0x0040) /* Peripheral increment mode */
#define DMA_CFG4_MINC                           ((uint16_t)0x0080) /* Memory increment mode */

#define DMA_CFG4_PSIZE                          ((uint16_t)0x0300) /* PSIZE[1:0] bits (Peripheral size) */
#define DMA_CFG4_PSIZE_0                        ((uint16_t)0x0100) /* Bit 0 */
#define DMA_CFG4_PSIZE_1                        ((uint16_t)0x0200) /* Bit 1 */

#define DMA_CFG4_MSIZE                          ((uint16_t)0x0C00) /* MSIZE[1:0] bits (Memory size) */
#define DMA_CFG4_MSIZE_0                        ((uint16_t)0x0400) /* Bit 0 */
#define DMA_CFG4_MSIZE_1                        ((uint16_t)0x0800) /* Bit 1 */

#define DMA_CFG4_PL                             ((uint16_t)0x3000) /* PL[1:0] bits (Channel Priority level) */
#define DMA_CFG4_PL_0                           ((uint16_t)0x1000) /* Bit 0 */
#define DMA_CFG4_PL_1                           ((uint16_t)0x2000) /* Bit 1 */

#define DMA_CFG4_MEM2MEM                        ((uint16_t)0x4000) /* Memory to memory mode */

/******************  Bit definition for DMA_CFG5 register  *******************/
#define DMA_CFG5_EN                             ((uint16_t)0x0001) /* Channel enable */
#define DMA_CFG5_TCIE                           ((uint16_t)0x0002) /* Transfer complete interrupt enable */
#define DMA_CFG5_HTIE                           ((uint16_t)0x0004) /* Half Transfer interrupt enable */
#define DMA_CFG5_TEIE                           ((uint16_t)0x0008) /* Transfer error interrupt enable */
#define DMA_CFG5_DIR                            ((uint16_t)0x0010) /* Data transfer direction */
#define DMA_CFG5_CIRC                           ((uint16_t)0x0020) /* Circular mode */
#define DMA_CFG5_PINC                           ((uint16_t)0x0040) /* Peripheral increment mode */
#define DMA_CFG5_MINC                           ((uint16_t)0x0080) /* Memory increment mode */

#define DMA_CFG5_PSIZE                          ((uint16_t)0x0300) /* PSIZE[1:0] bits (Peripheral size) */
#define DMA_CFG5_PSIZE_0                        ((uint16_t)0x0100) /* Bit 0 */
#define DMA_CFG5_PSIZE_1                        ((uint16_t)0x0200) /* Bit 1 */

#define DMA_CFG5_MSIZE                          ((uint16_t)0x0C00) /* MSIZE[1:0] bits (Memory size) */
#define DMA_CFG5_MSIZE_0                        ((uint16_t)0x0400) /* Bit 0 */
#define DMA_CFG5_MSIZE_1                        ((uint16_t)0x0800) /* Bit 1 */

#define DMA_CFG5_PL                             ((uint16_t)0x3000) /* PL[1:0] bits (Channel Priority level) */
#define DMA_CFG5_PL_0                           ((uint16_t)0x1000) /* Bit 0 */
#define DMA_CFG5_PL_1                           ((uint16_t)0x2000) /* Bit 1 */

#define DMA_CFG5_MEM2MEM                        ((uint16_t)0x4000) /* Memory to memory mode enable */

/*******************  Bit definition for DMA_CFG6 register  *******************/
#define DMA_CFG6_EN                             ((uint16_t)0x0001) /* Channel enable */
#define DMA_CFG6_TCIE                           ((uint16_t)0x0002) /* Transfer complete interrupt enable */
#define DMA_CFG6_HTIE                           ((uint16_t)0x0004) /* Half Transfer interrupt enable */
#define DMA_CFG6_TEIE                           ((uint16_t)0x0008) /* Transfer error interrupt enable */
#define DMA_CFG6_DIR                            ((uint16_t)0x0010) /* Data transfer direction */
#define DMA_CFG6_CIRC                           ((uint16_t)0x0020) /* Circular mode */
#define DMA_CFG6_PINC                           ((uint16_t)0x0040) /* Peripheral increment mode */
#define DMA_CFG6_MINC                           ((uint16_t)0x0080) /* Memory increment mode */

#define DMA_CFG6_PSIZE                          ((uint16_t)0x0300) /* PSIZE[1:0] bits (Peripheral size) */
#define DMA_CFG6_PSIZE_0                        ((uint16_t)0x0100) /* Bit 0 */
#define DMA_CFG6_PSIZE_1                        ((uint16_t)0x0200) /* Bit 1 */

#define DMA_CFG6_MSIZE                          ((uint16_t)0x0C00) /* MSIZE[1:0] bits (Memory size) */
#define DMA_CFG6_MSIZE_0                        ((uint16_t)0x0400) /* Bit 0 */
#define DMA_CFG6_MSIZE_1                        ((uint16_t)0x0800) /* Bit 1 */

#define DMA_CFG6_PL                             ((uint16_t)0x3000) /* PL[1:0] bits (Channel Priority level) */
#define DMA_CFG6_PL_0                           ((uint16_t)0x1000) /* Bit 0 */
#define DMA_CFG6_PL_1                           ((uint16_t)0x2000) /* Bit 1 */

#define DMA_CFG6_MEM2MEM                        ((uint16_t)0x4000) /* Memory to memory mode */

/*******************  Bit definition for DMA_CFG7 register  *******************/
#define DMA_CFG7_EN                             ((uint16_t)0x0001) /* Channel enable */
#define DMA_CFG7_TCIE                           ((uint16_t)0x0002) /* Transfer complete interrupt enable */
#define DMA_CFG7_HTIE                           ((uint16_t)0x0004) /* Half Transfer interrupt enable */
#define DMA_CFG7_TEIE                           ((uint16_t)0x0008) /* Transfer error interrupt enable */
#define DMA_CFG7_DIR                            ((uint16_t)0x0010) /* Data transfer direction */
#define DMA_CFG7_CIRC                           ((uint16_t)0x0020) /* Circular mode */
#define DMA_CFG7_PINC                           ((uint16_t)0x0040) /* Peripheral increment mode */
#define DMA_CFG7_MINC                           ((uint16_t)0x0080) /* Memory increment mode */

#define DMA_CFG7_PSIZE                          ((uint16_t)0x0300) /* PSIZE[1:0] bits (Peripheral size) */
#define DMA_CFG7_PSIZE_0                        ((uint16_t)0x0100) /* Bit 0 */
#define DMA_CFG7_PSIZE_1                        ((uint16_t)0x0200) /* Bit 1 */

#define DMA_CFG7_MSIZE                          ((uint16_t)0x0C00) /* MSIZE[1:0] bits (Memory size) */
#define DMA_CFG7_MSIZE_0                        ((uint16_t)0x0400) /* Bit 0 */
#define DMA_CFG7_MSIZE_1                        ((uint16_t)0x0800) /* Bit 1 */

#define DMA_CFG7_PL                             ((uint16_t)0x3000) /* PL[1:0] bits (Channel Priority level) */
#define DMA_CFG7_PL_0                           ((uint16_t)0x1000) /* Bit 0 */
#define DMA_CFG7_PL_1                           ((uint16_t)0x2000) /* Bit 1 */

#define DMA_CFG7_MEM2MEM                        ((uint16_t)0x4000) /* Memory to memory mode enable */

/*******************  Bit definition for DMA_CFG8 register  *******************/
#define DMA_CFG8_EN                             ((uint16_t)0x0001) /* Channel enable */
#define DMA_CFG8_TCIE                           ((uint16_t)0x0002) /* Transfer complete interrupt enable */
#define DMA_CFG8_HTIE                           ((uint16_t)0x0004) /* Half Transfer interrupt enable */
#define DMA_CFG8_TEIE                           ((uint16_t)0x0008) /* Transfer error interrupt enable */
#define DMA_CFG8_DIR                            ((uint16_t)0x0010) /* Data transfer direction */
#define DMA_CFG8_CIRC                           ((uint16_t)0x0020) /* Circular mode */
#define DMA_CFG8_PINC                           ((uint16_t)0x0040) /* Peripheral increment mode */
#define DMA_CFG8_MINC                           ((uint16_t)0x0080) /* Memory increment mode */

#define DMA_CFG8_PSIZE                          ((uint16_t)0x0300) /* PSIZE[1:0] bits (Peripheral size) */
#define DMA_CFG8_PSIZE_0                        ((uint16_t)0x0100) /* Bit 0 */
#define DMA_CFG8_PSIZE_1                        ((uint16_t)0x0200) /* Bit 1 */

#define DMA_CFG8_MSIZE                          ((uint16_t)0x0C00) /* MSIZE[1:0] bits (Memory size) */
#define DMA_CFG8_MSIZE_0                        ((uint16_t)0x0400) /* Bit 0 */
#define DMA_CFG8_MSIZE_1                        ((uint16_t)0x0800) /* Bit 1 */

#define DMA_CFG8_PL                             ((uint16_t)0x3000) /* PL[1:0] bits (Channel Priority level) */
#define DMA_CFG8_PL_0                           ((uint16_t)0x1000) /* Bit 0 */
#define DMA_CFG8_PL_1                           ((uint16_t)0x2000) /* Bit 1 */

#define DMA_CFG8_MEM2MEM                        ((uint16_t)0x4000) /* Memory to memory mode enable */

/******************  Bit definition for DMA_CNTR1 register  ******************/
#define DMA_CNTR1_NDT                           ((uint16_t)0xFFFF) /* Number of data to Transfer */

/******************  Bit definition for DMA_CNTR2 register  ******************/
#define DMA_CNTR2_NDT                           ((uint16_t)0xFFFF) /* Number of data to Transfer */

/******************  Bit definition for DMA_CNTR3 register  ******************/
#define DMA_CNTR3_NDT                           ((uint16_t)0xFFFF) /* Number of data to Transfer */

/******************  Bit definition for DMA_CNTR4 register  ******************/
#define DMA_CNTR4_NDT                           ((uint16_t)0xFFFF) /* Number of data to Transfer */

/******************  Bit definition for DMA_CNTR5 register  ******************/
#define DMA_CNTR5_NDT                           ((uint16_t)0xFFFF) /* Number of data to Transfer */

/******************  Bit definition for DMA_CNTR6 register  ******************/
#define DMA_CNTR6_NDT                           ((uint16_t)0xFFFF) /* Number of data to Transfer */

/******************  Bit definition for DMA_CNTR7 register  ******************/
#define DMA_CNTR7_NDT                           ((uint16_t)0xFFFF) /* Number of data to Transfer */

/******************  Bit definition for DMA_CNTR8 register  ******************/
#define DMA_CNTR8_NDT                           ((uint16_t)0xFFFF) /* Number of data to Transfer */

/******************  Bit definition for DMA_PADDR1 register  *******************/
#define DMA_PADDR1_PA                           ((uint32_t)0xFFFFFFFF) /* Peripheral Address */

/******************  Bit definition for DMA_PADDR2 register  *******************/
#define DMA_PADDR2_PA                           ((uint32_t)0xFFFFFFFF) /* Peripheral Address */

/******************  Bit definition for DMA_PADDR3 register  *******************/
#define DMA_PADDR3_PA                           ((uint32_t)0xFFFFFFFF) /* Peripheral Address */

/******************  Bit definition for DMA_PADDR4 register  *******************/
#define DMA_PADDR4_PA                           ((uint32_t)0xFFFFFFFF) /* Peripheral Address */

/******************  Bit definition for DMA_PADDR5 register  *******************/
#define DMA_PADDR5_PA                           ((uint32_t)0xFFFFFFFF) /* Peripheral Address */

/******************  Bit definition for DMA_PADDR6 register  *******************/
#define DMA_PADDR6_PA                           ((uint32_t)0xFFFFFFFF) /* Peripheral Address */

/******************  Bit definition for DMA_PADDR7 register  *******************/
#define DMA_PADDR7_PA                           ((uint32_t)0xFFFFFFFF) /* Peripheral Address */

/******************  Bit definition for DMA_PADDR8 register  *******************/
#define DMA_PADDR8_PA                           ((uint32_t)0xFFFFFFFF) /* Peripheral Address */

/******************  Bit definition for DMA_MADDR1 register  *******************/
#define DMA_MADDR1_MA                           ((uint32_t)0xFFFFFFFF) /* Memory Address */

/******************  Bit definition for DMA_MADDR2 register  *******************/
#define DMA_MADDR2_MA                           ((uint32_t)0xFFFFFFFF) /* Memory Address */

/******************  Bit definition for DMA_MADDR3 register  *******************/
#define DMA_MADDR3_MA                           ((uint32_t)0xFFFFFFFF) /* Memory Address */

/******************  Bit definition for DMA_MADDR4 register  *******************/
#define DMA_MADDR4_MA                           ((uint32_t)0xFFFFFFFF) /* Memory Address */

/******************  Bit definition for DMA_MADDR5 register  *******************/
#define DMA_MADDR5_MA                           ((uint32_t)0xFFFFFFFF) /* Memory Address */

/******************  Bit definition for DMA_MADDR6 register  *******************/
#define DMA_MADDR6_MA                           ((uint32_t)0xFFFFFFFF) /* Memory Address */

/******************  Bit definition for DMA_MADDR7 register  *******************/
#define DMA_MADDR7_MA                           ((uint32_t)0xFFFFFFFF) /* Memory Address */

/******************  Bit definition for DMA_MADDR8 register  *******************/
#define DMA_MADDR8_MA                           ((uint32_t)0xFFFFFFFF) /* Memory Address */

/******************************************************************************/
/*                    External Interrupt/Event Controller                     */
/******************************************************************************/

/*******************  Bit definition for EXTI_INTENR register  *******************/
#define EXTI_INTENR_MR0                         ((uint32_t)0x00000001) /* Interrupt Mask on line 0 */
#define EXTI_INTENR_MR1                         ((uint32_t)0x00000002) /* Interrupt Mask on line 1 */
#define EXTI_INTENR_MR2                         ((uint32_t)0x00000004) /* Interrupt Mask on line 2 */
#define EXTI_INTENR_MR3                         ((uint32_t)0x00000008) /* Interrupt Mask on line 3 */
#define EXTI_INTENR_MR4                         ((uint32_t)0x00000010) /* Interrupt Mask on line 4 */
#define EXTI_INTENR_MR5                         ((uint32_t)0x00000020) /* Interrupt Mask on line 5 */
#define EXTI_INTENR_MR6                         ((uint32_t)0x00000040) /* Interrupt Mask on line 6 */
#define EXTI_INTENR_MR7                         ((uint32_t)0x00000080) /* Interrupt Mask on line 7 */
#define EXTI_INTENR_MR8                         ((uint32_t)0x00000100) /* Interrupt Mask on line 8 */
#define EXTI_INTENR_MR9                         ((uint32_t)0x00000200) /* Interrupt Mask on line 9 */
#define EXTI_INTENR_MR10                        ((uint32_t)0x00000400) /* Interrupt Mask on line 10 */
#define EXTI_INTENR_MR11                        ((uint32_t)0x00000800) /* Interrupt Mask on line 11 */
#define EXTI_INTENR_MR12                        ((uint32_t)0x00001000) /* Interrupt Mask on line 12 */
#define EXTI_INTENR_MR13                        ((uint32_t)0x00002000) /* Interrupt Mask on line 13 */
#define EXTI_INTENR_MR14                        ((uint32_t)0x00004000) /* Interrupt Mask on line 14 */
#define EXTI_INTENR_MR15                        ((uint32_t)0x00008000) /* Interrupt Mask on line 15 */
#define EXTI_INTENR_MR16                        ((uint32_t)0x00010000) /* Interrupt Mask on line 16 */
#define EXTI_INTENR_MR17                        ((uint32_t)0x00020000) /* Interrupt Mask on line 17 */
#define EXTI_INTENR_MR18                        ((uint32_t)0x00040000) /* Interrupt Mask on line 18 */
#define EXTI_INTENR_MR19                        ((uint32_t)0x00080000) /* Interrupt Mask on line 19 */
#define EXTI_INTENR_MR20                        ((uint32_t)0x00100000) /* Interrupt Mask on line 20 */
#define EXTI_INTENR_MR21                        ((uint32_t)0x00200000) /* Interrupt Mask on line 21 */
#define EXTI_INTENR_MR22                        ((uint32_t)0x00400000) /* Interrupt Mask on line 22 */
#define EXTI_INTENR_MR23                        ((uint32_t)0x00800000) /* Interrupt Mask on line 23 */
#define EXTI_INTENR_MR24                        ((uint32_t)0x01000000) /* Interrupt Mask on line 24 */
#define EXTI_INTENR_MR25                        ((uint32_t)0x02000000) /* Interrupt Mask on line 25 */
#define EXTI_INTENR_MR26                        ((uint32_t)0x04000000) /* Interrupt Mask on line 26 */
#define EXTI_INTENR_MR27                        ((uint32_t)0x08000000) /* Interrupt Mask on line 27 */
#define EXTI_INTENR_MR28                        ((uint32_t)0x10000000) /* Interrupt Mask on line 28 */
#define EXTI_INTENR_MR29                        ((uint32_t)0x20000000) /* Interrupt Mask on line 29 */

/*******************  Bit definition for EXTI_EVENR register  *******************/
#define EXTI_EVENR_MR0                          ((uint32_t)0x00000001) /* Event Mask on line 0 */
#define EXTI_EVENR_MR1                          ((uint32_t)0x00000002) /* Event Mask on line 1 */
#define EXTI_EVENR_MR2                          ((uint32_t)0x00000004) /* Event Mask on line 2 */
#define EXTI_EVENR_MR3                          ((uint32_t)0x00000008) /* Event Mask on line 3 */
#define EXTI_EVENR_MR4                          ((uint32_t)0x00000010) /* Event Mask on line 4 */
#define EXTI_EVENR_MR5                          ((uint32_t)0x00000020) /* Event Mask on line 5 */
#define EXTI_EVENR_MR6                          ((uint32_t)0x00000040) /* Event Mask on line 6 */
#define EXTI_EVENR_MR7                          ((uint32_t)0x00000080) /* Event Mask on line 7 */
#define EXTI_EVENR_MR8                          ((uint32_t)0x00000100) /* Event Mask on line 8 */
#define EXTI_EVENR_MR9                          ((uint32_t)0x00000200) /* Event Mask on line 9 */
#define EXTI_EVENR_MR10                         ((uint32_t)0x00000400) /* Event Mask on line 10 */
#define EXTI_EVENR_MR11                         ((uint32_t)0x00000800) /* Event Mask on line 11 */
#define EXTI_EVENR_MR12                         ((uint32_t)0x00001000) /* Event Mask on line 12 */
#define EXTI_EVENR_MR13                         ((uint32_t)0x00002000) /* Event Mask on line 13 */
#define EXTI_EVENR_MR14                         ((uint32_t)0x00004000) /* Event Mask on line 14 */
#define EXTI_EVENR_MR15                         ((uint32_t)0x00008000) /* Event Mask on line 15 */
#define EXTI_EVENR_MR16                         ((uint32_t)0x00010000) /* Event Mask on line 16 */
#define EXTI_EVENR_MR17                         ((uint32_t)0x00020000) /* Event Mask on line 17 */
#define EXTI_EVENR_MR18                         ((uint32_t)0x00040000) /* Event Mask on line 18 */
#define EXTI_EVENR_MR19                         ((uint32_t)0x00080000) /* Event Mask on line 19 */
#define EXTI_EVENR_MR20                         ((uint32_t)0x00100000) /* Event Mask on line 20 */
#define EXTI_EVENR_MR21                         ((uint32_t)0x00200000) /* Event Mask on line 21 */
#define EXTI_EVENR_MR22                         ((uint32_t)0x00400000) /* Event Mask on line 22 */
#define EXTI_EVENR_MR23                         ((uint32_t)0x00800000) /* Event Mask on line 23 */
#define EXTI_EVENR_MR24                         ((uint32_t)0x01000000) /* Event Mask on line 24 */
#define EXTI_EVENR_MR25                         ((uint32_t)0x02000000) /* Event Mask on line 25 */
#define EXTI_EVENR_MR26                         ((uint32_t)0x04000000) /* Event Mask on line 26 */
#define EXTI_EVENR_MR27                         ((uint32_t)0x08000000) /* Event Mask on line 27 */
#define EXTI_EVENR_MR28                         ((uint32_t)0x10000000) /* Event Mask on line 28 */
#define EXTI_EVENR_MR29                         ((uint32_t)0x20000000) /* Event Mask on line 29 */

/******************  Bit definition for EXTI_RTENR register  *******************/
#define EXTI_RTENR_TR0                          ((uint32_t)0x00000001) /* Rising trigger event configuration bit of line 0 */
#define EXTI_RTENR_TR1                          ((uint32_t)0x00000002) /* Rising trigger event configuration bit of line 1 */
#define EXTI_RTENR_TR2                          ((uint32_t)0x00000004) /* Rising trigger event configuration bit of line 2 */
#define EXTI_RTENR_TR3                          ((uint32_t)0x00000008) /* Rising trigger event configuration bit of line 3 */
#define EXTI_RTENR_TR4                          ((uint32_t)0x00000010) /* Rising trigger event configuration bit of line 4 */
#define EXTI_RTENR_TR5                          ((uint32_t)0x00000020) /* Rising trigger event configuration bit of line 5 */
#define EXTI_RTENR_TR6                          ((uint32_t)0x00000040) /* Rising trigger event configuration bit of line 6 */
#define EXTI_RTENR_TR7                          ((uint32_t)0x00000080) /* Rising trigger event configuration bit of line 7 */
#define EXTI_RTENR_TR8                          ((uint32_t)0x00000100) /* Rising trigger event configuration bit of line 8 */
#define EXTI_RTENR_TR9                          ((uint32_t)0x00000200) /* Rising trigger event configuration bit of line 9 */
#define EXTI_RTENR_TR10                         ((uint32_t)0x00000400) /* Rising trigger event configuration bit of line 10 */
#define EXTI_RTENR_TR11                         ((uint32_t)0x00000800) /* Rising trigger event configuration bit of line 11 */
#define EXTI_RTENR_TR12                         ((uint32_t)0x00001000) /* Rising trigger event configuration bit of line 12 */
#define EXTI_RTENR_TR13                         ((uint32_t)0x00002000) /* Rising trigger event configuration bit of line 13 */
#define EXTI_RTENR_TR14                         ((uint32_t)0x00004000) /* Rising trigger event configuration bit of line 14 */
#define EXTI_RTENR_TR15                         ((uint32_t)0x00008000) /* Rising trigger event configuration bit of line 15 */
#define EXTI_RTENR_TR16                         ((uint32_t)0x00010000) /* Rising trigger event configuration bit of line 16 */
#define EXTI_RTENR_TR17                         ((uint32_t)0x00020000) /* Rising trigger event configuration bit of line 17 */
#define EXTI_RTENR_TR18                         ((uint32_t)0x00040000) /* Rising trigger event configuration bit of line 18 */
#define EXTI_RTENR_TR19                         ((uint32_t)0x00080000) /* Rising trigger event configuration bit of line 19 */
#define EXTI_RTENR_TR20                         ((uint32_t)0x00100000) /* Rising trigger event configuration bit of line 20 */
#define EXTI_RTENR_TR21                         ((uint32_t)0x00200000) /* Rising trigger event configuration bit of line 21 */
#define EXTI_RTENR_TR22                         ((uint32_t)0x00400000) /* Rising trigger event configuration bit of line 22 */
#define EXTI_RTENR_TR23                         ((uint32_t)0x00800000) /* Rising trigger event configuration bit of line 23 */
#define EXTI_RTENR_TR24                         ((uint32_t)0x01000000) /* Rising trigger event configuration bit of line 24 */
#define EXTI_RTENR_TR25                         ((uint32_t)0x02000000) /* Rising trigger event configuration bit of line 25 */
#define EXTI_RTENR_TR26                         ((uint32_t)0x04000000) /* Rising trigger event configuration bit of line 26 */
#define EXTI_RTENR_TR27                         ((uint32_t)0x08000000) /* Rising trigger event configuration bit of line 27 */
#define EXTI_RTENR_TR28                         ((uint32_t)0x10000000) /* Rising trigger event configuration bit of line 28 */
#define EXTI_RTENR_TR29                         ((uint32_t)0x20000000) /* Rising trigger event configuration bit of line 29 */

/******************  Bit definition for EXTI_FTENR register  *******************/
#define EXTI_FTENR_TR0                          ((uint32_t)0x00000001) /* Falling trigger event configuration bit of line 0 */
#define EXTI_FTENR_TR1                          ((uint32_t)0x00000002) /* Falling trigger event configuration bit of line 1 */
#define EXTI_FTENR_TR2                          ((uint32_t)0x00000004) /* Falling trigger event configuration bit of line 2 */
#define EXTI_FTENR_TR3                          ((uint32_t)0x00000008) /* Falling trigger event configuration bit of line 3 */
#define EXTI_FTENR_TR4                          ((uint32_t)0x00000010) /* Falling trigger event configuration bit of line 4 */
#define EXTI_FTENR_TR5                          ((uint32_t)0x00000020) /* Falling trigger event configuration bit of line 5 */
#define EXTI_FTENR_TR6                          ((uint32_t)0x00000040) /* Falling trigger event configuration bit of line 6 */
#define EXTI_FTENR_TR7                          ((uint32_t)0x00000080) /* Falling trigger event configuration bit of line 7 */
#define EXTI_FTENR_TR8                          ((uint32_t)0x00000100) /* Falling trigger event configuration bit of line 8 */
#define EXTI_FTENR_TR9                          ((uint32_t)0x00000200) /* Falling trigger event configuration bit of line 9 */
#define EXTI_FTENR_TR10                         ((uint32_t)0x00000400) /* Falling trigger event configuration bit of line 10 */
#define EXTI_FTENR_TR11                         ((uint32_t)0x00000800) /* Falling trigger event configuration bit of line 11 */
#define EXTI_FTENR_TR12                         ((uint32_t)0x00001000) /* Falling trigger event configuration bit of line 12 */
#define EXTI_FTENR_TR13                         ((uint32_t)0x00002000) /* Falling trigger event configuration bit of line 13 */
#define EXTI_FTENR_TR14                         ((uint32_t)0x00004000) /* Falling trigger event configuration bit of line 14 */
#define EXTI_FTENR_TR15                         ((uint32_t)0x00008000) /* Falling trigger event configuration bit of line 15 */
#define EXTI_FTENR_TR16                         ((uint32_t)0x00010000) /* Falling trigger event configuration bit of line 16 */
#define EXTI_FTENR_TR17                         ((uint32_t)0x00020000) /* Falling trigger event configuration bit of line 17 */
#define EXTI_FTENR_TR18                         ((uint32_t)0x00040000) /* Falling trigger event configuration bit of line 18 */
#define EXTI_FTENR_TR19                         ((uint32_t)0x00080000) /* Falling trigger event configuration bit of line 19 */
#define EXTI_FTENR_TR20                         ((uint32_t)0x00100000) /* Falling trigger event configuration bit of line 20 */
#define EXTI_FTENR_TR21                         ((uint32_t)0x00200000) /* Falling trigger event configuration bit of line 21 */
#define EXTI_FTENR_TR22                         ((uint32_t)0x00400000) /* Falling trigger event configuration bit of line 22 */
#define EXTI_FTENR_TR23                         ((uint32_t)0x00800000) /* Falling trigger event configuration bit of line 23 */
#define EXTI_FTENR_TR24                         ((uint32_t)0x01000000) /* Falling trigger event configuration bit of line 24 */
#define EXTI_FTENR_TR25                         ((uint32_t)0x02000000) /* Falling trigger event configuration bit of line 25 */
#define EXTI_FTENR_TR26                         ((uint32_t)0x04000000) /* Falling trigger event configuration bit of line 26 */
#define EXTI_FTENR_TR27                         ((uint32_t)0x08000000) /* Falling trigger event configuration bit of line 27 */
#define EXTI_FTENR_TR28                         ((uint32_t)0x10000000) /* Falling trigger event configuration bit of line 28 */
#define EXTI_FTENR_TR29                         ((uint32_t)0x20000000) /* Falling trigger event configuration bit of line 29 */

/******************  Bit definition for EXTI_SWIEVR register  ******************/
#define EXTI_SWIEVR_SWIEVR0                     ((uint32_t)0x00000001) /* Software Interrupt on line 0 */
#define EXTI_SWIEVR_SWIEVR1                     ((uint32_t)0x00000002) /* Software Interrupt on line 1 */
#define EXTI_SWIEVR_SWIEVR2                     ((uint32_t)0x00000004) /* Software Interrupt on line 2 */
#define EXTI_SWIEVR_SWIEVR3                     ((uint32_t)0x00000008) /* Software Interrupt on line 3 */
#define EXTI_SWIEVR_SWIEVR4                     ((uint32_t)0x00000010) /* Software Interrupt on line 4 */
#define EXTI_SWIEVR_SWIEVR5                     ((uint32_t)0x00000020) /* Software Interrupt on line 5 */
#define EXTI_SWIEVR_SWIEVR6                     ((uint32_t)0x00000040) /* Software Interrupt on line 6 */
#define EXTI_SWIEVR_SWIEVR7                     ((uint32_t)0x00000080) /* Software Interrupt on line 7 */
#define EXTI_SWIEVR_SWIEVR8                     ((uint32_t)0x00000100) /* Software Interrupt on line 8 */
#define EXTI_SWIEVR_SWIEVR9                     ((uint32_t)0x00000200) /* Software Interrupt on line 9 */
#define EXTI_SWIEVR_SWIEVR10                    ((uint32_t)0x00000400) /* Software Interrupt on line 10 */
#define EXTI_SWIEVR_SWIEVR11                    ((uint32_t)0x00000800) /* Software Interrupt on line 11 */
#define EXTI_SWIEVR_SWIEVR12                    ((uint32_t)0x00001000) /* Software Interrupt on line 12 */
#define EXTI_SWIEVR_SWIEVR13                    ((uint32_t)0x00002000) /* Software Interrupt on line 13 */
#define EXTI_SWIEVR_SWIEVR14                    ((uint32_t)0x00004000) /* Software Interrupt on line 14 */
#define EXTI_SWIEVR_SWIEVR15                    ((uint32_t)0x00008000) /* Software Interrupt on line 15 */
#define EXTI_SWIEVR_SWIEVR16                    ((uint32_t)0x00010000) /* Software Interrupt on line 16 */
#define EXTI_SWIEVR_SWIEVR17                    ((uint32_t)0x00020000) /* Software Interrupt on line 17 */
#define EXTI_SWIEVR_SWIEVR18                    ((uint32_t)0x00040000) /* Software Interrupt on line 18 */
#define EXTI_SWIEVR_SWIEVR19                    ((uint32_t)0x00080000) /* Software Interrupt on line 19 */
#define EXTI_SWIEVR_SWIEVR20                    ((uint32_t)0x00100000) /* Software Interrupt on line 20 */
#define EXTI_SWIEVR_SWIEVR21                    ((uint32_t)0x00200000) /* Software Interrupt on line 21 */
#define EXTI_SWIEVR_SWIEVR22                    ((uint32_t)0x00400000) /* Software Interrupt on line 22 */
#define EXTI_SWIEVR_SWIEVR23                    ((uint32_t)0x00800000) /* Software Interrupt on line 23 */
#define EXTI_SWIEVR_SWIEVR24                    ((uint32_t)0x01000000) /* Software Interrupt on line 24 */
#define EXTI_SWIEVR_SWIEVR25                    ((uint32_t)0x02000000) /* Software Interrupt on line 25 */
#define EXTI_SWIEVR_SWIEVR26                    ((uint32_t)0x04000000) /* Software Interrupt on line 26 */
#define EXTI_SWIEVR_SWIEVR27                    ((uint32_t)0x08000000) /* Software Interrupt on line 27 */
#define EXTI_SWIEVR_SWIEVR28                    ((uint32_t)0x10000000) /* Software Interrupt on line 28 */
#define EXTI_SWIEVR_SWIEVR29                    ((uint32_t)0x20000000) /* Software Interrupt on line 29 */

/*******************  Bit definition for EXTI_INTFR register  ********************/
#define EXTI_INTF_INTF0                         ((uint32_t)0x00000001) /* Pending bit for line 0 */
#define EXTI_INTF_INTF1                         ((uint32_t)0x00000002) /* Pending bit for line 1 */
#define EXTI_INTF_INTF2                         ((uint32_t)0x00000004) /* Pending bit for line 2 */
#define EXTI_INTF_INTF3                         ((uint32_t)0x00000008) /* Pending bit for line 3 */
#define EXTI_INTF_INTF4                         ((uint32_t)0x00000010) /* Pending bit for line 4 */
#define EXTI_INTF_INTF5                         ((uint32_t)0x00000020) /* Pending bit for line 5 */
#define EXTI_INTF_INTF6                         ((uint32_t)0x00000040) /* Pending bit for line 6 */
#define EXTI_INTF_INTF7                         ((uint32_t)0x00000080) /* Pending bit for line 7 */
#define EXTI_INTF_INTF8                         ((uint32_t)0x00000100) /* Pending bit for line 8 */
#define EXTI_INTF_INTF9                         ((uint32_t)0x00000200) /* Pending bit for line 9 */
#define EXTI_INTF_INTF10                        ((uint32_t)0x00000400) /* Pending bit for line 10 */
#define EXTI_INTF_INTF11                        ((uint32_t)0x00000800) /* Pending bit for line 11 */
#define EXTI_INTF_INTF12                        ((uint32_t)0x00001000) /* Pending bit for line 12 */
#define EXTI_INTF_INTF13                        ((uint32_t)0x00002000) /* Pending bit for line 13 */
#define EXTI_INTF_INTF14                        ((uint32_t)0x00004000) /* Pending bit for line 14 */
#define EXTI_INTF_INTF15                        ((uint32_t)0x00008000) /* Pending bit for line 15 */
#define EXTI_INTF_INTF16                        ((uint32_t)0x00010000) /* Pending bit for line 16 */
#define EXTI_INTF_INTF17                        ((uint32_t)0x00020000) /* Pending bit for line 17 */
#define EXTI_INTF_INTF18                        ((uint32_t)0x00040000) /* Pending bit for line 18 */
#define EXTI_INTF_INTF19                        ((uint32_t)0x00080000) /* Pending bit for line 19 */
#define EXTI_INTF_INTF20                        ((uint32_t)0x00100000) /* Pending bit for line 20 */
#define EXTI_INTF_INTF21                        ((uint32_t)0x00200000) /* Pending bit for line 21 */
#define EXTI_INTF_INTF22                        ((uint32_t)0x00400000) /* Pending bit for line 22 */
#define EXTI_INTF_INTF23                        ((uint32_t)0x00800000) /* Pending bit for line 23 */
#define EXTI_INTF_INTF24                        ((uint32_t)0x01000000) /* Pending bit for line 24 */
#define EXTI_INTF_INTF25                        ((uint32_t)0x02000000) /* Pending bit for line 25 */
#define EXTI_INTF_INTF26                        ((uint32_t)0x04000000) /* Pending bit for line 26 */
#define EXTI_INTF_INTF27                        ((uint32_t)0x08000000) /* Pending bit for line 27 */
#define EXTI_INTF_INTF28                        ((uint32_t)0x10000000) /* Pending bit for line 28 */
#define EXTI_INTF_INTF29                        ((uint32_t)0x20000000) /* Pending bit for line 29 */

/******************************************************************************/
/*                      FLASH and Option Bytes Registers                      */
/******************************************************************************/

/*******************  Bit definition for FLASH_ACTLR register  ******************/
#define FLASH_ACTLR_LATENCY                     ((uint8_t)0x03) /* LATENCY[2:0] bits (Latency) */
#define FLASH_ACTLR_LATENCY_0                   ((uint8_t)0x00) /* Bit 0 */
#define FLASH_ACTLR_LATENCY_1                   ((uint8_t)0x01) /* Bit 0 */
#define FLASH_ACTLR_LATENCY_2                   ((uint8_t)0x02) /* Bit 1 */

/******************  Bit definition for FLASH_KEYR register  ******************/
#define FLASH_KEYR_FKEYR                        ((uint32_t)0xFFFFFFFF) /* FPEC Key */

/*****************  Bit definition for FLASH_OBKEYR register  ****************/
#define FLASH_OBKEYR_OBKEYR                     ((uint32_t)0xFFFFFFFF) /* Option Byte Key */

/******************  Bit definition for FLASH_STATR register  *******************/
#define FLASH_STATR_BSY                         ((uint8_t)0x01) /* Busy */

#define FLASH_STATR_WRPRTERR                    ((uint8_t)0x10) /* Write Protection Error */
#define FLASH_STATR_EOP                         ((uint8_t)0x20) /* End of operation */
#define FLASH_STATR_FWAKE_FLAG                  ((uint8_t)0x40) /* Flag of wake */
#define FLASH_STATR_TURBO                       ((uint8_t)0x80) /* The state of TURBO Enable */
#define FLASH_STATR_BOOT_AVA                    ((uint16_t)0x1000) /* The state of Init Config */
#define FLASH_STATR_BOOT_STATUS                 ((uint16_t)0x2000) /* The source of Execute Program */
#define FLASH_STATR_BOOT_MODE                   ((uint16_t)0x4000) /* The switch of user section or boot section*/
#define FLASH_STATR_BOOT_LOCK                   ((uint16_t)0x8000) /* Lock boot area*/

/*******************  Bit definition for FLASH_CTLR register  *******************/
#define FLASH_CTLR_PER                          ((uint32_t)0x00000002) /* Sector Erase 4K */
#define FLASH_CTLR_MER                          ((uint32_t)0x00000004) /* Mass Erase */
#define FLASH_CTLR_OPTPG                        ((uint32_t)0x00000010) /* Option Byte Programming */
#define FLASH_CTLR_OPTER                        ((uint32_t)0x00000020) /* Option Byte Erase */
#define FLASH_CTLR_STRT                         ((uint32_t)0x00000040) /* Start */
#define FLASH_CTLR_LOCK                         ((uint32_t)0x00000080) /* Lock */
#define FLASH_CTLR_OBWRE                        ((uint32_t)0x00000200) /* Option Bytes Write Enable */
#define FLASH_CTLR_ERRIE                        ((uint32_t)0x00000400) /* Error Interrupt Enable */
#define FLASH_CTLR_EOPIE                        ((uint32_t)0x00001000) /* End of operation interrupt enable */
#define FLASH_CTLR_FWAKEIE                      ((uint32_t)0x00002000) /* Wake inter Enable */
#define FLASH_CTLR_FLOCK                        ((uint32_t)0x00008000) /* Fast Lock */
#define FLASH_CTLR_FTPG                         ((uint32_t)0x00010000) /* Fast Program */
#define FLASH_CTLR_FTER                         ((uint32_t)0x00020000) /* Fast Erase */
#define FLASH_CTLR_BUFLOAD                      ((uint32_t)0x00040000) /* BUF Load */
#define FLASH_CTLR_BUFRST                       ((uint32_t)0x00080000) /* BUF Reset */
#define FLASH_CTLR_BER32                        ((uint32_t)0x00800000) /* Block Erase 32K */

/*******************  Bit definition for FLASH_ADDR register  *******************/
#define FLASH_ADDR_FAR                          ((uint32_t)0xFFFFFFFF) /* Flash Address */

/******************  Bit definition for FLASH_OBR register  *******************/
#define FLASH_OBR_OPTERR                        ((uint16_t)0x0001) /* Option Byte Error */
#define FLASH_OBR_RDPRT                         ((uint16_t)0x0002) /* Read protection */

#define FLASH_OBR_USER                          ((uint16_t)0x007C) /* User Option Bytes */
#define FLASH_OBR_WDG_SW                        ((uint16_t)0x0004) /* WDG_SW */
#define FLASH_OBR_nRST_STOP                     ((uint16_t)0x0008) /* nRST_STOP */
#define FLASH_OBR_nRST_STDBY                    ((uint16_t)0x0010) /* nRST_STDBY */
#define FLASH_OBR_CFGRSTT                       ((uint16_t)0x0060) /* Config Reset delay time */

#define FLASH_OBR_FIX_11                        ((uint16_t)0x0300) /* fix 11 */
#define FLASH_OBR_DATA0                         ((uint32_t)0x0003FC00) /* Data byte0 */
#define FLASH_OBR_DATA1                         ((uint32_t)0x03FC0000) /* Data byte1 */

/******************  Bit definition for FLASH_WPR register  ******************/
#define FLASH_WPR_WRP                           ((uint32_t)0xFFFFFFFF) /* Write Protect */

/******************  Bit definition for FLASH_MODEKEYR register  ******************/
#define FLASH_MODEKEYR_MODEKEYR                 ((uint32_t)0xFFFFFFFF) /* Open fast program /erase */
#define FLASH_MODEKEYR_MODEKEYR1                ((uint32_t)0x45670123) 
#define FLASH_MODEKEYR_MODEKEYR2                ((uint32_t)0xCDEF89AB) 

/******************  Bit definition for BOOT_MODEKEYP register  ******************/
#define BOOT_MODEKEYP_MODEKEYR                  ((uint32_t)0xFFFFFFFF) /* Open Boot section */
#define BOOT_MODEKEYP_MODEKEYR1                 ((uint32_t)0x45670123)
#define BOOT_MODEKEYP_MODEKEYR2                 ((uint32_t)0xCDEF89AB)

/******************  Bit definition for FLASH_RDPR register  *******************/
#define FLASH_RDPR_RDPR                         ((uint32_t)0x000000FF) /* Read protection option byte */
#define FLASH_RDPR_nRDPR                        ((uint32_t)0x0000FF00) /* Read protection complemented option byte */

/******************  Bit definition for FLASH_USER register  ******************/
#define FLASH_USER_USER                         ((uint32_t)0x00FF0000) /* User option byte */
#define FLASH_USER_nUSER                        ((uint32_t)0xFF000000) /* User complemented option byte */

/******************  Bit definition for FLASH_Data0 register  *****************/
#define FLASH_Data0_Data0                       ((uint32_t)0x000000FF) /* User data storage option byte */
#define FLASH_Data0_nData0                      ((uint32_t)0x0000FF00) /* User data storage complemented option byte */

/******************  Bit definition for FLASH_Data1 register  *****************/
#define FLASH_Data1_Data1                       ((uint32_t)0x00FF0000) /* User data storage option byte */
#define FLASH_Data1_nData1                      ((uint32_t)0xFF000000) /* User data storage complemented option byte */

/******************  Bit definition for FLASH_WRPR0 register  ******************/
#define FLASH_WRPR0_WRPR0                       ((uint32_t)0x000000FF) /* Flash memory write protection option bytes */
#define FLASH_WRPR0_nWRPR0                      ((uint32_t)0x0000FF00) /* Flash memory write protection complemented option bytes */

/******************  Bit definition for FLASH_WRPR1 register  ******************/
#define FLASH_WRPR1_WRPR1                       ((uint32_t)0x00FF0000) /* Flash memory write protection option bytes */
#define FLASH_WRPR1_nWRPR1                      ((uint32_t)0xFF000000) /* Flash memory write protection complemented option bytes */

/******************  Bit definition for FLASH_WRPR2 register  ******************/
#define FLASH_WRPR2_WRPR2                       ((uint32_t)0x000000FF) /* Flash memory write protection option bytes */
#define FLASH_WRPR2_nWRPR2                      ((uint32_t)0x0000FF00) /* Flash memory write protection complemented option bytes */

/******************  Bit definition for FLASH_WRPR3 register  ******************/
#define FLASH_WRPR3_WRPR3                       ((uint32_t)0x00FF0000) /* Flash memory write protection option bytes */
#define FLASH_WRPR3_nWRPR3                      ((uint32_t)0xFF000000) /* Flash memory write protection complemented option bytes */

/******************************************************************************/
/*                General Purpose and Alternate Function I/O                  */
/******************************************************************************/

/*******************  Bit definition for GPIO_CFGLR register  *******************/
#define GPIO_CFGLR_MODE                         ((uint32_t)0x33333333) /* Port x mode bits */

#define GPIO_CFGLR_MODE0                        ((uint32_t)0x00000003) /* MODE0[1:0] bits (Port x mode bits, pin 0) */
#define GPIO_CFGLR_MODE0_0                      ((uint32_t)0x00000001) /* Bit 0 */
#define GPIO_CFGLR_MODE0_1                      ((uint32_t)0x00000002) /* Bit 1 */

#define GPIO_CFGLR_MODE1                        ((uint32_t)0x00000030) /* MODE1[1:0] bits (Port x mode bits, pin 1) */
#define GPIO_CFGLR_MODE1_0                      ((uint32_t)0x00000010) /* Bit 0 */
#define GPIO_CFGLR_MODE1_1                      ((uint32_t)0x00000020) /* Bit 1 */

#define GPIO_CFGLR_MODE2                        ((uint32_t)0x00000300) /* MODE2[1:0] bits (Port x mode bits, pin 2) */
#define GPIO_CFGLR_MODE2_0                      ((uint32_t)0x00000100) /* Bit 0 */
#define GPIO_CFGLR_MODE2_1                      ((uint32_t)0x00000200) /* Bit 1 */

#define GPIO_CFGLR_MODE3                        ((uint32_t)0x00003000) /* MODE3[1:0] bits (Port x mode bits, pin 3) */
#define GPIO_CFGLR_MODE3_0                      ((uint32_t)0x00001000) /* Bit 0 */
#define GPIO_CFGLR_MODE3_1                      ((uint32_t)0x00002000) /* Bit 1 */

#define GPIO_CFGLR_MODE4                        ((uint32_t)0x00030000) /* MODE4[1:0] bits (Port x mode bits, pin 4) */
#define GPIO_CFGLR_MODE4_0                      ((uint32_t)0x00010000) /* Bit 0 */
#define GPIO_CFGLR_MODE4_1                      ((uint32_t)0x00020000) /* Bit 1 */

#define GPIO_CFGLR_MODE5                        ((uint32_t)0x00300000) /* MODE5[1:0] bits (Port x mode bits, pin 5) */
#define GPIO_CFGLR_MODE5_0                      ((uint32_t)0x00100000) /* Bit 0 */
#define GPIO_CFGLR_MODE5_1                      ((uint32_t)0x00200000) /* Bit 1 */

#define GPIO_CFGLR_MODE6                        ((uint32_t)0x03000000) /* MODE6[1:0] bits (Port x mode bits, pin 6) */
#define GPIO_CFGLR_MODE6_0                      ((uint32_t)0x01000000) /* Bit 0 */
#define GPIO_CFGLR_MODE6_1                      ((uint32_t)0x02000000) /* Bit 1 */

#define GPIO_CFGLR_MODE7                        ((uint32_t)0x30000000) /* MODE7[1:0] bits (Port x mode bits, pin 7) */
#define GPIO_CFGLR_MODE7_0                      ((uint32_t)0x10000000) /* Bit 0 */
#define GPIO_CFGLR_MODE7_1                      ((uint32_t)0x20000000) /* Bit 1 */

#define GPIO_CFGLR_CNF                          ((uint32_t)0xCCCCCCCC) /* Port x configuration bits */

#define GPIO_CFGLR_CNF0                         ((uint32_t)0x0000000C) /* CNF0[1:0] bits (Port x configuration bits, pin 0) */
#define GPIO_CFGLR_CNF0_0                       ((uint32_t)0x00000004) /* Bit 0 */
#define GPIO_CFGLR_CNF0_1                       ((uint32_t)0x00000008) /* Bit 1 */

#define GPIO_CFGLR_CNF1                         ((uint32_t)0x000000C0) /* CNF1[1:0] bits (Port x configuration bits, pin 1) */
#define GPIO_CFGLR_CNF1_0                       ((uint32_t)0x00000040) /* Bit 0 */
#define GPIO_CFGLR_CNF1_1                       ((uint32_t)0x00000080) /* Bit 1 */

#define GPIO_CFGLR_CNF2                         ((uint32_t)0x00000C00) /* CNF2[1:0] bits (Port x configuration bits, pin 2) */
#define GPIO_CFGLR_CNF2_0                       ((uint32_t)0x00000400) /* Bit 0 */
#define GPIO_CFGLR_CNF2_1                       ((uint32_t)0x00000800) /* Bit 1 */

#define GPIO_CFGLR_CNF3                         ((uint32_t)0x0000C000) /* CNF3[1:0] bits (Port x configuration bits, pin 3) */
#define GPIO_CFGLR_CNF3_0                       ((uint32_t)0x00004000) /* Bit 0 */
#define GPIO_CFGLR_CNF3_1                       ((uint32_t)0x00008000) /* Bit 1 */

#define GPIO_CFGLR_CNF4                         ((uint32_t)0x000C0000) /* CNF4[1:0] bits (Port x configuration bits, pin 4) */
#define GPIO_CFGLR_CNF4_0                       ((uint32_t)0x00040000) /* Bit 0 */
#define GPIO_CFGLR_CNF4_1                       ((uint32_t)0x00080000) /* Bit 1 */

#define GPIO_CFGLR_CNF5                         ((uint32_t)0x00C00000) /* CNF5[1:0] bits (Port x configuration bits, pin 5) */
#define GPIO_CFGLR_CNF5_0                       ((uint32_t)0x00400000) /* Bit 0 */
#define GPIO_CFGLR_CNF5_1                       ((uint32_t)0x00800000) /* Bit 1 */

#define GPIO_CFGLR_CNF6                         ((uint32_t)0x0C000000) /* CNF6[1:0] bits (Port x configuration bits, pin 6) */
#define GPIO_CFGLR_CNF6_0                       ((uint32_t)0x04000000) /* Bit 0 */
#define GPIO_CFGLR_CNF6_1                       ((uint32_t)0x08000000) /* Bit 1 */

#define GPIO_CFGLR_CNF7                         ((uint32_t)0xC0000000) /* CNF7[1:0] bits (Port x configuration bits, pin 7) */
#define GPIO_CFGLR_CNF7_0                       ((uint32_t)0x40000000) /* Bit 0 */
#define GPIO_CFGLR_CNF7_1                       ((uint32_t)0x80000000) /* Bit 1 */

/*******************  Bit definition for GPIO_CFGHR register  *******************/
#define GPIO_CFGHR_MODE                         ((uint32_t)0x33333333) /* Port x mode bits */

#define GPIO_CFGHR_MODE8                        ((uint32_t)0x00000003) /* MODE8[1:0] bits (Port x mode bits, pin 8) */
#define GPIO_CFGHR_MODE8_0                      ((uint32_t)0x00000001) /* Bit 0 */
#define GPIO_CFGHR_MODE8_1                      ((uint32_t)0x00000002) /* Bit 1 */

#define GPIO_CFGHR_MODE9                        ((uint32_t)0x00000030) /* MODE9[1:0] bits (Port x mode bits, pin 9) */
#define GPIO_CFGHR_MODE9_0                      ((uint32_t)0x00000010) /* Bit 0 */
#define GPIO_CFGHR_MODE9_1                      ((uint32_t)0x00000020) /* Bit 1 */

#define GPIO_CFGHR_MODE10                       ((uint32_t)0x00000300) /* MODE10[1:0] bits (Port x mode bits, pin 10) */
#define GPIO_CFGHR_MODE10_0                     ((uint32_t)0x00000100) /* Bit 0 */
#define GPIO_CFGHR_MODE10_1                     ((uint32_t)0x00000200) /* Bit 1 */

#define GPIO_CFGHR_MODE11                       ((uint32_t)0x00003000) /* MODE11[1:0] bits (Port x mode bits, pin 11) */
#define GPIO_CFGHR_MODE11_0                     ((uint32_t)0x00001000) /* Bit 0 */
#define GPIO_CFGHR_MODE11_1                     ((uint32_t)0x00002000) /* Bit 1 */

#define GPIO_CFGHR_MODE12                       ((uint32_t)0x00030000) /* MODE12[1:0] bits (Port x mode bits, pin 12) */
#define GPIO_CFGHR_MODE12_0                     ((uint32_t)0x00010000) /* Bit 0 */
#define GPIO_CFGHR_MODE12_1                     ((uint32_t)0x00020000) /* Bit 1 */

#define GPIO_CFGHR_MODE13                       ((uint32_t)0x00300000) /* MODE13[1:0] bits (Port x mode bits, pin 13) */
#define GPIO_CFGHR_MODE13_0                     ((uint32_t)0x00100000) /* Bit 0 */
#define GPIO_CFGHR_MODE13_1                     ((uint32_t)0x00200000) /* Bit 1 */

#define GPIO_CFGHR_MODE14                       ((uint32_t)0x03000000) /* MODE14[1:0] bits (Port x mode bits, pin 14) */
#define GPIO_CFGHR_MODE14_0                     ((uint32_t)0x01000000) /* Bit 0 */
#define GPIO_CFGHR_MODE14_1                     ((uint32_t)0x02000000) /* Bit 1 */

#define GPIO_CFGHR_MODE15                       ((uint32_t)0x30000000) /* MODE15[1:0] bits (Port x mode bits, pin 15) */
#define GPIO_CFGHR_MODE15_0                     ((uint32_t)0x10000000) /* Bit 0 */
#define GPIO_CFGHR_MODE15_1                     ((uint32_t)0x20000000) /* Bit 1 */

#define GPIO_CFGHR_CNF                          ((uint32_t)0xCCCCCCCC) /* Port x configuration bits */

#define GPIO_CFGHR_CNF8                         ((uint32_t)0x0000000C) /* CNF8[1:0] bits (Port x configuration bits, pin 8) */
#define GPIO_CFGHR_CNF8_0                       ((uint32_t)0x00000004) /* Bit 0 */
#define GPIO_CFGHR_CNF8_1                       ((uint32_t)0x00000008) /* Bit 1 */

#define GPIO_CFGHR_CNF9                         ((uint32_t)0x000000C0) /* CNF9[1:0] bits (Port x configuration bits, pin 9) */
#define GPIO_CFGHR_CNF9_0                       ((uint32_t)0x00000040) /* Bit 0 */
#define GPIO_CFGHR_CNF9_1                       ((uint32_t)0x00000080) /* Bit 1 */

#define GPIO_CFGHR_CNF10                        ((uint32_t)0x00000C00) /* CNF10[1:0] bits (Port x configuration bits, pin 10) */
#define GPIO_CFGHR_CNF10_0                      ((uint32_t)0x00000400) /* Bit 0 */
#define GPIO_CFGHR_CNF10_1                      ((uint32_t)0x00000800) /* Bit 1 */

#define GPIO_CFGHR_CNF11                        ((uint32_t)0x0000C000) /* CNF11[1:0] bits (Port x configuration bits, pin 11) */
#define GPIO_CFGHR_CNF11_0                      ((uint32_t)0x00004000) /* Bit 0 */
#define GPIO_CFGHR_CNF11_1                      ((uint32_t)0x00008000) /* Bit 1 */

#define GPIO_CFGHR_CNF12                        ((uint32_t)0x000C0000) /* CNF12[1:0] bits (Port x configuration bits, pin 12) */
#define GPIO_CFGHR_CNF12_0                      ((uint32_t)0x00040000) /* Bit 0 */
#define GPIO_CFGHR_CNF12_1                      ((uint32_t)0x00080000) /* Bit 1 */

#define GPIO_CFGHR_CNF13                        ((uint32_t)0x00C00000) /* CNF13[1:0] bits (Port x configuration bits, pin 13) */
#define GPIO_CFGHR_CNF13_0                      ((uint32_t)0x00400000) /* Bit 0 */
#define GPIO_CFGHR_CNF13_1                      ((uint32_t)0x00800000) /* Bit 1 */

#define GPIO_CFGHR_CNF14                        ((uint32_t)0x0C000000) /* CNF14[1:0] bits (Port x configuration bits, pin 14) */
#define GPIO_CFGHR_CNF14_0                      ((uint32_t)0x04000000) /* Bit 0 */
#define GPIO_CFGHR_CNF14_1                      ((uint32_t)0x08000000) /* Bit 1 */

#define GPIO_CFGHR_CNF15                        ((uint32_t)0xC0000000) /* CNF15[1:0] bits (Port x configuration bits, pin 15) */
#define GPIO_CFGHR_CNF15_0                      ((uint32_t)0x40000000) /* Bit 0 */
#define GPIO_CFGHR_CNF15_1                      ((uint32_t)0x80000000) /* Bit 1 */

/*******************  Bit definition for GPIO_INDR register  *******************/
#define GPIO_INDR_IDR0                          ((uint16_t)0x0001) /* Port input data, bit 0 */
#define GPIO_INDR_IDR1                          ((uint16_t)0x0002) /* Port input data, bit 1 */
#define GPIO_INDR_IDR2                          ((uint16_t)0x0004) /* Port input data, bit 2 */
#define GPIO_INDR_IDR3                          ((uint16_t)0x0008) /* Port input data, bit 3 */
#define GPIO_INDR_IDR4                          ((uint16_t)0x0010) /* Port input data, bit 4 */
#define GPIO_INDR_IDR5                          ((uint16_t)0x0020) /* Port input data, bit 5 */
#define GPIO_INDR_IDR6                          ((uint16_t)0x0040) /* Port input data, bit 6 */
#define GPIO_INDR_IDR7                          ((uint16_t)0x0080) /* Port input data, bit 7 */
#define GPIO_INDR_IDR8                          ((uint16_t)0x0100) /* Port input data, bit 8 */
#define GPIO_INDR_IDR9                          ((uint16_t)0x0200) /* Port input data, bit 9 */
#define GPIO_INDR_IDR10                         ((uint16_t)0x0400) /* Port input data, bit 10 */
#define GPIO_INDR_IDR11                         ((uint16_t)0x0800) /* Port input data, bit 11 */
#define GPIO_INDR_IDR12                         ((uint16_t)0x1000) /* Port input data, bit 12 */
#define GPIO_INDR_IDR13                         ((uint16_t)0x2000) /* Port input data, bit 13 */
#define GPIO_INDR_IDR14                         ((uint16_t)0x4000) /* Port input data, bit 14 */
#define GPIO_INDR_IDR15                         ((uint16_t)0x8000) /* Port input data, bit 15 */
#define GPIO_INDR_IDR16                         ((uint32_t)0x10000) /* Port input data, bit 16 */
#define GPIO_INDR_IDR17                         ((uint32_t)0x20000) /* Port input data, bit 17 */
#define GPIO_INDR_IDR18                         ((uint32_t)0x40000) /* Port input data, bit 18 */
#define GPIO_INDR_IDR19                         ((uint32_t)0x80000) /* Port input data, bit 19 */
#define GPIO_INDR_IDR20                         ((uint32_t)0x100000) /* Port input data, bit 20 */
#define GPIO_INDR_IDR21                         ((uint32_t)0x200000) /* Port input data, bit 21 */
#define GPIO_INDR_IDR22                         ((uint32_t)0x400000) /* Port input data, bit 22 */
#define GPIO_INDR_IDR23                         ((uint32_t)0x800000) /* Port input data, bit 23 */

/*******************  Bit definition for GPIO_OUTDR register  *******************/
#define GPIO_OUTDR_ODR0                         ((uint16_t)0x0001) /* Port output data, bit 0 */
#define GPIO_OUTDR_ODR1                         ((uint16_t)0x0002) /* Port output data, bit 1 */
#define GPIO_OUTDR_ODR2                         ((uint16_t)0x0004) /* Port output data, bit 2 */
#define GPIO_OUTDR_ODR3                         ((uint16_t)0x0008) /* Port output data, bit 3 */
#define GPIO_OUTDR_ODR4                         ((uint16_t)0x0010) /* Port output data, bit 4 */
#define GPIO_OUTDR_ODR5                         ((uint16_t)0x0020) /* Port output data, bit 5 */
#define GPIO_OUTDR_ODR6                         ((uint16_t)0x0040) /* Port output data, bit 6 */
#define GPIO_OUTDR_ODR7                         ((uint16_t)0x0080) /* Port output data, bit 7 */
#define GPIO_OUTDR_ODR8                         ((uint16_t)0x0100) /* Port output data, bit 8 */
#define GPIO_OUTDR_ODR9                         ((uint16_t)0x0200) /* Port output data, bit 9 */
#define GPIO_OUTDR_ODR10                        ((uint16_t)0x0400) /* Port output data, bit 10 */
#define GPIO_OUTDR_ODR11                        ((uint16_t)0x0800) /* Port output data, bit 11 */
#define GPIO_OUTDR_ODR12                        ((uint16_t)0x1000) /* Port output data, bit 12 */
#define GPIO_OUTDR_ODR13                        ((uint16_t)0x2000) /* Port output data, bit 13 */
#define GPIO_OUTDR_ODR14                        ((uint16_t)0x4000) /* Port output data, bit 14 */
#define GPIO_OUTDR_ODR15                        ((uint16_t)0x8000) /* Port output data, bit 15 */
#define GPIO_OUTDR_ODR16                        ((uint32_t)0x10000) /* Port output data, bit 16 */
#define GPIO_OUTDR_ODR17                        ((uint32_t)0x20000) /* Port output data, bit 17 */
#define GPIO_OUTDR_ODR18                        ((uint32_t)0x40000) /* Port output data, bit 18 */
#define GPIO_OUTDR_ODR19                        ((uint32_t)0x80000) /* Port output data, bit 19 */
#define GPIO_OUTDR_ODR20                        ((uint32_t)0x100000) /* Port output data, bit 20 */
#define GPIO_OUTDR_ODR21                        ((uint32_t)0x200000) /* Port output data, bit 21 */
#define GPIO_OUTDR_ODR22                        ((uint32_t)0x400000) /* Port output data, bit 22 */
#define GPIO_OUTDR_ODR23                        ((uint32_t)0x800000) /* Port output data, bit 23 */

/******************  Bit definition for GPIO_BSHR register  *******************/
#define GPIO_BSHR_BS0                           ((uint32_t)0x00000001) /* Port x Set bit 0 */
#define GPIO_BSHR_BS1                           ((uint32_t)0x00000002) /* Port x Set bit 1 */
#define GPIO_BSHR_BS2                           ((uint32_t)0x00000004) /* Port x Set bit 2 */
#define GPIO_BSHR_BS3                           ((uint32_t)0x00000008) /* Port x Set bit 3 */
#define GPIO_BSHR_BS4                           ((uint32_t)0x00000010) /* Port x Set bit 4 */
#define GPIO_BSHR_BS5                           ((uint32_t)0x00000020) /* Port x Set bit 5 */
#define GPIO_BSHR_BS6                           ((uint32_t)0x00000040) /* Port x Set bit 6 */
#define GPIO_BSHR_BS7                           ((uint32_t)0x00000080) /* Port x Set bit 7 */
#define GPIO_BSHR_BS8                           ((uint32_t)0x00000100) /* Port x Set bit 8 */
#define GPIO_BSHR_BS9                           ((uint32_t)0x00000200) /* Port x Set bit 9 */
#define GPIO_BSHR_BS10                          ((uint32_t)0x00000400) /* Port x Set bit 10 */
#define GPIO_BSHR_BS11                          ((uint32_t)0x00000800) /* Port x Set bit 11 */
#define GPIO_BSHR_BS12                          ((uint32_t)0x00001000) /* Port x Set bit 12 */
#define GPIO_BSHR_BS13                          ((uint32_t)0x00002000) /* Port x Set bit 13 */
#define GPIO_BSHR_BS14                          ((uint32_t)0x00004000) /* Port x Set bit 14 */
#define GPIO_BSHR_BS15                          ((uint32_t)0x00008000) /* Port x Set bit 15 */

#define GPIO_BSHR_BR0                           ((uint32_t)0x00010000) /* Port x Reset bit 0 */
#define GPIO_BSHR_BR1                           ((uint32_t)0x00020000) /* Port x Reset bit 1 */
#define GPIO_BSHR_BR2                           ((uint32_t)0x00040000) /* Port x Reset bit 2 */
#define GPIO_BSHR_BR3                           ((uint32_t)0x00080000) /* Port x Reset bit 3 */
#define GPIO_BSHR_BR4                           ((uint32_t)0x00100000) /* Port x Reset bit 4 */
#define GPIO_BSHR_BR5                           ((uint32_t)0x00200000) /* Port x Reset bit 5 */
#define GPIO_BSHR_BR6                           ((uint32_t)0x00400000) /* Port x Reset bit 6 */
#define GPIO_BSHR_BR7                           ((uint32_t)0x00800000) /* Port x Reset bit 7 */
#define GPIO_BSHR_BR8                           ((uint32_t)0x01000000) /* Port x Reset bit 8 */
#define GPIO_BSHR_BR9                           ((uint32_t)0x02000000) /* Port x Reset bit 9 */
#define GPIO_BSHR_BR10                          ((uint32_t)0x04000000) /* Port x Reset bit 10 */
#define GPIO_BSHR_BR11                          ((uint32_t)0x08000000) /* Port x Reset bit 11 */
#define GPIO_BSHR_BR12                          ((uint32_t)0x10000000) /* Port x Reset bit 12 */
#define GPIO_BSHR_BR13                          ((uint32_t)0x20000000) /* Port x Reset bit 13 */
#define GPIO_BSHR_BR14                          ((uint32_t)0x40000000) /* Port x Reset bit 14 */
#define GPIO_BSHR_BR15                          ((uint32_t)0x80000000) /* Port x Reset bit 15 */

/*******************  Bit definition for GPIO_BCR register  *******************/
#define GPIO_BCR_BR0                            ((uint16_t)0x0001) /* Port x Reset bit 0 */
#define GPIO_BCR_BR1                            ((uint16_t)0x0002) /* Port x Reset bit 1 */
#define GPIO_BCR_BR2                            ((uint16_t)0x0004) /* Port x Reset bit 2 */
#define GPIO_BCR_BR3                            ((uint16_t)0x0008) /* Port x Reset bit 3 */
#define GPIO_BCR_BR4                            ((uint16_t)0x0010) /* Port x Reset bit 4 */
#define GPIO_BCR_BR5                            ((uint16_t)0x0020) /* Port x Reset bit 5 */
#define GPIO_BCR_BR6                            ((uint16_t)0x0040) /* Port x Reset bit 6 */
#define GPIO_BCR_BR7                            ((uint16_t)0x0080) /* Port x Reset bit 7 */
#define GPIO_BCR_BR8                            ((uint16_t)0x0100) /* Port x Reset bit 8 */
#define GPIO_BCR_BR9                            ((uint16_t)0x0200) /* Port x Reset bit 9 */
#define GPIO_BCR_BR10                           ((uint16_t)0x0400) /* Port x Reset bit 10 */
#define GPIO_BCR_BR11                           ((uint16_t)0x0800) /* Port x Reset bit 11 */
#define GPIO_BCR_BR12                           ((uint16_t)0x1000) /* Port x Reset bit 12 */
#define GPIO_BCR_BR13                           ((uint16_t)0x2000) /* Port x Reset bit 13 */
#define GPIO_BCR_BR14                           ((uint16_t)0x4000) /* Port x Reset bit 14 */
#define GPIO_BCR_BR15                           ((uint16_t)0x8000) /* Port x Reset bit 15 */
#define GPIO_BCR_BR16                           ((uint32_t)0x10000) /* Port x Reset bit 16 */
#define GPIO_BCR_BR17                           ((uint32_t)0x20000) /* Port x Reset bit 17 */
#define GPIO_BCR_BR18                           ((uint32_t)0x40000) /* Port x Reset bit 18 */
#define GPIO_BCR_BR19                           ((uint32_t)0x80000) /* Port x Reset bit 19 */
#define GPIO_BCR_BR20                           ((uint32_t)0x100000) /* Port x Reset bit 20 */
#define GPIO_BCR_BR21                           ((uint32_t)0x200000) /* Port x Reset bit 21 */
#define GPIO_BCR_BR22                           ((uint32_t)0x400000) /* Port x Reset bit 22 */
#define GPIO_BCR_BR23                           ((uint32_t)0x800000) /* Port x Reset bit 23 */


/******************  Bit definition for GPIO_LCKR register  *******************/
#define GPIO_LCK0                               ((uint32_t)0x00000001) /* Port x Lock bit 0 */
#define GPIO_LCK1                               ((uint32_t)0x00000002) /* Port x Lock bit 1 */
#define GPIO_LCK2                               ((uint32_t)0x00000004) /* Port x Lock bit 2 */
#define GPIO_LCK3                               ((uint32_t)0x00000008) /* Port x Lock bit 3 */
#define GPIO_LCK4                               ((uint32_t)0x00000010) /* Port x Lock bit 4 */
#define GPIO_LCK5                               ((uint32_t)0x00000020) /* Port x Lock bit 5 */
#define GPIO_LCK6                               ((uint32_t)0x00000040) /* Port x Lock bit 6 */
#define GPIO_LCK7                               ((uint32_t)0x00000080) /* Port x Lock bit 7 */
#define GPIO_LCK8                               ((uint32_t)0x00000100) /* Port x Lock bit 8 */
#define GPIO_LCK9                               ((uint32_t)0x00000200) /* Port x Lock bit 9 */
#define GPIO_LCK10                              ((uint32_t)0x00000400) /* Port x Lock bit 10 */
#define GPIO_LCK11                              ((uint32_t)0x00000800) /* Port x Lock bit 11 */
#define GPIO_LCK12                              ((uint32_t)0x00001000) /* Port x Lock bit 12 */
#define GPIO_LCK13                              ((uint32_t)0x00002000) /* Port x Lock bit 13 */
#define GPIO_LCK14                              ((uint32_t)0x00004000) /* Port x Lock bit 14 */
#define GPIO_LCK15                              ((uint32_t)0x00008000) /* Port x Lock bit 15 */
#define GPIO_LCK16                              ((uint32_t)0x00010000) /* Port x Lock bit 16 */
#define GPIO_LCK17                              ((uint32_t)0x00020000) /* Port x Lock bit 17 */
#define GPIO_LCK18                              ((uint32_t)0x00040000) /* Port x Lock bit 18 */
#define GPIO_LCK19                              ((uint32_t)0x00080000) /* Port x Lock bit 19 */
#define GPIO_LCK20                              ((uint32_t)0x00100000) /* Port x Lock bit 20 */
#define GPIO_LCK21                              ((uint32_t)0x00200000) /* Port x Lock bit 21 */
#define GPIO_LCK22                              ((uint32_t)0x00400000) /* Port x Lock bit 22 */
#define GPIO_LCK23                              ((uint32_t)0x00800000) /* Port x Lock bit 23 */

#define GPIO_LCKK                               ((uint32_t)0x01000000) /* Lock key */

/*******************  Bit definition for GPIO_CFGXR register  *******************/
#define GPIO_CFGXR_MODE                         ((uint32_t)0x33333333) /* Port x mode bits */

#define GPIO_CFGXR_MODE16                       ((uint32_t)0x00000003) /* MODE16[1:0] bits (Port x mode bits, pin 0) */
#define GPIO_CFGXR_MODE16_0                     ((uint32_t)0x00000001) /* Bit 0 */
#define GPIO_CFGXR_MODE16_1                     ((uint32_t)0x00000002) /* Bit 1 */

#define GPIO_CFGXR_MODE17                       ((uint32_t)0x00000030) /* MODE17[1:0] bits (Port x mode bits, pin 1) */
#define GPIO_CFGXR_MODE17_0                     ((uint32_t)0x00000010) /* Bit 0 */
#define GPIO_CFGXR_MODE17_1                     ((uint32_t)0x00000020) /* Bit 1 */

#define GPIO_CFGXR_MODE18                       ((uint32_t)0x00000300) /* MODE18[1:0] bits (Port x mode bits, pin 2) */
#define GPIO_CFGXR_MODE18_0                     ((uint32_t)0x00000100) /* Bit 0 */
#define GPIO_CFGXR_MODE18_1                     ((uint32_t)0x00000200) /* Bit 1 */

#define GPIO_CFGXR_MODE19                       ((uint32_t)0x00003000) /* MODE19[1:0] bits (Port x mode bits, pin 3) */
#define GPIO_CFGXR_MODE19_0                     ((uint32_t)0x00001000) /* Bit 0 */
#define GPIO_CFGXR_MODE19_1                     ((uint32_t)0x00002000) /* Bit 1 */

#define GPIO_CFGXR_MODE20                       ((uint32_t)0x00030000) /* MODE20[1:0] bits (Port x mode bits, pin 4) */
#define GPIO_CFGXR_MODE20_0                     ((uint32_t)0x00010000) /* Bit 0 */
#define GPIO_CFGXR_MODE20_1                     ((uint32_t)0x00020000) /* Bit 1 */

#define GPIO_CFGXR_MODE21                       ((uint32_t)0x00300000) /* MODE21[1:0] bits (Port x mode bits, pin 5) */
#define GPIO_CFGXR_MODE21_0                     ((uint32_t)0x00100000) /* Bit 0 */
#define GPIO_CFGXR_MODE21_1                     ((uint32_t)0x00200000) /* Bit 1 */

#define GPIO_CFGXR_MODE22                       ((uint32_t)0x03000000) /* MODE22[1:0] bits (Port x mode bits, pin 6) */
#define GPIO_CFGXR_MODE22_0                     ((uint32_t)0x01000000) /* Bit 0 */
#define GPIO_CFGXR_MODE22_1                     ((uint32_t)0x02000000) /* Bit 1 */

#define GPIO_CFGXR_MODE23                       ((uint32_t)0x30000000) /* MODE23[1:0] bits (Port x mode bits, pin 7) */
#define GPIO_CFGXR_MODE23_0                     ((uint32_t)0x10000000) /* Bit 0 */
#define GPIO_CFGXR_MODE23_1                     ((uint32_t)0x20000000) /* Bit 1 */

#define GPIO_CFGXR_CNF                          ((uint32_t)0xCCCCCCCC) /* Port x configuration bits */

#define GPIO_CFGXR_CNF16                        ((uint32_t)0x0000000C) /* CNF16[1:0] bits (Port x configuration bits, pin 0) */
#define GPIO_CFGXR_CNF16_0                      ((uint32_t)0x00000004) /* Bit 0 */
#define GPIO_CFGXR_CNF16_1                      ((uint32_t)0x00000008) /* Bit 1 */

#define GPIO_CFGXR_CNF17                        ((uint32_t)0x000000C0) /* CNF17[1:0] bits (Port x configuration bits, pin 1) */
#define GPIO_CFGXR_CNF17_0                      ((uint32_t)0x00000040) /* Bit 0 */
#define GPIO_CFGXR_CNF17_1                      ((uint32_t)0x00000080) /* Bit 1 */

#define GPIO_CFGXR_CNF18                        ((uint32_t)0x00000C00) /* CNF18[1:0] bits (Port x configuration bits, pin 2) */
#define GPIO_CFGXR_CNF18_0                      ((uint32_t)0x00000400) /* Bit 0 */
#define GPIO_CFGXR_CNF18_1                      ((uint32_t)0x00000800) /* Bit 1 */

#define GPIO_CFGXR_CNF19                        ((uint32_t)0x0000C000) /* CNF19[1:0] bits (Port x configuration bits, pin 3) */
#define GPIO_CFGXR_CNF19_0                      ((uint32_t)0x00004000) /* Bit 0 */
#define GPIO_CFGXR_CNF19_1                      ((uint32_t)0x00008000) /* Bit 1 */

#define GPIO_CFGXR_CNF20                        ((uint32_t)0x000C0000) /* CNF20[1:0] bits (Port x configuration bits, pin 4) */
#define GPIO_CFGXR_CNF20_0                      ((uint32_t)0x00040000) /* Bit 0 */
#define GPIO_CFGXR_CNF20_1                      ((uint32_t)0x00080000) /* Bit 1 */

#define GPIO_CFGXR_CNF21                        ((uint32_t)0x00C00000) /* CNF21[1:0] bits (Port x configuration bits, pin 5) */
#define GPIO_CFGXR_CNF21_0                      ((uint32_t)0x00400000) /* Bit 0 */
#define GPIO_CFGXR_CNF21_1                      ((uint32_t)0x00800000) /* Bit 1 */

#define GPIO_CFGXR_CNF22                        ((uint32_t)0x0C000000) /* CNF22[1:0] bits (Port x configuration bits, pin 6) */
#define GPIO_CFGXR_CNF22_0                      ((uint32_t)0x04000000) /* Bit 0 */
#define GPIO_CFGXR_CNF22_1                      ((uint32_t)0x08000000) /* Bit 1 */

#define GPIO_CFGXR_CNF23                        ((uint32_t)0xC0000000) /* CNF23[1:0] bits (Port x configuration bits, pin 7) */
#define GPIO_CFGXR_CNF23_0                      ((uint32_t)0x40000000) /* Bit 0 */
#define GPIO_CFGXR_CNF23_1                      ((uint32_t)0x80000000) /* Bit 1 */

/******************  Bit definition for GPIO_BSXR register  *******************/
#define GPIO_BSXR_BS16                          ((uint32_t)0x00000001) /* Port x Set bit 0 */
#define GPIO_BSXR_BS17                          ((uint32_t)0x00000002) /* Port x Set bit 1 */
#define GPIO_BSXR_BS18                          ((uint32_t)0x00000004) /* Port x Set bit 2 */
#define GPIO_BSXR_BS19                          ((uint32_t)0x00000008) /* Port x Set bit 3 */
#define GPIO_BSXR_BS20                          ((uint32_t)0x00000010) /* Port x Set bit 4 */
#define GPIO_BSXR_BS21                          ((uint32_t)0x00000020) /* Port x Set bit 5 */
#define GPIO_BSXR_BS22                          ((uint32_t)0x00000040) /* Port x Set bit 6 */
#define GPIO_BSXR_BS23                          ((uint32_t)0x00000080) /* Port x Set bit 7 */

#define GPIO_BSXR_BR16                          ((uint32_t)0x00010000) /* Port x Reset bit 0 */
#define GPIO_BSXR_BR17                          ((uint32_t)0x00020000) /* Port x Reset bit 1 */
#define GPIO_BSXR_BR18                          ((uint32_t)0x00040000) /* Port x Reset bit 2 */
#define GPIO_BSXR_BR19                          ((uint32_t)0x00080000) /* Port x Reset bit 3 */
#define GPIO_BSXR_BR20                          ((uint32_t)0x00100000) /* Port x Reset bit 4 */
#define GPIO_BSXR_BR21                          ((uint32_t)0x00200000) /* Port x Reset bit 5 */
#define GPIO_BSXR_BR22                          ((uint32_t)0x00400000) /* Port x Reset bit 6 */
#define GPIO_BSXR_BR23                          ((uint32_t)0x00800000) /* Port x Reset bit 7 */

/******************  Bit definition for AFIO_PCFR1register  *******************/
#define AFIO_PCFR1_SPI1_REMAP                   ((uint32_t)0x00000003) /* SPI1_REMAP[1:0] bits (SPI1 remapping) */
#define AFIO_PCFR1_SPI1_REMAP_0                 ((uint32_t)0x00000001) /* Bit 0 */
#define AFIO_PCFR1_SPI1_REMAP_1                 ((uint32_t)0x00000002) /* Bit 1 */

#define AFIO_PCFR1_I2C1_REMAP                   ((uint32_t)0x0000001C) /* I2C1_REMAP[4:2] bits (I2C1 remapping) */
#define AFIO_PCFR1_I2C1_REMAP_0                 ((uint32_t)0x00000004) /* Bit 0 */
#define AFIO_PCFR1_I2C1_REMAP_1                 ((uint32_t)0x00000008) /* Bit 1 */
#define AFIO_PCFR1_I2C1_REMAP_2                 ((uint32_t)0x00000010) /* Bit 2 */

#define AFIO_PCFR1_USART1_REMAP                 ((uint32_t)0x00000060) /* USART1_REMAP[6:5] bits (USART1 remapping) */
#define AFIO_PCFR1_USART1_REMAP_0               ((uint32_t)0x00000020) /* Bit 0 */
#define AFIO_PCFR1_USART1_REMAP_1               ((uint32_t)0x00000040) /* Bit 1 */

#define AFIO_PCFR1_USART2_REMAP                 ((uint32_t)0x00000380) /* USART2_REMAP[9:7] bits (USART2 remapping) */
#define AFIO_PCFR1_USART2_REMAP_0               ((uint32_t)0x00000080) /* Bit 0 */
#define AFIO_PCFR1_USART2_REMAP_1               ((uint32_t)0x00000100) /* Bit 1 */
#define AFIO_PCFR1_USART2_REMAP_2               ((uint32_t)0x00000200) /* Bit 2 */

#define AFIO_PCFR1_USART3_REMAP                 ((uint32_t)0x00000C00) /* USART3_REMAP[11:10] bits (USART3 remapping) */
#define AFIO_PCFR1_USART3_REMAP_0               ((uint32_t)0x00000400) /* Bit 0 */
#define AFIO_PCFR1_USART3_REMAP_1               ((uint32_t)0x00000800) /* Bit 1 */

#define AFIO_PCFR1_USART4_REMAP                 ((uint32_t)0x00007000) /* USART4_REMAP[14:12] bits (USART4 remapping) */
#define AFIO_PCFR1_USART4_REMAP_0               ((uint32_t)0x00001000) /* Bit 0 */
#define AFIO_PCFR1_USART4_REMAP_1               ((uint32_t)0x00002000) /* Bit 1 */
#define AFIO_PCFR1_USART4_REMAP_2               ((uint32_t)0x00004000) /* Bit 2 */

#define AFIO_PCFR1_TIM1_REMAP                   ((uint32_t)0x00038000) /* TIM1_REMAP[17:15] bits (TIM1 remapping) */
#define AFIO_PCFR1_TIM1_REMAP_0                 ((uint32_t)0x00008000) /* Bit 0 */
#define AFIO_PCFR1_TIM1_REMAP_1                 ((uint32_t)0x00010000) /* Bit 1 */
#define AFIO_PCFR1_TIM1_REMAP_2                 ((uint32_t)0x00020000) /* Bit 2 */

#define AFIO_PCFR1_TIM2_REMAP                   ((uint32_t)0x001C0000) /* TIM2_REMAP[20:18] bits (TIM2 remapping) */
#define AFIO_PCFR1_TIM2_REMAP_0                 ((uint32_t)0x00040000) /* Bit 0 */
#define AFIO_PCFR1_TIM2_REMAP_1                 ((uint32_t)0x00080000) /* Bit 1 */
#define AFIO_PCFR1_TIM2_REMAP_2                 ((uint32_t)0x00100000) /* Bit 2 */

#define AFIO_PCFR1_TIM3_REMAP                   ((uint32_t)0x00600000) /* TIM3_REMAP[22:21] bits (TIM3 remapping) */
#define AFIO_PCFR1_TIM3_REMAP_0                 ((uint32_t)0x00200000) /* Bit 0 */
#define AFIO_PCFR1_TIM3_REMAP_1                 ((uint32_t)0x00400000) /* Bit 1 */

#define AFIO_PCFR1_PIOC_REMAP                   ((uint32_t)0x00800000) /* PIOC[23] bits (PIOC remapping) */

#define AFIO_PCFR1_SWJ_CFG                      ((uint32_t)0x07000000) /* SWJ_CFG[2:0] bits (Serial Wire JTAG configuration) */
#define AFIO_PCFR1_SWJ_CFG_0                    ((uint32_t)0x01000000) /* Bit 0 */
#define AFIO_PCFR1_SWJ_CFG_1                    ((uint32_t)0x02000000) /* Bit 1 */
#define AFIO_PCFR1_SWJ_CFG_2                    ((uint32_t)0x04000000) /* Bit 2 */

/*****************  Bit definition for AFIO_EXTICR1 register  *****************/
#define AFIO_EXTICR1_EXTI0                      ((uint32_t)0x00000003) /* EXTI 0 configuration */
#define AFIO_EXTICR1_EXTI1                      ((uint32_t)0x0000000C) /* EXTI 1 configuration */
#define AFIO_EXTICR1_EXTI2                      ((uint32_t)0x00000030) /* EXTI 2 configuration */
#define AFIO_EXTICR1_EXTI3                      ((uint32_t)0x000000C0) /* EXTI 3 configuration */
#define AFIO_EXTICR1_EXTI4                      ((uint32_t)0x00000300) /* EXTI 4 configuration */
#define AFIO_EXTICR1_EXTI5                      ((uint32_t)0x00000C00) /* EXTI 5 configuration */
#define AFIO_EXTICR1_EXTI6                      ((uint32_t)0x00003000) /* EXTI 6 configuration */
#define AFIO_EXTICR1_EXTI7                      ((uint32_t)0x0000C000) /* EXTI 7 configuration */
#define AFIO_EXTICR1_EXTI8                      ((uint32_t)0x00030000) /* EXTI 8 configuration */
#define AFIO_EXTICR1_EXTI9                      ((uint32_t)0x000C0000) /* EXTI 9 configuration */
#define AFIO_EXTICR1_EXTI10                     ((uint32_t)0x00300000) /* EXTI 10 configuration */
#define AFIO_EXTICR1_EXTI11                     ((uint32_t)0x00C00000) /* EXTI 11 configuration */
#define AFIO_EXTICR1_EXTI12                     ((uint32_t)0x03000000) /* EXTI 12 configuration */
#define AFIO_EXTICR1_EXTI13                     ((uint32_t)0x0C000000) /* EXTI 13 configuration */
#define AFIO_EXTICR1_EXTI14                     ((uint32_t)0x30000000) /* EXTI 14 configuration */
#define AFIO_EXTICR1_EXTI15                     ((uint32_t)0xC0000000) /* EXTI 15 configuration */

#define AFIO_EXTICR1_EXTI0_PA                   ((uint32_t)0x00000000) /* PA[0] pin */
#define AFIO_EXTICR1_EXTI0_PB                   ((uint32_t)0x00000001) /* PB[0] pin */
#define AFIO_EXTICR1_EXTI0_PC                   ((uint32_t)0x00000002) /* PC[0] pin */

#define AFIO_EXTICR1_EXTI1_PA                   ((uint32_t)0x00000000) /* PA[1] pin */
#define AFIO_EXTICR1_EXTI1_PB                   ((uint32_t)0x00000004) /* PB[1] pin */
#define AFIO_EXTICR1_EXTI1_PC                   ((uint32_t)0x00000008) /* PC[1] pin */

#define AFIO_EXTICR1_EXTI2_PA                   ((uint32_t)0x00000000) /* PA[2] pin */
#define AFIO_EXTICR1_EXTI2_PB                   ((uint32_t)0x00000010) /* PB[2] pin */
#define AFIO_EXTICR1_EXTI2_PC                   ((uint32_t)0x00000020) /* PC[2] pin */

#define AFIO_EXTICR1_EXTI3_PA                   ((uint32_t)0x00000000) /* PA[3] pin */
#define AFIO_EXTICR1_EXTI3_PB                   ((uint32_t)0x00000040) /* PB[3] pin */
#define AFIO_EXTICR1_EXTI3_PC                   ((uint32_t)0x00000080) /* PC[3] pin */

#define AFIO_EXTICR1_EXTI4_PA                   ((uint32_t)0x00000000) /* PA[4] pin */
#define AFIO_EXTICR1_EXTI4_PB                   ((uint32_t)0x00000100) /* PB[4] pin */
#define AFIO_EXTICR1_EXTI4_PC                   ((uint32_t)0x00000200) /* PC[4] pin */

#define AFIO_EXTICR1_EXTI5_PA                   ((uint32_t)0x00000000) /* PA[5] pin */
#define AFIO_EXTICR1_EXTI5_PB                   ((uint32_t)0x00000400) /* PB[5] pin */
#define AFIO_EXTICR1_EXTI5_PC                   ((uint32_t)0x00000800) /* PC[5] pin */

#define AFIO_EXTICR1_EXTI6_PA                   ((uint32_t)0x00000000) /* PA[6] pin */
#define AFIO_EXTICR1_EXTI6_PB                   ((uint32_t)0x00001000) /* PB[6] pin */
#define AFIO_EXTICR1_EXTI6_PC                   ((uint32_t)0x00002000) /* PC[6] pin */

#define AFIO_EXTICR1_EXTI7_PA                   ((uint32_t)0x00000000) /* PA[7] pin */
#define AFIO_EXTICR1_EXTI7_PB                   ((uint32_t)0x00004000) /* PB[7] pin */
#define AFIO_EXTICR1_EXTI7_PC                   ((uint32_t)0x00008000) /* PC[7] pin */

#define AFIO_EXTICR1_EXTI8_PA                   ((uint32_t)0x00000000) /* PA[8] pin */
#define AFIO_EXTICR1_EXTI8_PB                   ((uint32_t)0x00010000) /* PB[8] pin */
#define AFIO_EXTICR1_EXTI8_PC                   ((uint32_t)0x00020000) /* PC[8] pin */

#define AFIO_EXTICR1_EXTI9_PA                   ((uint32_t)0x00000000) /* PA[9] pin */
#define AFIO_EXTICR1_EXTI9_PB                   ((uint32_t)0x00040000) /* PB[9] pin */
#define AFIO_EXTICR1_EXTI9_PC                   ((uint32_t)0x00080000) /* PC[9] pin */

#define AFIO_EXTICR1_EXTI10_PA                  ((uint32_t)0x00000000) /* PA[10] pin */
#define AFIO_EXTICR1_EXTI10_PB                  ((uint32_t)0x00100000) /* PB[10] pin */
#define AFIO_EXTICR1_EXTI10_PC                  ((uint32_t)0x00200000) /* PC[10] pin */

#define AFIO_EXTICR1_EXTI11_PA                  ((uint32_t)0x00000000) /* PA[11] pin */
#define AFIO_EXTICR1_EXTI11_PB                  ((uint32_t)0x00400000) /* PB[11] pin */
#define AFIO_EXTICR1_EXTI11_PC                  ((uint32_t)0x00800000) /* PC[11] pin */

#define AFIO_EXTICR1_EXTI12_PA                  ((uint32_t)0x00000000) /* PA[12] pin */
#define AFIO_EXTICR1_EXTI12_PB                  ((uint32_t)0x01000000) /* PB[12] pin */
#define AFIO_EXTICR1_EXTI12_PC                  ((uint32_t)0x02000000) /* PC[12] pin */

#define AFIO_EXTICR1_EXTI13_PA                  ((uint32_t)0x00000000) /* PA[13] pin */
#define AFIO_EXTICR1_EXTI13_PB                  ((uint32_t)0x04000000) /* PB[13] pin */
#define AFIO_EXTICR1_EXTI13_PC                  ((uint32_t)0x08000000) /* PC[13] pin */

#define AFIO_EXTICR1_EXTI14_PA                  ((uint32_t)0x00000000) /* PA[14] pin */
#define AFIO_EXTICR1_EXTI14_PB                  ((uint32_t)0x10000000) /* PB[14] pin */
#define AFIO_EXTICR1_EXTI14_PC                  ((uint32_t)0x20000000) /* PC[14] pin */

#define AFIO_EXTICR1_EXTI15_PA                  ((uint32_t)0x00000000) /* PA[15] pin */
#define AFIO_EXTICR1_EXTI15_PB                  ((uint32_t)0x40000000) /* PB[15] pin */
#define AFIO_EXTICR1_EXTI15_PC                  ((uint32_t)0x80000000) /* PC[15] pin */

/*****************  Bit definition for AFIO_EXTICR2 register  *****************/
#define AFIO_EXTICR2_EXTI16                     ((uint32_t)0x00000003) /* EXTI 16 configuration */
#define AFIO_EXTICR2_EXTI17                     ((uint32_t)0x0000000C) /* EXTI 17 configuration */
#define AFIO_EXTICR2_EXTI18                     ((uint32_t)0x00000030) /* EXTI 18 configuration */
#define AFIO_EXTICR2_EXTI19                     ((uint32_t)0x000000C0) /* EXTI 19 configuration */
#define AFIO_EXTICR2_EXTI20                     ((uint32_t)0x00000300) /* EXTI 20 configuration */
#define AFIO_EXTICR2_EXTI21                     ((uint32_t)0x00000C00) /* EXTI 21 configuration */
#define AFIO_EXTICR2_EXTI22                     ((uint32_t)0x00003000) /* EXTI 22 configuration */
#define AFIO_EXTICR2_EXTI23                     ((uint32_t)0x0000C000) /* EXTI 23 configuration */

#define AFIO_EXTICR2_EXTI16_PA                  ((uint32_t)0x00000000) /* PA[16] pin */
#define AFIO_EXTICR2_EXTI16_PB                  ((uint32_t)0x00000001) /* PB[16] pin */
#define AFIO_EXTICR2_EXTI16_PC                  ((uint32_t)0x00000002) /* PC[16] pin */

#define AFIO_EXTICR2_EXTI17_PA                  ((uint32_t)0x00000000) /* PA[17] pin */
#define AFIO_EXTICR2_EXTI17_PB                  ((uint32_t)0x00000004) /* PB[17] pin */
#define AFIO_EXTICR2_EXTI17_PC                  ((uint32_t)0x00000008) /* PC[17] pin */

#define AFIO_EXTICR2_EXTI18_PA                  ((uint32_t)0x00000000) /* PA[18] pin */
#define AFIO_EXTICR2_EXTI18_PB                  ((uint32_t)0x00000010) /* PB[18] pin */
#define AFIO_EXTICR2_EXTI18_PC                  ((uint32_t)0x00000020) /* PC[18] pin */

#define AFIO_EXTICR2_EXTI19_PA                  ((uint32_t)0x00000000) /* PA[19] pin */
#define AFIO_EXTICR2_EXTI19_PB                  ((uint32_t)0x00000040) /* PB[19] pin */
#define AFIO_EXTICR2_EXTI19_PC                  ((uint32_t)0x00000080) /* PC[19] pin */

#define AFIO_EXTICR2_EXTI20_PA                  ((uint32_t)0x00000000) /* PA[20] pin */
#define AFIO_EXTICR2_EXTI20_PB                  ((uint32_t)0x00000100) /* PB[20] pin */
#define AFIO_EXTICR2_EXTI20_PC                  ((uint32_t)0x00000200) /* PC[20] pin */

#define AFIO_EXTICR2_EXTI21_PA                  ((uint32_t)0x00000000) /* PA[21] pin */
#define AFIO_EXTICR2_EXTI21_PB                  ((uint32_t)0x00000400) /* PB[21] pin */
#define AFIO_EXTICR2_EXTI21_PC                  ((uint32_t)0x00000800) /* PC[21] pin */

#define AFIO_EXTICR2_EXTI22_PA                  ((uint32_t)0x00000000) /* PA[22] pin */
#define AFIO_EXTICR2_EXTI22_PB                  ((uint32_t)0x00001000) /* PB[22] pin */
#define AFIO_EXTICR2_EXTI22_PC                  ((uint32_t)0x00002000) /* PC[22] pin */

#define AFIO_EXTICR2_EXTI23_PA                  ((uint32_t)0x00000000) /* PA[23] pin */
#define AFIO_EXTICR2_EXTI23_PB                  ((uint32_t)0x00004000) /* PB[23] pin */
#define AFIO_EXTICR2_EXTI23_PC                  ((uint32_t)0x00008000) /* PC[23] pin */

/*******************  Bit definition for AFIO_CTLR register  ********************/
#define AFIO_CTLR_UDM_PUE                       ((uint32_t)0x00000003) /* PC16/UDM Pin pull-up Mode*/
#define AFIO_CTLR_UDM_PUE_0                     ((uint32_t)0x00000001) /* bit[0] */
#define AFIO_CTLR_UDM_PUE_1                     ((uint32_t)0x00000002) /* bit[1] */

#define AFIO_CTLR_UDP_PUE                       ((uint32_t)0x0000000C) /* PC17/UDP Pin pull-up Mode*/
#define AFIO_CTLR_UDP_PUE_0                     ((uint32_t)0x00000004) /* bit[2] */
#define AFIO_CTLR_UDP_PUE_1                     ((uint32_t)0x00000008) /* bit[3] */

#define AFIO_CTLR_USB_PHY_V33                   ((uint32_t)0x00000040) /* USB transceiver PHY output and pull-up limiter configuration */
#define AFIO_CTLR_USB_IOEN                      ((uint32_t)0x00000080) /* USB Remap pin enable */
#define AFIO_CTLR_USBPD_PHY_V33                 ((uint32_t)0x00000100) /* USBPD transceiver PHY output and pull-up limiter configuration */
#define AFIO_CTLR_USBPD_IN_HVT                  ((uint32_t)0x00000200) /* PD pin PC14/PC15 high threshold input mode */
#define AFIO_CTLR_UDP_BC_VSRC                   ((uint32_t)0x00010000) /* PC17/UDP pin BC protocol source voltage enable */
#define AFIO_CTLR_UDM_BC_VSRC                   ((uint32_t)0x00020000) /* PC16/UDM pin BC protocol source voltage enable */
#define AFIO_CTLR_UDP_BC_CMPO                   ((uint32_t)0x00040000) /* PC17/UDP pin BC protocol comparator status */
#define AFIO_CTLR_UDM_BC_CMPO                   ((uint32_t)0x00080000) /* PC16/UDM pin BC protocol comparator status */
#define AFIO_CTLR_PA3_FILT_EN                   ((uint32_t)0x01000000) /* Controls the input filtering of the PA3 pin */
#define AFIO_CTLR_PA4_FILT_EN                   ((uint32_t)0x02000000) /* Controls the input filtering of the PA4 pin */
#define AFIO_CTLR_PB5_FILT_EN                   ((uint32_t)0x04000000) /* Controls the input filtering of the PB5 pin */
#define AFIO_CTLR_PB6_FILT_EN                   ((uint32_t)0x08000000) /* Controls the input filtering of the PB6 pin */

/******************************************************************************/
/*                           Independent WATCHDOG                             */
/******************************************************************************/

/*******************  Bit definition for IWDG_CTLR register  ********************/
#define IWDG_KEY                                ((uint16_t)0xFFFF) /* Key value (write only, read 0000h) */

/*******************  Bit definition for IWDG_PSCR register  ********************/
#define IWDG_PR                                 ((uint8_t)0x07) /* PR[2:0] (Prescaler divider) */
#define IWDG_PR_0                               ((uint8_t)0x01) /* Bit 0 */
#define IWDG_PR_1                               ((uint8_t)0x02) /* Bit 1 */
#define IWDG_PR_2                               ((uint8_t)0x04) /* Bit 2 */

/*******************  Bit definition for IWDG_RLDR register  *******************/
#define IWDG_RL                                 ((uint16_t)0x0FFF) /* Watchdog counter reload value */

/*******************  Bit definition for IWDG_STATR register  ********************/
#define IWDG_PVU                                ((uint8_t)0x01) /* Watchdog prescaler value update */
#define IWDG_RVU                                ((uint8_t)0x02) /* Watchdog counter reload value update */

/******************************************************************************/
/*                      Inter-integrated Circuit Interface                    */
/******************************************************************************/

/*******************  Bit definition for I2C_CTLR1 register  ********************/
#define I2C_CTLR1_PE                            ((uint16_t)0x0001) /* Peripheral Enable */
#define I2C_CTLR1_ENARP                         ((uint16_t)0x0010) /* ARP Enable */
#define I2C_CTLR1_ENPEC                         ((uint16_t)0x0020) /* PEC Enable */
#define I2C_CTLR1_ENGC                          ((uint16_t)0x0040) /* General Call Enable */
#define I2C_CTLR1_NOSTRETCH                     ((uint16_t)0x0080) /* Clock Stretching Disable (Slave mode) */
#define I2C_CTLR1_START                         ((uint16_t)0x0100) /* Start Generation */
#define I2C_CTLR1_STOP                          ((uint16_t)0x0200) /* Stop Generation */
#define I2C_CTLR1_ACK                           ((uint16_t)0x0400) /* Acknowledge Enable */
#define I2C_CTLR1_POS                           ((uint16_t)0x0800) /* Acknowledge/PEC Position (for data reception) */
#define I2C_CTLR1_PEC                           ((uint16_t)0x1000) /* Packet Error Checking */

#define I2C_CTLR1_SWRST                         ((uint16_t)0x8000) /* Software Reset */

/*******************  Bit definition for I2C_CTLR2 register  ********************/
#define I2C_CTLR2_FREQ                          ((uint16_t)0x003F) /* FREQ[5:0] bits (Peripheral Clock Frequency) */
#define I2C_CTLR2_FREQ_0                        ((uint16_t)0x0001) /* Bit 0 */
#define I2C_CTLR2_FREQ_1                        ((uint16_t)0x0002) /* Bit 1 */
#define I2C_CTLR2_FREQ_2                        ((uint16_t)0x0004) /* Bit 2 */
#define I2C_CTLR2_FREQ_3                        ((uint16_t)0x0008) /* Bit 3 */
#define I2C_CTLR2_FREQ_4                        ((uint16_t)0x0010) /* Bit 4 */
#define I2C_CTLR2_FREQ_5                        ((uint16_t)0x0020) /* Bit 5 */

#define I2C_CTLR2_ITERREN                       ((uint16_t)0x0100) /* Error Interrupt Enable */
#define I2C_CTLR2_ITEVTEN                       ((uint16_t)0x0200) /* Event Interrupt Enable */
#define I2C_CTLR2_ITBUFEN                       ((uint16_t)0x0400) /* Buffer Interrupt Enable */
#define I2C_CTLR2_DMAEN                         ((uint16_t)0x0800) /* DMA Requests Enable */
#define I2C_CTLR2_LAST                          ((uint16_t)0x1000) /* DMA Last Transfer */

/*******************  Bit definition for I2C_OADDR1 register  *******************/
#define I2C_OADDR1_ADD1_7                       ((uint16_t)0x00FE) /* Interface Address */
#define I2C_OADDR1_ADD8_9                       ((uint16_t)0x0300) /* Interface Address */

#define I2C_OADDR1_ADD0                         ((uint16_t)0x0001) /* Bit 0 */
#define I2C_OADDR1_ADD1                         ((uint16_t)0x0002) /* Bit 1 */
#define I2C_OADDR1_ADD2                         ((uint16_t)0x0004) /* Bit 2 */
#define I2C_OADDR1_ADD3                         ((uint16_t)0x0008) /* Bit 3 */
#define I2C_OADDR1_ADD4                         ((uint16_t)0x0010) /* Bit 4 */
#define I2C_OADDR1_ADD5                         ((uint16_t)0x0020) /* Bit 5 */
#define I2C_OADDR1_ADD6                         ((uint16_t)0x0040) /* Bit 6 */
#define I2C_OADDR1_ADD7                         ((uint16_t)0x0080) /* Bit 7 */
#define I2C_OADDR1_ADD8                         ((uint16_t)0x0100) /* Bit 8 */
#define I2C_OADDR1_ADD9                         ((uint16_t)0x0200) /* Bit 9 */

#define I2C_OADDR1_ADDMODE                      ((uint16_t)0x8000) /* Addressing Mode (Slave mode) */

/*******************  Bit definition for I2C_OADDR2 register  *******************/
#define I2C_OADDR2_ENDUAL                       ((uint8_t)0x01) /* Dual addressing mode enable */
#define I2C_OADDR2_ADD2                         ((uint8_t)0xFE) /* Interface address */

/********************  Bit definition for I2C_DATAR register  ********************/
#define I2C_DR_DATAR                            ((uint8_t)0xFF) /* 8-bit Data Register */

/*******************  Bit definition for I2C_STAR1 register  ********************/
#define I2C_STAR1_SB                            ((uint16_t)0x0001) /* Start Bit (Master mode) */
#define I2C_STAR1_ADDR                          ((uint16_t)0x0002) /* Address sent (master mode)/matched (slave mode) */
#define I2C_STAR1_BTF                           ((uint16_t)0x0004) /* Byte Transfer Finished */
#define I2C_STAR1_ADD10                         ((uint16_t)0x0008) /* 10-bit header sent (Master mode) */
#define I2C_STAR1_STOPF                         ((uint16_t)0x0010) /* Stop detection (Slave mode) */
#define I2C_STAR1_RXNE                          ((uint16_t)0x0040) /* Data Register not Empty (receivers) */
#define I2C_STAR1_TXE                           ((uint16_t)0x0080) /* Data Register Empty (transmitters) */
#define I2C_STAR1_BERR                          ((uint16_t)0x0100) /* Bus Error */
#define I2C_STAR1_ARLO                          ((uint16_t)0x0200) /* Arbitration Lost (master mode) */
#define I2C_STAR1_AF                            ((uint16_t)0x0400) /* Acknowledge Failure */
#define I2C_STAR1_OVR                           ((uint16_t)0x0800) /* Overrun/Underrun */
#define I2C_STAR1_PECERR                        ((uint16_t)0x1000) /* PEC Error in reception */

/*******************  Bit definition for I2C_STAR2 register  ********************/
#define I2C_STAR2_MSL                           ((uint16_t)0x0001) /* Master/Slave */
#define I2C_STAR2_BUSY                          ((uint16_t)0x0002) /* Bus Busy */
#define I2C_STAR2_TRA                           ((uint16_t)0x0004) /* Transmitter/Receiver */
#define I2C_STAR2_GENCALL                       ((uint16_t)0x0010) /* General Call Address (Slave mode) */
#define I2C_STAR2_DUALF                         ((uint16_t)0x0080) /* Dual Flag (Slave mode) */
#define I2C_STAR2_PEC                           ((uint16_t)0xFF00) /* Packet Error Checking Register */

/*******************  Bit definition for I2C_CKCFGR register  ********************/
#define I2C_CKCFGR_CCR                          ((uint16_t)0x0FFF) /* Clock Control Register in Fast/Standard mode (Master mode) */
#define I2C_CKCFGR_DUTY                         ((uint16_t)0x4000) /* Fast Mode Duty Cycle */
#define I2C_CKCFGR_FS                           ((uint16_t)0x8000) /* I2C Master Mode Selection */

/******************************************************************************/
/*                             Power Control                                  */
/******************************************************************************/

/********************  Bit definition for PWR_CTLR register  ********************/
#define PWR_CTLR_PDDS                           ((uint16_t)0x0002) /* Power Down Deepsleep */

#define PWR_CTLR_PLS                            ((uint16_t)0x0060) /* PLS[2:0] bits (PVD Level Selection) */
#define PWR_CTLR_PLS_0                          ((uint16_t)0x0020) /* Bit 0 */
#define PWR_CTLR_PLS_1                          ((uint16_t)0x0040) /* Bit 1 */

#define PWR_CTLR_PLS_MODE0                      ((uint16_t)0x0000) /* PVD level 0 */
#define PWR_CTLR_PLS_MODE1                      ((uint16_t)0x0020) /* PVD level 1 */
#define PWR_CTLR_PLS_MODE2                      ((uint16_t)0x0040) /* PVD level 2 */
#define PWR_CTLR_PLS_MODE3                      ((uint16_t)0x0060) /* PVD level 3 */

#define PWR_CTLR_LP_REG                         ((uint16_t)0x0200) /* Software configure flash into lower energy mode */
#define PWR_CTLR_LP                             ((uint16_t)0x0C00) /* Software configure flash Status */
#define PWR_CTLR_LP_0                           ((uint16_t)0x0400)
#define PWR_CTLR_LP_1                           ((uint16_t)0x0800)

/*******************  Bit definition for PWR_CSR register  ********************/
#define PWR_CSR_PVDO                            ((uint16_t)0x0004) /* PVD Output */
#define PWR_CSR_Flash_ack                       ((uint16_t)0x0200) /* Flash Status */

/******************************************************************************/
/*                         Reset and Clock Control                            */
/******************************************************************************/

/********************  Bit definition for RCC_CTLR register  ********************/
#define RCC_HSION                               ((uint32_t)0x00000001) /* Internal High Speed clock enable */
#define RCC_HSIRDY                              ((uint32_t)0x00000002) /* Internal High Speed clock ready flag */
#define RCC_HSITRIM                             ((uint32_t)0x000000F8) /* Internal High Speed clock trimming */
#define RCC_HSICAL                              ((uint32_t)0x0000FF00) /* Internal High Speed clock Calibration */

/*******************  Bit definition for RCC_CFGR0 register  *******************/
#define RCC_HPRE                                ((uint32_t)0x000000F0) /* HPRE[3:0] bits (AHB prescaler) */
#define RCC_HPRE_0                              ((uint32_t)0x00000010) /* Bit 0 */
#define RCC_HPRE_1                              ((uint32_t)0x00000020) /* Bit 1 */
#define RCC_HPRE_2                              ((uint32_t)0x00000040) /* Bit 2 */
#define RCC_HPRE_3                              ((uint32_t)0x00000080) /* Bit 3 */

#define RCC_HPRE_DIV1                           ((uint32_t)0x00000000) /* SYSCLK not divided */
#define RCC_HPRE_DIV2                           ((uint32_t)0x00000010) /* SYSCLK divided by 2 */
#define RCC_HPRE_DIV3                           ((uint32_t)0x00000020) /* SYSCLK divided by 3 */
#define RCC_HPRE_DIV4                           ((uint32_t)0x00000030) /* SYSCLK divided by 4 */
#define RCC_HPRE_DIV5                           ((uint32_t)0x00000040) /* SYSCLK divided by 5 */
#define RCC_HPRE_DIV6                           ((uint32_t)0x00000050) /* SYSCLK divided by 6 */
#define RCC_HPRE_DIV7                           ((uint32_t)0x00000060) /* SYSCLK divided by 7 */
#define RCC_HPRE_DIV8                           ((uint32_t)0x00000070) /* SYSCLK divided by 8 */
#define RCC_HPRE_DIV16                          ((uint32_t)0x000000B0) /* SYSCLK divided by 16 */
#define RCC_HPRE_DIV32                          ((uint32_t)0x000000C0) /* SYSCLK divided by 32 */
#define RCC_HPRE_DIV64                          ((uint32_t)0x000000D0) /* SYSCLK divided by 64 */
#define RCC_HPRE_DIV128                         ((uint32_t)0x000000E0) /* SYSCLK divided by 128 */
#define RCC_HPRE_DIV256                         ((uint32_t)0x000000F0) /* SYSCLK divided by 256 */

#define RCC_CFGR0_MCO                           ((uint32_t)0x07000000) /* MCO[2:0] bits (Microcontroller Clock Output) */
#define RCC_MCO_0                               ((uint32_t)0x01000000) /* Bit 0 */
#define RCC_MCO_1                               ((uint32_t)0x02000000) /* Bit 1 */
#define RCC_MCO_2                               ((uint32_t)0x04000000) /* Bit 2 */

#define RCC_MCO_NOCLOCK                         ((uint32_t)0x00000000) /* No clock */
#define RCC_CFGR0_MCO_SYSCLK                    ((uint32_t)0x04000000) /* System clock selected as MCO source */
#define RCC_CFGR0_MCO_HSI                       ((uint32_t)0x05000000) /* HSI clock selected as MCO source */

/*****************  Bit definition for RCC_APB2PRSTR register  *****************/
#define RCC_AFIORST                             ((uint32_t)0x00000001) /* Alternate Function I/O reset */
#define RCC_IOPARST                             ((uint32_t)0x00000004) /* I/O port A reset */
#define RCC_IOPBRST                             ((uint32_t)0x00000008) /* I/O port B reset */
#define RCC_IOPCRST                             ((uint32_t)0x00000010) /* I/O port C reset */
#define RCC_ADC1RST                             ((uint32_t)0x00000200) /* ADC 1 interface reset */
#define RCC_TIM1RST                             ((uint32_t)0x00000800) /* TIM1 Timer reset */
#define RCC_SPI1RST                             ((uint32_t)0x00001000) /* SPI 1 reset */
#define RCC_USART1RST                           ((uint32_t)0x00004000) /* USART1 reset */

/*****************  Bit definition for RCC_APB1PRSTR register  *****************/
#define RCC_TIM2RST                             ((uint32_t)0x00000001) /* Timer 2 reset */
#define RCC_TIM3RST                             ((uint32_t)0x00000002) /* Timer 3 reset */
#define RCC_WWDGRST                             ((uint32_t)0x00000800) /* Window Watchdog reset */
#define RCC_USART2RST                           ((uint32_t)0x00020000) /* USART 2 reset */
#define RCC_USART3RST                           ((uint32_t)0x00040000) /* USART 3 reset */
#define RCC_USART4RST                           ((uint32_t)0x00080000) /* USART 4 reset */
#define RCC_I2C1RST                             ((uint32_t)0x00200000) /* I2C 1 reset */
#define RCC_PWRRST                              ((uint32_t)0x10000000) /* Power interface reset */

/******************  Bit definition for RCC_AHBPCENR register  ******************/
#define RCC_DMA1EN                              ((uint32_t)0x00000001) /* DMA1 clock enable */
#define RCC_SRAMEN                              ((uint32_t)0x00000004) /* SRAM interface clock enable */
#define RCC_USBFS                               ((uint32_t)0x00001000) /* USBFS clock enable */
#define RCC_USBPD                               ((uint32_t)0x00020000) /* USBPD clock enable */

/******************  Bit definition for RCC_APB2PCENR register  *****************/
#define RCC_AFIOEN                              ((uint32_t)0x00000001) /* Alternate Function I/O clock enable */
#define RCC_IOPAEN                              ((uint32_t)0x00000004) /* I/O port A clock enable */
#define RCC_IOPBEN                              ((uint32_t)0x00000008) /* I/O port B clock enable */
#define RCC_IOPCEN                              ((uint32_t)0x00000010) /* I/O port C clock enable */
#define RCC_ADC1EN                              ((uint32_t)0x00000200) /* ADC 1 interface clock enable */
#define RCC_TIM1EN                              ((uint32_t)0x00000800) /* TIM1 Timer clock enable */
#define RCC_SPI1EN                              ((uint32_t)0x00001000) /* SPI 1 clock enable */
#define RCC_USART1EN                            ((uint32_t)0x00004000) /* USART1 clock enable */

/*****************  Bit definition for RCC_APB1PCENR register  ******************/
#define RCC_TIM2EN                              ((uint32_t)0x00000001) /* Timer 2 clock enabled*/
#define RCC_TIM3EN                              ((uint32_t)0x00000002) /* Timer 3 clock enable */
#define RCC_WWDGEN                              ((uint32_t)0x00000800) /* Window Watchdog clock enable */
#define RCC_USART2EN                            ((uint32_t)0x00020000) /* USART 2 clock enable */
#define RCC_USART3EN                            ((uint32_t)0x00040000) /* USART 3 clock enable */
#define RCC_USART4EN                            ((uint32_t)0x00080000) /* USART 4 clock enable */
#define RCC_I2C1EN                              ((uint32_t)0x00200000) /* I2C 1 clock enable */
#define RCC_PWREN                               ((uint32_t)0x10000000) /* Power interface clock enable */

/*******************  Bit definition for RCC_RSTSCKR register  ********************/
#define RCC_RMVF                                ((uint32_t)0x01000000) /* Remove reset flag */
#define RCC_OPARSTF                             ((uint32_t)0x02000000) /* OPA reset flag */
#define RCC_PINRSTF                             ((uint32_t)0x04000000) /* PIN reset flag */
#define RCC_PORRSTF                             ((uint32_t)0x08000000) /* POR/PDR reset flag */
#define RCC_SFTRSTF                             ((uint32_t)0x10000000) /* Software Reset flag */
#define RCC_IWDGRSTF                            ((uint32_t)0x20000000) /* Independent Watchdog reset flag */
#define RCC_WWDGRSTF                            ((uint32_t)0x40000000) /* Window watchdog reset flag */
#define RCC_LPWRRSTF                            ((uint32_t)0x80000000) /* Low-Power reset flag */

/*******************  Bit definition for RCC_AHBRSTR register  ********************/
#define RCC_USBFSRST                            ((uint32_t)0x00001000) /* USBFS reset */
#define RCC_PIOCRST                             ((uint32_t)0x00002000) /* PIOC RST */
#define RCC_USBPDRST                            ((uint32_t)0x00020000) /* USBPD reset */

/******************************************************************************/
/*                        Serial Peripheral Interface                         */
/******************************************************************************/

/*******************  Bit definition for SPI_CTLR1 register  ********************/
#define SPI_CTLR1_CPHA                          ((uint16_t)0x0001) /* Clock Phase */
#define SPI_CTLR1_CPOL                          ((uint16_t)0x0002) /* Clock Polarity */
#define SPI_CTLR1_MSTR                          ((uint16_t)0x0004) /* Master Selection */

#define SPI_CTLR1_BR                            ((uint16_t)0x0038) /* BR[2:0] bits (Baud Rate Control) */
#define SPI_CTLR1_BR_0                          ((uint16_t)0x0008) /* Bit 0 */
#define SPI_CTLR1_BR_1                          ((uint16_t)0x0010) /* Bit 1 */
#define SPI_CTLR1_BR_2                          ((uint16_t)0x0020) /* Bit 2 */

#define SPI_CTLR1_SPE                           ((uint16_t)0x0040) /* SPI Enable */
#define SPI_CTLR1_LSBFIRST                      ((uint16_t)0x0080) /* Frame Format */
#define SPI_CTLR1_SSI                           ((uint16_t)0x0100) /* Internal slave select */
#define SPI_CTLR1_SSM                           ((uint16_t)0x0200) /* Software slave management */
#define SPI_CTLR1_RXONLY                        ((uint16_t)0x0400) /* Receive only */
#define SPI_CTLR1_DFF                           ((uint16_t)0x0800) /* Data Frame Format */
#define SPI_CTLR1_CRCNEXT                       ((uint16_t)0x1000) /* Transmit CRC next */
#define SPI_CTLR1_CRCEN                         ((uint16_t)0x2000) /* Hardware CRC calculation enable */
#define SPI_CTLR1_BIDIOE                        ((uint16_t)0x4000) /* Output enable in bidirectional mode */
#define SPI_CTLR1_BIDIMODE                      ((uint16_t)0x8000) /* Bidirectional data mode enable */

/*******************  Bit definition for SPI_CTLR2 register  ********************/
#define SPI_CTLR2_RXDMAEN                       ((uint8_t)0x01) /* Rx Buffer DMA Enable */
#define SPI_CTLR2_TXDMAEN                       ((uint8_t)0x02) /* Tx Buffer DMA Enable */
#define SPI_CTLR2_SSOE                          ((uint8_t)0x04) /* SS Output Enable */
#define SPI_CTLR2_ERRIE                         ((uint8_t)0x20) /* Error Interrupt Enable */
#define SPI_CTLR2_RXNEIE                        ((uint8_t)0x40) /* RX buffer Not Empty Interrupt Enable */
#define SPI_CTLR2_TXEIE                         ((uint8_t)0x80) /* Tx buffer Empty Interrupt Enable */
#define SPI_CTLR2_ODEN                          ((uint16_t)0x8000) /* SPI OD output Enable */

/********************  Bit definition for SPI_STATR register  ********************/
#define SPI_STATR_RXNE                          ((uint8_t)0x01) /* Receive buffer Not Empty */
#define SPI_STATR_TXE                           ((uint8_t)0x02) /* Transmit buffer Empty */
#define SPI_STATR_CHSIDE                        ((uint8_t)0x04) /* Channel side */
#define SPI_STATR_UDR                           ((uint8_t)0x08) /* Underrun flag */
#define SPI_STATR_CRCERR                        ((uint8_t)0x10) /* CRC Error flag */
#define SPI_STATR_MODF                          ((uint8_t)0x20) /* Mode fault */
#define SPI_STATR_OVR                           ((uint8_t)0x40) /* Overrun flag */
#define SPI_STATR_BSY                           ((uint8_t)0x80) /* Busy flag */

/********************  Bit definition for SPI_DATAR register  ********************/
#define SPI_DATAR_DR                            ((uint16_t)0xFFFF) /* Data Register */

/*******************  Bit definition for SPI_CRCR register  ******************/
#define SPI_CRCR_CRCPOLY                        ((uint16_t)0xFFFF) /* CRC polynomial register */

/******************  Bit definition for SPI_RCRCR register  ******************/
#define SPI_RCRCR_RXCRC                         ((uint16_t)0xFFFF) /* Rx CRC Register */

/******************  Bit definition for SPI_TCRCR register  ******************/
#define SPI_TCRCR_TXCRC                         ((uint16_t)0xFFFF) /* Tx CRC Register */

/******************  Bit definition for SPI_HSCR register  *****************/
#define SPI_HSCR_HSRXEN                         ((uint16_t)0x0001) /* Read Enable under SPI High speed mode */

/******************************************************************************/
/*                                    TIM                                     */
/******************************************************************************/

/*******************  Bit definition for TIM_CTLR1 register  ********************/
#define TIM_CEN                                 ((uint16_t)0x0001) /* Counter enable */
#define TIM_UDIS                                ((uint16_t)0x0002) /* Update disable */
#define TIM_URS                                 ((uint16_t)0x0004) /* Update request source */
#define TIM_OPM                                 ((uint16_t)0x0008) /* One pulse mode */
#define TIM_DIR                                 ((uint16_t)0x0010) /* Direction */

#define TIM_CMS                                 ((uint16_t)0x0060) /* CMS[1:0] bits (Center-aligned mode selection) */
#define TIM_CMS_0                               ((uint16_t)0x0020) /* Bit 0 */
#define TIM_CMS_1                               ((uint16_t)0x0040) /* Bit 1 */

#define TIM_ARPE                                ((uint16_t)0x0080) /* Auto-reload preload enable */

#define TIM_CTLR1_CKD                           ((uint16_t)0x0300) /* CKD[1:0] bits (clock division) */
#define TIM_CKD_0                               ((uint16_t)0x0100) /* Bit 0 */
#define TIM_CKD_1                               ((uint16_t)0x0200) /* Bit 1 */

#define TIM_CMP_BK                              ((uint16_t)0x1000) /* voltage comparator break enable, TIM1 only */
#define TIM_CAPOV                               ((uint16_t)0x4000) /* Cfg mode of capture value */
#define TIM_CAPLVL                              ((uint16_t)0x8000) 

/*******************  Bit definition for TIM_CTLR2 register  ********************/
#define TIM_CCPC                                ((uint16_t)0x0001) /* Capture/Compare Preloaded Control */
#define TIM_CCUS                                ((uint16_t)0x0004) /* Capture/Compare Control Update Selection */
#define TIM_CCDS                                ((uint16_t)0x0008) /* Capture/Compare DMA Selection */

#define TIM_MMS                                 ((uint16_t)0x0070) /* MMS[2:0] bits (Master Mode Selection) */
#define TIM_MMS_0                               ((uint16_t)0x0010) /* Bit 0 */
#define TIM_MMS_1                               ((uint16_t)0x0020) /* Bit 1 */
#define TIM_MMS_2                               ((uint16_t)0x0040) /* Bit 2 */

#define TIM_TI1S                                ((uint16_t)0x0080) /* TI1 Selection */
#define TIM_OIS1                                ((uint16_t)0x0100) /* Output Idle state 1 (OC1 output) */
#define TIM_OIS1N                               ((uint16_t)0x0200) /* Output Idle state 1 (OC1N output) */
#define TIM_OIS2                                ((uint16_t)0x0400) /* Output Idle state 2 (OC2 output) */
#define TIM_OIS2N                               ((uint16_t)0x0800) /* Output Idle state 2 (OC2N output) */
#define TIM_OIS3                                ((uint16_t)0x1000) /* Output Idle state 3 (OC3 output) */
#define TIM_OIS3N                               ((uint16_t)0x2000) /* Output Idle state 3 (OC3N output) */
#define TIM_OIS4                                ((uint16_t)0x4000) /* Output Idle state 4 (OC4 output) */

/*******************  Bit definition for TIM_SMCFGR register  *******************/
#define TIM_SMS                                 ((uint16_t)0x0007) /* SMS[2:0] bits (Slave mode selection) */
#define TIM_SMS_0                               ((uint16_t)0x0001) /* Bit 0 */
#define TIM_SMS_1                               ((uint16_t)0x0002) /* Bit 1 */
#define TIM_SMS_2                               ((uint16_t)0x0004) /* Bit 2 */

#define TIM_TS                                  ((uint16_t)0x0070) /* TS[2:0] bits (Trigger selection) */
#define TIM_TS_0                                ((uint16_t)0x0010) /* Bit 0 */
#define TIM_TS_1                                ((uint16_t)0x0020) /* Bit 1 */
#define TIM_TS_2                                ((uint16_t)0x0040) /* Bit 2 */

#define TIM_MSM                                 ((uint16_t)0x0080) /* Master/slave mode */

#define TIM_ETF                                 ((uint16_t)0x0F00) /* ETF[3:0] bits (External trigger filter) */
#define TIM_ETF_0                               ((uint16_t)0x0100) /* Bit 0 */
#define TIM_ETF_1                               ((uint16_t)0x0200) /* Bit 1 */
#define TIM_ETF_2                               ((uint16_t)0x0400) /* Bit 2 */
#define TIM_ETF_3                               ((uint16_t)0x0800) /* Bit 3 */

#define TIM_ETPS                                ((uint16_t)0x3000) /* ETPS[1:0] bits (External trigger prescaler) */
#define TIM_ETPS_0                              ((uint16_t)0x1000) /* Bit 0 */
#define TIM_ETPS_1                              ((uint16_t)0x2000) /* Bit 1 */

#define TIM_ECE                                 ((uint16_t)0x4000) /* External clock enable */
#define TIM_ETP                                 ((uint16_t)0x8000) /* External trigger polarity */

/*******************  Bit definition for TIM_DMAINTENR register  *******************/
#define TIM_UIE                                 ((uint16_t)0x0001) /* Update interrupt enable */
#define TIM_CC1IE                               ((uint16_t)0x0002) /* Capture/Compare 1 interrupt enable */
#define TIM_CC2IE                               ((uint16_t)0x0004) /* Capture/Compare 2 interrupt enable */
#define TIM_CC3IE                               ((uint16_t)0x0008) /* Capture/Compare 3 interrupt enable */
#define TIM_CC4IE                               ((uint16_t)0x0010) /* Capture/Compare 4 interrupt enable */
#define TIM_COMIE                               ((uint16_t)0x0020) /* COM interrupt enable */
#define TIM_TIE                                 ((uint16_t)0x0040) /* Trigger interrupt enable */
#define TIM_BIE                                 ((uint16_t)0x0080) /* Break interrupt enable */
#define TIM_UDE                                 ((uint16_t)0x0100) /* Update DMA request enable */
#define TIM_CC1DE                               ((uint16_t)0x0200) /* Capture/Compare 1 DMA request enable */
#define TIM_CC2DE                               ((uint16_t)0x0400) /* Capture/Compare 2 DMA request enable */
#define TIM_CC3DE                               ((uint16_t)0x0800) /* Capture/Compare 3 DMA request enable */
#define TIM_CC4DE                               ((uint16_t)0x1000) /* Capture/Compare 4 DMA request enable */
#define TIM_COMDE                               ((uint16_t)0x2000) /* COM DMA request enable */
#define TIM_TDE                                 ((uint16_t)0x4000) /* Trigger DMA request enable */

/********************  Bit definition for TIM_INTFR register  ********************/
#define TIM_UIF                                 ((uint16_t)0x0001) /* Update interrupt Flag */
#define TIM_CC1IF                               ((uint16_t)0x0002) /* Capture/Compare 1 interrupt Flag */
#define TIM_CC2IF                               ((uint16_t)0x0004) /* Capture/Compare 2 interrupt Flag */
#define TIM_CC3IF                               ((uint16_t)0x0008) /* Capture/Compare 3 interrupt Flag */
#define TIM_CC4IF                               ((uint16_t)0x0010) /* Capture/Compare 4 interrupt Flag */
#define TIM_COMIF                               ((uint16_t)0x0020) /* COM interrupt Flag */
#define TIM_TIF                                 ((uint16_t)0x0040) /* Trigger interrupt Flag */
#define TIM_BIF                                 ((uint16_t)0x0080) /* Break interrupt Flag */
#define TIM_CC1OF                               ((uint16_t)0x0200) /* Capture/Compare 1 Overcapture Flag */
#define TIM_CC2OF                               ((uint16_t)0x0400) /* Capture/Compare 2 Overcapture Flag */
#define TIM_CC3OF                               ((uint16_t)0x0800) /* Capture/Compare 3 Overcapture Flag */
#define TIM_CC4OF                               ((uint16_t)0x1000) /* Capture/Compare 4 Overcapture Flag */

/*******************  Bit definition for TIM_SWEVGR register  ********************/
#define TIM_UG                                  ((uint8_t)0x01) /* Update Generation */
#define TIM_CC1G                                ((uint8_t)0x02) /* Capture/Compare 1 Generation */
#define TIM_CC2G                                ((uint8_t)0x04) /* Capture/Compare 2 Generation */
#define TIM_CC3G                                ((uint8_t)0x08) /* Capture/Compare 3 Generation */
#define TIM_CC4G                                ((uint8_t)0x10) /* Capture/Compare 4 Generation */
#define TIM_COMG                                ((uint8_t)0x20) /* Capture/Compare Control Update Generation */
#define TIM_TG                                  ((uint8_t)0x40) /* Trigger Generation */
#define TIM_BG                                  ((uint8_t)0x80) /* Break Generation */

/******************  Bit definition for TIM_CHCTLR1 register  *******************/
#define TIM_CC1S                                ((uint16_t)0x0003) /* CC1S[1:0] bits (Capture/Compare 1 Selection) */
#define TIM_CC1S_0                              ((uint16_t)0x0001) /* Bit 0 */
#define TIM_CC1S_1                              ((uint16_t)0x0002) /* Bit 1 */

#define TIM_OC1FE                               ((uint16_t)0x0004) /* Output Compare 1 Fast enable */
#define TIM_OC1PE                               ((uint16_t)0x0008) /* Output Compare 1 Preload enable */

#define TIM_OC1M                                ((uint16_t)0x0070) /* OC1M[2:0] bits (Output Compare 1 Mode) */
#define TIM_OC1M_0                              ((uint16_t)0x0010) /* Bit 0 */
#define TIM_OC1M_1                              ((uint16_t)0x0020) /* Bit 1 */
#define TIM_OC1M_2                              ((uint16_t)0x0040) /* Bit 2 */

#define TIM_OC1CE                               ((uint16_t)0x0080) /* Output Compare 1Clear Enable */

#define TIM_CC2S                                ((uint16_t)0x0300) /* CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define TIM_CC2S_0                              ((uint16_t)0x0100) /* Bit 0 */
#define TIM_CC2S_1                              ((uint16_t)0x0200) /* Bit 1 */

#define TIM_OC2FE                               ((uint16_t)0x0400) /* Output Compare 2 Fast enable */
#define TIM_OC2PE                               ((uint16_t)0x0800) /* Output Compare 2 Preload enable */

#define TIM_OC2M                                ((uint16_t)0x7000) /* OC2M[2:0] bits (Output Compare 2 Mode) */
#define TIM_OC2M_0                              ((uint16_t)0x1000) /* Bit 0 */
#define TIM_OC2M_1                              ((uint16_t)0x2000) /* Bit 1 */
#define TIM_OC2M_2                              ((uint16_t)0x4000) /* Bit 2 */

#define TIM_OC2CE                               ((uint16_t)0x8000) /* Output Compare 2 Clear Enable */

#define TIM_IC1PSC                              ((uint16_t)0x000C) /* IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
#define TIM_IC1PSC_0                            ((uint16_t)0x0004) /* Bit 0 */
#define TIM_IC1PSC_1                            ((uint16_t)0x0008) /* Bit 1 */

#define TIM_IC1F                                ((uint16_t)0x00F0) /* IC1F[3:0] bits (Input Capture 1 Filter) */
#define TIM_IC1F_0                              ((uint16_t)0x0010) /* Bit 0 */
#define TIM_IC1F_1                              ((uint16_t)0x0020) /* Bit 1 */
#define TIM_IC1F_2                              ((uint16_t)0x0040) /* Bit 2 */
#define TIM_IC1F_3                              ((uint16_t)0x0080) /* Bit 3 */

#define TIM_IC2PSC                              ((uint16_t)0x0C00) /* IC2PSC[1:0] bits (Input Capture 2 Prescaler) */
#define TIM_IC2PSC_0                            ((uint16_t)0x0400) /* Bit 0 */
#define TIM_IC2PSC_1                            ((uint16_t)0x0800) /* Bit 1 */

#define TIM_IC2F                                ((uint16_t)0xF000) /* IC2F[3:0] bits (Input Capture 2 Filter) */
#define TIM_IC2F_0                              ((uint16_t)0x1000) /* Bit 0 */
#define TIM_IC2F_1                              ((uint16_t)0x2000) /* Bit 1 */
#define TIM_IC2F_2                              ((uint16_t)0x4000) /* Bit 2 */
#define TIM_IC2F_3                              ((uint16_t)0x8000) /* Bit 3 */

/******************  Bit definition for TIM_CHCTLR2 register  *******************/
#define TIM_CC3S                                ((uint16_t)0x0003) /* CC3S[1:0] bits (Capture/Compare 3 Selection) */
#define TIM_CC3S_0                              ((uint16_t)0x0001) /* Bit 0 */
#define TIM_CC3S_1                              ((uint16_t)0x0002) /* Bit 1 */

#define TIM_OC3FE                               ((uint16_t)0x0004) /* Output Compare 3 Fast enable */
#define TIM_OC3PE                               ((uint16_t)0x0008) /* Output Compare 3 Preload enable */

#define TIM_OC3M                                ((uint16_t)0x0070) /* OC3M[2:0] bits (Output Compare 3 Mode) */
#define TIM_OC3M_0                              ((uint16_t)0x0010) /* Bit 0 */
#define TIM_OC3M_1                              ((uint16_t)0x0020) /* Bit 1 */
#define TIM_OC3M_2                              ((uint16_t)0x0040) /* Bit 2 */

#define TIM_OC3CE                               ((uint16_t)0x0080) /* Output Compare 3 Clear Enable */

#define TIM_CC4S                                ((uint16_t)0x0300) /* CC4S[1:0] bits (Capture/Compare 4 Selection) */
#define TIM_CC4S_0                              ((uint16_t)0x0100) /* Bit 0 */
#define TIM_CC4S_1                              ((uint16_t)0x0200) /* Bit 1 */

#define TIM_OC4FE                               ((uint16_t)0x0400) /* Output Compare 4 Fast enable */
#define TIM_OC4PE                               ((uint16_t)0x0800) /* Output Compare 4 Preload enable */

#define TIM_OC4M                                ((uint16_t)0x7000) /* OC4M[2:0] bits (Output Compare 4 Mode) */
#define TIM_OC4M_0                              ((uint16_t)0x1000) /* Bit 0 */
#define TIM_OC4M_1                              ((uint16_t)0x2000) /* Bit 1 */
#define TIM_OC4M_2                              ((uint16_t)0x4000) /* Bit 2 */

#define TIM_OC4CE                               ((uint16_t)0x8000) /* Output Compare 4 Clear Enable */

#define TIM_IC3PSC                              ((uint16_t)0x000C) /* IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
#define TIM_IC3PSC_0                            ((uint16_t)0x0004) /* Bit 0 */
#define TIM_IC3PSC_1                            ((uint16_t)0x0008) /* Bit 1 */

#define TIM_IC3F                                ((uint16_t)0x00F0) /* IC3F[3:0] bits (Input Capture 3 Filter) */
#define TIM_IC3F_0                              ((uint16_t)0x0010) /* Bit 0 */
#define TIM_IC3F_1                              ((uint16_t)0x0020) /* Bit 1 */
#define TIM_IC3F_2                              ((uint16_t)0x0040) /* Bit 2 */
#define TIM_IC3F_3                              ((uint16_t)0x0080) /* Bit 3 */

#define TIM_IC4PSC                              ((uint16_t)0x0C00) /* IC4PSC[1:0] bits (Input Capture 4 Prescaler) */
#define TIM_IC4PSC_0                            ((uint16_t)0x0400) /* Bit 0 */
#define TIM_IC4PSC_1                            ((uint16_t)0x0800) /* Bit 1 */

#define TIM_IC4F                                ((uint16_t)0xF000) /* IC4F[3:0] bits (Input Capture 4 Filter) */
#define TIM_IC4F_0                              ((uint16_t)0x1000) /* Bit 0 */
#define TIM_IC4F_1                              ((uint16_t)0x2000) /* Bit 1 */
#define TIM_IC4F_2                              ((uint16_t)0x4000) /* Bit 2 */
#define TIM_IC4F_3                              ((uint16_t)0x8000) /* Bit 3 */

/*******************  Bit definition for TIM_CCER register  *******************/
#define TIM_CC1E                                ((uint16_t)0x0001) /* Capture/Compare 1 output enable */
#define TIM_CC1P                                ((uint16_t)0x0002) /* Capture/Compare 1 output Polarity */
#define TIM_CC1NE                               ((uint16_t)0x0004) /* Capture/Compare 1 Complementary output enable */
#define TIM_CC1NP                               ((uint16_t)0x0008) /* Capture/Compare 1 Complementary output Polarity */
#define TIM_CC2E                                ((uint16_t)0x0010) /* Capture/Compare 2 output enable */
#define TIM_CC2P                                ((uint16_t)0x0020) /* Capture/Compare 2 output Polarity */
#define TIM_CC2NE                               ((uint16_t)0x0040) /* Capture/Compare 2 Complementary output enable */
#define TIM_CC2NP                               ((uint16_t)0x0080) /* Capture/Compare 2 Complementary output Polarity */
#define TIM_CC3E                                ((uint16_t)0x0100) /* Capture/Compare 3 output enable */
#define TIM_CC3P                                ((uint16_t)0x0200) /* Capture/Compare 3 output Polarity */
#define TIM_CC3NE                               ((uint16_t)0x0400) /* Capture/Compare 3 Complementary output enable */
#define TIM_CC3NP                               ((uint16_t)0x0800) /* Capture/Compare 3 Complementary output Polarity */
#define TIM_CC4E                                ((uint16_t)0x1000) /* Capture/Compare 4 output enable */
#define TIM_CC4P                                ((uint16_t)0x2000) /* Capture/Compare 4 output Polarity */

/*******************  Bit definition for TIM_CNT register  ********************/
#define TIM_CNT                                 ((uint16_t)0xFFFF) /* Counter Value */

/*******************  Bit definition for TIM_PSC register  ********************/
#define TIM_PSC                                 ((uint16_t)0xFFFF) /* Prescaler Value */

/*******************  Bit definition for TIM_ATRLR register  ********************/
#define TIM_ARR                                 ((uint16_t)0xFFFF) /* actual auto-reload Value */

/*******************  Bit definition for TIM_RPTCR register  ********************/
#define TIM_REP                                 ((uint8_t)0xFF) /* Repetition Counter Value */

/*******************  Bit definition for TIM_CH1CVR register  *******************/
#define TIM_CCR1                                ((uint16_t)0xFFFF) /* Capture/Compare 1 Value */
#define TIM_LEVEL1                              ((uint32_t)0x00010000)

/*******************  Bit definition for TIM_CH2CVR register  *******************/
#define TIM_CCR2                                ((uint16_t)0xFFFF) /* Capture/Compare 2 Value */
#define TIM_LEVEL2                              ((uint32_t)0x00010000)

/*******************  Bit definition for TIM_CH3CVR register  *******************/
#define TIM_CCR3                                ((uint16_t)0xFFFF) /* Capture/Compare 3 Value */
#define TIM_LEVEL3                              ((uint32_t)0x00010000)

/*******************  Bit definition for TIM_CH4CVR register  *******************/
#define TIM_CCR4                                ((uint16_t)0xFFFF) /* Capture/Compare 4 Value */
#define TIM_LEVEL4                              ((uint32_t)0x00010000)

/*******************  Bit definition for TIM_BDTR register  *******************/
#define TIM_DTG                                 ((uint16_t)0x00FF) /* DTG[0:7] bits (Dead-Time Generator set-up) */
#define TIM_DTG_0                               ((uint16_t)0x0001) /* Bit 0 */
#define TIM_DTG_1                               ((uint16_t)0x0002) /* Bit 1 */
#define TIM_DTG_2                               ((uint16_t)0x0004) /* Bit 2 */
#define TIM_DTG_3                               ((uint16_t)0x0008) /* Bit 3 */
#define TIM_DTG_4                               ((uint16_t)0x0010) /* Bit 4 */
#define TIM_DTG_5                               ((uint16_t)0x0020) /* Bit 5 */
#define TIM_DTG_6                               ((uint16_t)0x0040) /* Bit 6 */
#define TIM_DTG_7                               ((uint16_t)0x0080) /* Bit 7 */

#define TIM_LOCK                                ((uint16_t)0x0300) /* LOCK[1:0] bits (Lock Configuration) */
#define TIM_LOCK_0                              ((uint16_t)0x0100) /* Bit 0 */
#define TIM_LOCK_1                              ((uint16_t)0x0200) /* Bit 1 */

#define TIM_OSSI                                ((uint16_t)0x0400) /* Off-State Selection for Idle mode */
#define TIM_OSSR                                ((uint16_t)0x0800) /* Off-State Selection for Run mode */
#define TIM_BKE                                 ((uint16_t)0x1000) /* Break enable */
#define TIM_BKP                                 ((uint16_t)0x2000) /* Break Polarity */
#define TIM_AOE                                 ((uint16_t)0x4000) /* Automatic Output enable */
#define TIM_MOE                                 ((uint16_t)0x8000) /* Main Output enable */

/*******************  Bit definition for TIM_DMACFGR register  ********************/
#define TIM_DBA                                 ((uint16_t)0x001F) /* DBA[4:0] bits (DMA Base Address) */
#define TIM_DBA_0                               ((uint16_t)0x0001) /* Bit 0 */
#define TIM_DBA_1                               ((uint16_t)0x0002) /* Bit 1 */
#define TIM_DBA_2                               ((uint16_t)0x0004) /* Bit 2 */
#define TIM_DBA_3                               ((uint16_t)0x0008) /* Bit 3 */
#define TIM_DBA_4                               ((uint16_t)0x0010) /* Bit 4 */

#define TIM_DBL                                 ((uint16_t)0x1F00) /* DBL[4:0] bits (DMA Burst Length) */
#define TIM_DBL_0                               ((uint16_t)0x0100) /* Bit 0 */
#define TIM_DBL_1                               ((uint16_t)0x0200) /* Bit 1 */
#define TIM_DBL_2                               ((uint16_t)0x0400) /* Bit 2 */
#define TIM_DBL_3                               ((uint16_t)0x0800) /* Bit 3 */
#define TIM_DBL_4                               ((uint16_t)0x1000) /* Bit 4 */

/*******************  Bit definition for TIM_DMAADR register  *******************/
#define TIM_DMAR_DMAB                           ((uint16_t)0xFFFF) /* DMA register for burst accesses */

/*******************  Bit definition for TIM_SPEC register  *******************/
#define TIM_SPEC_PWM_EN_1_2                     ((uint16_t)0x0001) /* Channel 1 and Channel 2 alternate */
#define TIM_SPEC_PWM_EN_3_4                     ((uint16_t)0x0002) /* Channel 3 and Channel 4 alternate */
#define TIM_SPEC_PWM_OC1                        ((uint16_t)0x0010) /* Channel 1 invalid level under alternate mode */
#define TIM_SPEC_PWM_OC2                        ((uint16_t)0x0020) /* Channel 2 invalid level under alternate mode */
#define TIM_SPEC_PWM_OC3                        ((uint16_t)0x0040) /* Channel 3 invalid level under alternate mode */
#define TIM_SPEC_PWM_OC4                        ((uint16_t)0x0080) /* Channel 4 invalid level under alternate mode */
#define TIM_SPEC_TOGGLE                         ((uint16_t)0x8000) /* valid channel indicator */

/******************************************************************************/
/*         Universal Synchronous Asynchronous Receiver Transmitter            */
/******************************************************************************/

/*******************  Bit definition for USART_STATR register  *******************/
#define USART_STATR_PE                          ((uint16_t)0x0001) /* Parity Error */
#define USART_STATR_FE                          ((uint16_t)0x0002) /* Framing Error */
#define USART_STATR_NE                          ((uint16_t)0x0004) /* Noise Error Flag */
#define USART_STATR_ORE                         ((uint16_t)0x0008) /* OverRun Error */
#define USART_STATR_IDLE                        ((uint16_t)0x0010) /* IDLE line detected */
#define USART_STATR_RXNE                        ((uint16_t)0x0020) /* Read Data Register Not Empty */
#define USART_STATR_TC                          ((uint16_t)0x0040) /* Transmission Complete */
#define USART_STATR_TXE                         ((uint16_t)0x0080) /* Transmit Data Register Empty */
#define USART_STATR_LBD                         ((uint16_t)0x0100) /* LIN Break Detection Flag */
#define USART_STATR_CTS                         ((uint16_t)0x0200) /* CTS Flag */

/*******************  Bit definition for USART_DATAR register  *******************/
#define USART_DATAR_DR                          ((uint16_t)0x01FF) /* Data value */

/******************  Bit definition for USART_BRR register  *******************/
#define USART_BRR_DIV_Fraction                  ((uint16_t)0x000F) /* Fraction of USARTDIV */
#define USART_BRR_DIV_Mantissa                  ((uint16_t)0xFFF0) /* Mantissa of USARTDIV */

/******************  Bit definition for USART_CTLR1 register  *******************/
#define USART_CTLR1_SBK                         ((uint16_t)0x0001) /* Send Break */
#define USART_CTLR1_RWU                         ((uint16_t)0x0002) /* Receiver wakeup */
#define USART_CTLR1_RE                          ((uint16_t)0x0004) /* Receiver Enable */
#define USART_CTLR1_TE                          ((uint16_t)0x0008) /* Transmitter Enable */
#define USART_CTLR1_IDLEIE                      ((uint16_t)0x0010) /* IDLE Interrupt Enable */
#define USART_CTLR1_RXNEIE                      ((uint16_t)0x0020) /* RXNE Interrupt Enable */
#define USART_CTLR1_TCIE                        ((uint16_t)0x0040) /* Transmission Complete Interrupt Enable */
#define USART_CTLR1_TXEIE                       ((uint16_t)0x0080) /* PE Interrupt Enable */
#define USART_CTLR1_PEIE                        ((uint16_t)0x0100) /* PE Interrupt Enable */
#define USART_CTLR1_PS                          ((uint16_t)0x0200) /* Parity Selection */
#define USART_CTLR1_PCE                         ((uint16_t)0x0400) /* Parity Control Enable */
#define USART_CTLR1_WAKE                        ((uint16_t)0x0800) /* Wakeup method */
#define USART_CTLR1_M                           ((uint16_t)0x1000) /* Word length */
#define USART_CTLR1_UE                          ((uint16_t)0x2000) /* USART Enable */

/******************  Bit definition for USART_CTLR2 register  *******************/
#define USART_CTLR2_ADD                         ((uint16_t)0x000F) /* Address of the USART node */
#define USART_CTLR2_LBDL                        ((uint16_t)0x0020) /* LIN Break Detection Length */
#define USART_CTLR2_LBDIE                       ((uint16_t)0x0040) /* LIN Break Detection Interrupt Enable */
#define USART_CTLR2_LBCL                        ((uint16_t)0x0100) /* Last Bit Clock pulse */
#define USART_CTLR2_CPHA                        ((uint16_t)0x0200) /* Clock Phase */
#define USART_CTLR2_CPOL                        ((uint16_t)0x0400) /* Clock Polarity */
#define USART_CTLR2_CLKEN                       ((uint16_t)0x0800) /* Clock Enable */

#define USART_CTLR2_STOP                        ((uint16_t)0x3000) /* STOP[1:0] bits (STOP bits) */
#define USART_CTLR2_STOP_0                      ((uint16_t)0x1000) /* Bit 0 */
#define USART_CTLR2_STOP_1                      ((uint16_t)0x2000) /* Bit 1 */

#define USART_CTLR2_LINEN                       ((uint16_t)0x4000) /* LIN mode enable */

/******************  Bit definition for USART_CTLR3 register  *******************/
#define USART_CTLR3_EIE                         ((uint16_t)0x0001) /* Error Interrupt Enable */
#define USART_CTLR3_IREN                        ((uint16_t)0x0002) /* IrDA mode Enable */
#define USART_CTLR3_IRLP                        ((uint16_t)0x0004) /* IrDA Low-Power */
#define USART_CTLR3_HDSEL                       ((uint16_t)0x0008) /* Half-Duplex Selection */
#define USART_CTLR3_NACK                        ((uint16_t)0x0010) /* Smartcard NACK enable */
#define USART_CTLR3_SCEN                        ((uint16_t)0x0020) /* Smartcard mode enable */
#define USART_CTLR3_DMAR                        ((uint16_t)0x0040) /* DMA Enable Receiver */
#define USART_CTLR3_DMAT                        ((uint16_t)0x0080) /* DMA Enable Transmitter */
#define USART_CTLR3_RTSE                        ((uint16_t)0x0100) /* RTS Enable */
#define USART_CTLR3_CTSE                        ((uint16_t)0x0200) /* CTS Enable */
#define USART_CTLR3_CTSIE                       ((uint16_t)0x0400) /* CTS Interrupt Enable */

/******************  Bit definition for USART_GPR register  ******************/
#define USART_GPR_PSC                           ((uint16_t)0x00FF) /* PSC[7:0] bits (Prescaler value) */
#define USART_GPR_PSC_0                         ((uint16_t)0x0001) /* Bit 0 */
#define USART_GPR_PSC_1                         ((uint16_t)0x0002) /* Bit 1 */
#define USART_GPR_PSC_2                         ((uint16_t)0x0004) /* Bit 2 */
#define USART_GPR_PSC_3                         ((uint16_t)0x0008) /* Bit 3 */
#define USART_GPR_PSC_4                         ((uint16_t)0x0010) /* Bit 4 */
#define USART_GPR_PSC_5                         ((uint16_t)0x0020) /* Bit 5 */
#define USART_GPR_PSC_6                         ((uint16_t)0x0040) /* Bit 6 */
#define USART_GPR_PSC_7                         ((uint16_t)0x0080) /* Bit 7 */

#define USART_GPR_GT                            ((uint16_t)0xFF00) /* Guard time value */

/******************************************************************************/
/*                            Window WATCHDOG                                 */
/******************************************************************************/

/*******************  Bit definition for WWDG_CTLR register  ********************/
#define WWDG_CTLR_T                             ((uint8_t)0x7F) /* T[6:0] bits (7-Bit counter (MSB to LSB)) */
#define WWDG_CTLR_T0                            ((uint8_t)0x01) /* Bit 0 */
#define WWDG_CTLR_T1                            ((uint8_t)0x02) /* Bit 1 */
#define WWDG_CTLR_T2                            ((uint8_t)0x04) /* Bit 2 */
#define WWDG_CTLR_T3                            ((uint8_t)0x08) /* Bit 3 */
#define WWDG_CTLR_T4                            ((uint8_t)0x10) /* Bit 4 */
#define WWDG_CTLR_T5                            ((uint8_t)0x20) /* Bit 5 */
#define WWDG_CTLR_T6                            ((uint8_t)0x40) /* Bit 6 */

#define WWDG_CTLR_WDGA                          ((uint8_t)0x80) /* Activation bit */

/*******************  Bit definition for WWDG_CFGR register  *******************/
#define WWDG_CFGR_W                             ((uint16_t)0x007F) /* W[6:0] bits (7-bit window value) */
#define WWDG_CFGR_W0                            ((uint16_t)0x0001) /* Bit 0 */
#define WWDG_CFGR_W1                            ((uint16_t)0x0002) /* Bit 1 */
#define WWDG_CFGR_W2                            ((uint16_t)0x0004) /* Bit 2 */
#define WWDG_CFGR_W3                            ((uint16_t)0x0008) /* Bit 3 */
#define WWDG_CFGR_W4                            ((uint16_t)0x0010) /* Bit 4 */
#define WWDG_CFGR_W5                            ((uint16_t)0x0020) /* Bit 5 */
#define WWDG_CFGR_W6                            ((uint16_t)0x0040) /* Bit 6 */

#define WWDG_CFGR_WDGTB                         ((uint16_t)0x0180) /* WDGTB[1:0] bits (Timer Base) */
#define WWDG_CFGR_WDGTB0                        ((uint16_t)0x0080) /* Bit 0 */
#define WWDG_CFGR_WDGTB1                        ((uint16_t)0x0100) /* Bit 1 */

#define WWDG_CFGR_EWI                           ((uint16_t)0x0200) /* Early Wakeup Interrupt */

/*******************  Bit definition for WWDG_STATR register  ********************/
#define WWDG_STATR_EWIF                         ((uint8_t)0x01) /* Early Wakeup Interrupt Flag */

/******************************************************************************/
/*                              OPA and CMP                                   */
/******************************************************************************/

/*******************  Bit definition for OPA_CFGR1 register  ********************/
#define OPA_CFGR1_POLL_EN1                      ((uint16_t)0x0001)
#define OPA_CFGR1_POLL_EN2                      ((uint16_t)0x0002)
#define OPA_CFGR1_BKIN_EN1                      ((uint16_t)0x0004)
#define OPA_CFGR1_BKIN_EN2                      ((uint16_t)0x0008)
#define OPA_CFGR1_RST_EN1                       ((uint16_t)0x0010)
#define OPA_CFGR1_RST_EN2                       ((uint16_t)0x0020)
#define OPA_CFGR1_BKIN_SEL                      ((uint16_t)0x0040)
#define OPA_CFGR1_POLL_LOCK                     ((uint16_t)0x0080)
#define OPA_CFGR1_IE_OUT1                       ((uint16_t)0x0100)
#define OPA_CFGR1_IE_OUT2                       ((uint16_t)0x0200)
#define OPA_CFGR1_IE_CNT                        ((uint16_t)0x0400)
#define OPA_CFGR1_NMI_EN                        ((uint16_t)0x0800)
#define OPA_CFGR1_IF_OUT1                       ((uint16_t)0x1000)
#define OPA_CFGR1_IF_OUT2                       ((uint16_t)0x2000)
#define OPA_CFGR1_IF_CNT                        ((uint16_t)0x4000)

/*******************  Bit definition for OPA_CFGR2 register  ********************/
#define OPA_CFGR2_POLL_VLU                      ((uint16_t)0x01FF)
#define OPA_CFGR2_POLL1_NUM                     ((uint16_t)0x0600)
#define OPA_CFGR2_POLL2_NUM                     ((uint16_t)0x1800)

/*******************  Bit definition for OPA_CTLR1 register  ********************/
#define OPA_CTLR1_EN1                           ((uint32_t)0x00000001)
#define OPA_CTLR1_MODE1                         ((uint32_t)0x00000002)

#define OPA_CTLR1_PSEL1                         ((uint32_t)0x00000018)

#define OPA_CTLR1_FB_EN1                        ((uint32_t)0x00000020)
#define OPA_CTLR1_NSEL1                         ((uint32_t)0x000001C0)

#define OPA_CTLR1_EN2                           ((uint32_t)0x00010000)
#define OPA_CTLR1_MODE2                         ((uint32_t)0x00020000)

#define OPA_CTLR1_PSEL2                         ((uint32_t)0x00180000)

#define OPA_CTLR1_FB_EN2                        ((uint32_t)0x00200000)
#define OPA_CTLR1_NSEL2                         ((uint32_t)0x01C00000)

#define OPA_CTLR1_OPA_LOCK                      ((uint32_t)0x80000000)

/*******************  Bit definition for OPA_CTLR2 register  ********************/
#define OPA_CTLR2_EN1                           ((uint32_t)0x00000001)
#define OPA_CTLR2_MODE1                         ((uint32_t)0x00000002)
#define OPA_CTLR2_NSEL1                         ((uint32_t)0x00000004)
#define OPA_CTLR2_PSEL1                         ((uint32_t)0x00000008)

#define OPA_CTLR2_EN2                           ((uint32_t)0x00000020)
#define OPA_CTLR2_MODE2                         ((uint32_t)0x00000040)
#define OPA_CTLR2_NSEL2                         ((uint32_t)0x00000080)
#define OPA_CTLR2_PSEL2                         ((uint32_t)0x00000100)

#define OPA_CTLR2_EN3                           ((uint32_t)0x00000400)
#define OPA_CTLR2_MODE3                         ((uint32_t)0x00000800)
#define OPA_CTLR2_NSEL3                         ((uint32_t)0x00001000)
#define OPA_CTLR2_PSEL3                         ((uint32_t)0x00002000)

#define OPA_CTLR2_CMP_LOCK                      ((uint32_t)0x00002000)

/*******************  Bit definition for OPA_KEY register  ********************/
#define OPA_KEY                                 ((uint32_t)0xFFFFFFFF)

/*******************  Bit definition for CMP_KEY register  ********************/
#define CMP_KEY                                 ((uint32_t)0xFFFFFFFF)

/*******************  Bit definition for POLL_KEY register  ********************/
#define POLL_KEY                                ((uint32_t)0xFFFFFFFF)

#include "ch32x035_conf.h"

#ifdef __cplusplus
}
#endif

#endif
