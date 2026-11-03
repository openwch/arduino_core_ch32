/********************************** (C) COPYRIGHT  *******************************
 * File Name          : core_riscv.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/07/25
 * Description        : CH585 Series RISC-V Core Peripheral Access Layer Header File
 *********************************************************************************
 * Copyright (c) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
#ifndef __CORE_RISCV_H__
#define __CORE_RISCV_H__

#ifdef __cplusplus
extern "C" {
#endif

/* define compiler specific symbols */
#if defined ( __CC_ARM   )
#define __ASM            __asm                                      /*!< asm keyword for ARM Compiler          */
#define __INLINE         __inline                                   /*!< inline keyword for ARM Compiler       */

#elif defined ( __ICCARM__ )
#define __ASM           __asm                                       /*!< asm keyword for IAR Compiler          */
#define __INLINE        inline                                      /*!< inline keyword for IAR Compiler. Only avaiable in High optimization mode! */

#elif defined   (  __GNUC__  )
#define __ASM            __asm                                      /*!< asm keyword for GNU Compiler          */
#define __INLINE         inline                                     /*!< inline keyword for GNU Compiler       */

#elif defined   (  __TASKING__  )
#define __ASM            __asm                                      /*!< asm keyword for TASKING Compiler      */
#define __INLINE         inline                                     /*!< inline keyword for TASKING Compiler   */

#endif

/* IO definitions */
#ifdef __cplusplus
#define __I                 volatile            /*!< defines 'read only' permissions      */
#else
#define __I                 volatile const      /*!< defines 'read only' permissions     */
#endif
#define __O                 volatile            /*!< defines 'write only' permissions     */
#define __IO                volatile            /*!< defines 'read / write' permissions   */
#define RV_STATIC_INLINE    static inline

//typedef enum {SUCCESS = 0, ERROR = !SUCCESS} ErrorStatus;

typedef enum
{
    DISABLE = 0,
    ENABLE = !DISABLE
} FunctionalState;
typedef enum
{
    RESET = 0,
    SET = !RESET
} FlagStatus, ITStatus;

/* memory mapped structure for Program Fast Interrupt Controller (PFIC) */
typedef struct
{
    __I uint32_t  ISR[8];           // 0
    __I uint32_t  IPR[8];           // 20H
    __IO uint32_t ITHRESDR;         // 40H
    uint8_t       RESERVED[8];      // 44H
    __I uint32_t  GISR;             // 4CH
    __IO uint8_t  VTFIDR[4];        // 50H
    uint8_t       RESERVED0[0x0C];  // 54H
    __IO uint32_t VTFADDR[4];       // 60H
    uint8_t       RESERVED1[0x90];  // 70H
    __O uint32_t  IENR[8];          // 100H
    uint8_t       RESERVED2[0x60];  // 120H
    __O uint32_t  IRER[8];          // 180H
    uint8_t       RESERVED3[0x60];  // 1A0H
    __O uint32_t  IPSR[8];          // 200H
    uint8_t       RESERVED4[0x60];  // 220H
    __O uint32_t  IPRR[8];          // 280H
    uint8_t       RESERVED5[0x60];  // 2A0H
    __IO uint32_t IACTR[8];         // 300H
    uint8_t       RESERVED6[0xE0];  // 320H
    __IO uint8_t  IPRIOR[256];      // 400H
    uint8_t       RESERVED7[0x810]; // 500H
    __IO uint32_t SCTLR;            // D10H
} PFIC_Type;

/* memory mapped structure for SysTick */
typedef struct
{
    __IO uint32_t CTLR;
    __IO uint32_t SR;
    union
    {
        __IO uint32_t CNT;
        __IO uint32_t CNTL;
    };
    uint8_t       RESERVED[4];
    union
    {
        __IO uint32_t CMP;
        __IO uint32_t CMPL;
    };
    uint8_t       RESERVED0[4];
} SysTick_Type;

#define PFIC                    ((PFIC_Type *)0xE000E000)
#define SysTick                 ((SysTick_Type *)0xE000F000)

#define PFIC_KEY1               ((uint32_t)0xFA050000)
#define PFIC_KEY2               ((uint32_t)0xBCAF0000)
#define PFIC_KEY3               ((uint32_t)0xBEEF0000)

/* ##########################   define  #################################### */
#define __nop()                 __asm__ volatile("nop")

#define read_csr(reg)           ({unsigned long __tmp;                      \
        __asm__ volatile ("csrr %0, " #reg : "=r"(__tmp));                  \
        __tmp; })

#define write_csr(reg, val)     ({                                          \
        if (__builtin_constant_p(val) && (unsigned long)(val) < 32)         \
        __asm__ volatile ("csrw  " #reg ", %0" :: "i"(val));                \
        else                                                                \
            __asm__ volatile ("csrw  " #reg ", %0" :: "r"(val)); })

/*********************************************************************
 * @fn      __risc_v_enable_irq
 *
 * @brief   recover Global Interrupt
 *
 * @return  mpie and mie bit in mstatus.
 */
__attribute__((always_inline)) RV_STATIC_INLINE uint32_t __risc_v_enable_irq(uint32_t mpie_mie)
{
    uint32_t result;

    __asm volatile("csrrs %0, 0x800, %1" : \
                   "=r"(result): "r"(mpie_mie) : "memory");
    return result;
}

/*********************************************************************
 * @fn      __disable_irq
 *
 * @brief   Disable Global Interrupt
 *
 * @return  mpie and mie bit in mstatus.
 */
__attribute__((always_inline)) RV_STATIC_INLINE uint32_t __risc_v_disable_irq(void)
{
    uint32_t result;

    __asm volatile("csrrc %0, 0x800, %1" : \
                   "=r"(result): "r"(0x88) : "memory");
    return result & 0x88;
}

/* ##########################   PFIC functions  #################################### */

#define PFIC_EnableAllIRQ()     {write_csr(0x800, 0x88);}
#define PFIC_DisableAllIRQ()    {write_csr(0x800, 0x80);asm volatile("fence.i");}

/*******************************************************************************
 * @fn      PFIC_EnableIRQ
 *
 * @brief   Enable Interrupt
 *
 * @param   IRQn    - Interrupt Numbers
 */
__attribute__((always_inline)) RV_STATIC_INLINE void PFIC_EnableIRQ(IRQn_Type IRQn)
{
    PFIC->IENR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
}

/*******************************************************************************
 * @fn      PFIC_DisableIRQ
 *
 * @brief   Disable Interrupt
 *
 * @param   IRQn    - Interrupt Numbers
 */
__attribute__((always_inline)) RV_STATIC_INLINE void PFIC_DisableIRQ(IRQn_Type IRQn)
{
    PFIC->IRER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
    asm volatile("fence.i");
}

/*******************************************************************************
 * @fn      PFIC_GetStatusIRQ
 *
 * @brief   Get Interrupt Enable State
 *
 * @param   IRQn    - Interrupt Numbers
 *
 * @return  1: Interrupt Enable
 *          0: Interrupt Disable
 */
__attribute__((always_inline)) RV_STATIC_INLINE uint32_t PFIC_GetStatusIRQ(IRQn_Type IRQn)
{
    return ((uint32_t)((PFIC->ISR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F))) ? 1 : 0));
}

/*******************************************************************************
 * @fn      PFIC_GetPendingIRQ
 *
 * @brief   Get Interrupt Pending State
 *
 * @param   IRQn    - Interrupt Numbers
 *
 * @return  1: Interrupt Pending Enable
 *          0: Interrupt Pending Disable
 */
__attribute__((always_inline)) RV_STATIC_INLINE uint32_t PFIC_GetPendingIRQ(IRQn_Type IRQn)
{
    return ((uint32_t)((PFIC->IPR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F))) ? 1 : 0));
}

/*******************************************************************************
 * @fn      PFIC_SetPendingIRQ
 *
 * @brief   Set Interrupt Pending
 *
 * @param   IRQn    - Interrupt Numbers
 */
__attribute__((always_inline)) RV_STATIC_INLINE void PFIC_SetPendingIRQ(IRQn_Type IRQn)
{
    PFIC->IPSR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
}

/*******************************************************************************
 * @fn      PFIC_ClearPendingIRQ
 *
 * @brief   Clear Interrupt Pending
 *
 * @param   IRQn    - Interrupt Numbers
 */
__attribute__((always_inline)) RV_STATIC_INLINE void PFIC_ClearPendingIRQ(IRQn_Type IRQn)
{
    PFIC->IPRR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
}

/*******************************************************************************
 * @fn      PFIC_GetActive
 *
 * @brief   Get Interrupt Active State
 *
 * @param   IRQn    - Interrupt Numbers
 *
 * @return  1: Interrupt Active
 *          0: Interrupt No Active.
 */
__attribute__((always_inline)) RV_STATIC_INLINE uint32_t PFIC_GetActive(IRQn_Type IRQn)
{
    return ((uint32_t)((PFIC->IACTR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F))) ? 1 : 0));
}

/*******************************************************************************
 * @fn      PFIC_SetPriority
 *
 * @brief   Set Interrupt Priority
 *
 * @param   IRQn        - Interrupt Numbers
 * @param   priority    - bit7:         pre-emption priority
 */
__attribute__((always_inline)) RV_STATIC_INLINE void PFIC_SetPriority(IRQn_Type IRQn, uint8_t priority)
{
    PFIC->IPRIOR[(uint32_t)(IRQn)] = priority ? 0x80 : 0;
}

/*********************************************************************
 * @fn      SetVTFIRQ
 *
 * @brief   Set VTF Interrupt
 *
 * @param   addr - VTF interrupt service function base address.
 *                  IRQn - Interrupt Numbers
 *                  num - VTF Interrupt Numbers
 *                  NewState -  DISABLE or ENABLE
 *
 * @return  none
 */
__attribute__((always_inline)) RV_STATIC_INLINE void SetVTFIRQ(uint32_t addr, IRQn_Type IRQn, uint8_t num, FunctionalState NewState)
{
    if (num > 3)  return ;

    if (NewState != DISABLE)
    {
        PFIC->VTFIDR[num] = IRQn;
        PFIC->VTFADDR[num] = ((addr & 0xFFFFFFFE) | 0x1);
    }
    else
    {
        PFIC->VTFIDR[num] = IRQn;
        PFIC->VTFADDR[num] = ((addr & 0xFFFFFFFE) & (~0x1));
    }
}

/*********************************************************************
 * @fn       _SEV
 *
 * @brief   Set Event
 *
 * @return  none
 */
__attribute__((always_inline)) RV_STATIC_INLINE void _SEV(void)
{

    PFIC->SCTLR |= (1 << 3) | (1 << 5);

}

/*********************************************************************
 * @fn       _WFE
 *
 * @brief   Wait for Events
 *
 * @return  none
 */
__attribute__((always_inline)) RV_STATIC_INLINE void _WFE(void)
{
    PFIC->SCTLR |= (1 << 3);
    asm volatile("wfi");
}


/*********************************************************************
 * @fn      __WFE
 *
 * @brief   Wait for Events
 *
 * @return  None
 */
__attribute__((always_inline)) RV_STATIC_INLINE void __WFE(void)
{
    _SEV();
    _WFE();
    _WFE();
}

/*********************************************************************
 * @fn      __WFI
 *
 * @brief   Wait for Interrupt
 */
__attribute__((always_inline)) RV_STATIC_INLINE void __WFI(void)
{
    PFIC->SCTLR &= ~(1 << 3); // wfi
    __asm__ volatile("wfi");
}

/*********************************************************************
 * @fn      PFIC_SystemReset
 *
 * @brief   Initiate a system reset request
 */
__attribute__((always_inline)) RV_STATIC_INLINE void PFIC_SystemReset(void)
{
    PFIC->SCTLR = 0x80000000;
}

/*********************************************************************
 * @fn      __get_MSTATUS
 *
 * @brief   Return the Machine Status Register
 *
 * @return  mstatus value
 */
__attribute__((always_inline)) RV_STATIC_INLINE uint32_t __get_MSTATUS(void)
{
    uint32_t result;

    __ASM volatile("csrr %0," "mstatus" : "=r"(result));
    return (result);
}

/*********************************************************************
 * @fn      __set_MSTATUS
 *
 * @brief   Set the Machine Status Register
 *
 * @param   value  - set mstatus value
 *
 * @return  none
 */
__attribute__((always_inline)) RV_STATIC_INLINE void __set_MSTATUS(uint32_t value)
{
    __ASM volatile("csrw mstatus, %0" : : "r"(value));
}

/*********************************************************************
 * @fn      __get_MISA
 *
 * @brief   Return the Machine ISA Register
 *
 * @return  misa value
 */
__attribute__((always_inline)) RV_STATIC_INLINE uint32_t __get_MISA(void)
{
    uint32_t result;

    __ASM volatile("csrr %0," "misa" : "=r"(result));
    return (result);
}

/*********************************************************************
 * @fn      __set_MISA
 *
 * @brief   Set the Machine ISA Register
 *
 * @param   value  - set misa value
 *
 * @return  none
 */
__attribute__((always_inline)) RV_STATIC_INLINE void __set_MISA(uint32_t value)
{
    __ASM volatile("csrw misa, %0" : : "r"(value));
}

/*********************************************************************
 * @fn      __get_MTVEC
 *
 * @brief   Return the Machine Trap-Vector Base-Address Register
 *
 * @return  mtvec value
 */
__attribute__((always_inline)) RV_STATIC_INLINE uint32_t __get_MTVEC(void)
{
    uint32_t result;

    __ASM volatile("csrr %0," "mtvec" : "=r"(result));
    return (result);
}

/*********************************************************************
 * @fn      __set_MTVEC
 *
 * @brief   Set the Machine Trap-Vector Base-Address Register
 *
 * @param   value  - set mtvec value
 *
 * @return  none
 */
__attribute__((always_inline)) RV_STATIC_INLINE void __set_MTVEC(uint32_t value)
{
    __ASM volatile("csrw mtvec, %0" : : "r"(value));
}

/*********************************************************************
 * @fn      __get_MSCRATCH
 *
 * @brief   Return the Machine Seratch Register
 *
 * @return  mscratch value
 */
__attribute__((always_inline)) RV_STATIC_INLINE uint32_t __get_MSCRATCH(void)
{
    uint32_t result;

    __ASM volatile("csrr %0," "mscratch" : "=r"(result));
    return (result);
}

/*********************************************************************
 * @fn      __set_MSCRATCH
 *
 * @brief   Set the Machine Seratch Register
 *
 * @param   value  - set mscratch value
 *
 * @return  none
 */
__attribute__((always_inline)) RV_STATIC_INLINE void __set_MSCRATCH(uint32_t value)
{
    __ASM volatile("csrw mscratch, %0" : : "r"(value));
}

/*********************************************************************
 * @fn      __get_MEPC
 *
 * @brief   Return the Machine Exception Program Register
 *
 * @return  mepc value
 */
__attribute__((always_inline)) RV_STATIC_INLINE uint32_t __get_MEPC(void)
{
    uint32_t result;

    __ASM volatile("csrr %0," "mepc" : "=r"(result));
    return (result);
}

/*********************************************************************
 * @fn      __set_MEPC
 *
 * @brief   Set the Machine Exception Program Register
 *
 * @return  mepc value
 */
__attribute__((always_inline)) RV_STATIC_INLINE void __set_MEPC(uint32_t value)
{
    __ASM volatile("csrw mepc, %0" : : "r"(value));
}

/*********************************************************************
 * @fn      __get_MCAUSE
 *
 * @brief   Return the Machine Cause Register
 *
 * @return  mcause value
 */
__attribute__((always_inline)) RV_STATIC_INLINE uint32_t __get_MCAUSE(void)
{
    uint32_t result;

    __ASM volatile("csrr %0," "mcause" : "=r"(result));
    return (result);
}

/*********************************************************************
 * @fn      __set_MEPC
 *
 * @brief   Set the Machine Cause Register
 *
 * @return  mcause value
 */
__attribute__((always_inline)) RV_STATIC_INLINE void __set_MCAUSE(uint32_t value)
{
    __ASM volatile("csrw mcause, %0" : : "r"(value));
}

/*********************************************************************
 * @fn      __get_MTVAL
 *
 * @brief   Return the Machine Trap Value Register
 *
 * @return  mtval value
 */
__attribute__((always_inline)) RV_STATIC_INLINE uint32_t __get_MTVAL(void)
{
    uint32_t result;

    __ASM volatile("csrr %0," "mtval" : "=r"(result));
    return (result);
}

/*********************************************************************
 * @fn      __set_MTVAL
 *
 * @brief   Set the Machine Trap Value Register
 *
 * @return  mtval value
 */
__attribute__((always_inline)) RV_STATIC_INLINE void __set_MTVAL(uint32_t value)
{
    __ASM volatile("csrw mtval, %0" : : "r"(value));
}

/*********************************************************************
 * @fn      __get_MVENDORID
 *
 * @brief   Return Vendor ID Register
 *
 * @return  mvendorid value
 */
__attribute__((always_inline)) RV_STATIC_INLINE uint32_t __get_MVENDORID(void)
{
    uint32_t result;

    __ASM volatile("csrr %0," "mvendorid" : "=r"(result));
    return (result);
}

/*********************************************************************
 * @fn      __get_MARCHID
 *
 * @brief   Return Machine Architecture ID Register
 *
 * @return  marchid value
 */
__attribute__((always_inline)) RV_STATIC_INLINE uint32_t __get_MARCHID(void)
{
    uint32_t result;

    __ASM volatile("csrr %0," "marchid" : "=r"(result));
    return (result);
}

/*********************************************************************
 * @fn      __get_MIMPID
 *
 * @brief   Return Machine Implementation ID Register
 *
 * @return  mimpid value
 */
__attribute__((always_inline)) RV_STATIC_INLINE uint32_t __get_MIMPID(void)
{
    uint32_t result;

    __ASM volatile("csrr %0," "mimpid" : "=r"(result));
    return (result);
}

/*********************************************************************
 * @fn      __get_MHARTID
 *
 * @brief   Return Hart ID Register
 *
 * @return  mhartid value
 */
__attribute__((always_inline)) RV_STATIC_INLINE uint32_t __get_MHARTID(void)
{
    uint32_t result;

    __ASM volatile("csrr %0," "mhartid" : "=r"(result));
    return (result);
}

/*********************************************************************
 * @fn      __get_SP
 *
 * @brief   Return SP Register
 *
 * @return  SP value
 */
__attribute__((always_inline)) RV_STATIC_INLINE uint32_t __get_SP(void)
{
    uint32_t result;

    __ASM volatile("mv %0," "sp" : "=r"(result) :);
    return (result);
}

/*********************************************************************
 * @fn        __MCPY
 *
 * @brief    fast memory copy asm instruction.
 * @details  copy memory from start to dst, copy length is (end - start).
 *
 *
 * @return  None.
 */
__attribute__((always_inline)) RV_STATIC_INLINE void __MCPY(void *dst, void *start, void *end)
{
    __asm volatile("mcpy %2, %0, %1" : \
                   "+r"(start), "+r"(dst) : "r"(end) : "memory");
}

#define SysTick_SR_SWIE             (1 << 31)
#define SysTick_SR_CNTIF            (1 << 0)

#define SysTick_LOAD_RELOAD_Msk     (0xFFFFFFFF)
#define SysTick_CTLR_MODE           (1 << 4)
#define SysTick_CTLR_STRE           (1 << 3)
#define SysTick_CTLR_STCLK          (1 << 2)
#define SysTick_CTLR_STIE           (1 << 1)
#define SysTick_CTLR_STE            (1 << 0)


RV_STATIC_INLINE uint32_t SysTick_Config(uint32_t ticks)
{
    if ((ticks - 1) > SysTick_LOAD_RELOAD_Msk)
        return (1); /* Reload value impossible */

    SysTick->CNTL = 0;
    SysTick->CMP = ticks - 1;   /* set reload register */
    PFIC_EnableIRQ(SysTick_IRQn);
    SysTick->CTLR = SysTick_CTLR_STRE   |
                    SysTick_CTLR_STCLK  |
                    SysTick_CTLR_STIE   |
                    SysTick_CTLR_STE;   /* Enable SysTick IRQ and SysTick Timer */
    return (0);                         /* Function successful */
}

RV_STATIC_INLINE uint32_t __SysTick_Config(uint32_t ticks)
{
    if ((ticks - 1) > SysTick_LOAD_RELOAD_Msk)
        return (1); /* Reload value impossible */

    SysTick->CNTL = 0;
    SysTick->CMP = ticks - 1;   /* set reload register */
    SysTick->CTLR = SysTick_CTLR_STRE   |
                    SysTick_CTLR_STCLK  |
                    SysTick_CTLR_STIE   |
                    SysTick_CTLR_STE;   /* Enable SysTick IRQ and SysTick Timer */
    return (0);                         /* Function successful */
}

#ifdef __cplusplus
}
#endif

#endif /* __CORE_RV3A_H__ */
