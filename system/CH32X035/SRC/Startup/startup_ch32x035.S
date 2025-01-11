;/********************************** (C) COPYRIGHT *******************************
;* File Name          : startup_ch32x035.s
;* Author             : WCH
;* Version            : V1.0.1
;* Date               : 2023/12/06
;* Description        : vector table for eclipse toolchain.
;*********************************************************************************
;* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
;* Attention: This software (modified or not) and binary are used for 
;* microcontroller manufactured by Nanjing Qinheng Microelectronics.
;*******************************************************************************/

	.section	.init,"ax",@progbits
	.global	_start
	.align	1
_start:
	j	handle_reset

    .section    .vector,"ax",@progbits
    .align  1
_vector_base:
    .option norvc;
    .word   _start
    .word   0
    .word   NMI_Handler                /* NMI */
    .word   HardFault_Handler          /* Hard Fault */
    .word   0
    .word   Ecall_M_Mode_Handler       /* Ecall M Mode */
    .word   0
    .word   0
    .word   Ecall_U_Mode_Handler       /* Ecall U Mode */
    .word   Break_Point_Handler        /* Break Point */
    .word   0
    .word   0
    .word   SysTick_Handler            /* SysTick */
    .word   0
    .word   SW_Handler                 /* SW */
    .word   0
    /* External Interrupts */
    .word   WWDG_IRQHandler            /* Window Watchdog */
    .word   PVD_IRQHandler             /* PVD through EXTI Line detect */
    .word   FLASH_IRQHandler           /* Flash */
    .word   0
    .word   EXTI7_0_IRQHandler         /* EXTI Line 7..0 */
    .word   AWU_IRQHandler             /* Auto Wake up */
    .word   DMA1_Channel1_IRQHandler   /* DMA1 Channel 1 */
    .word   DMA1_Channel2_IRQHandler   /* DMA1 Channel 2 */
    .word   DMA1_Channel3_IRQHandler   /* DMA1 Channel 3 */
    .word   DMA1_Channel4_IRQHandler   /* DMA1 Channel 4 */
    .word   DMA1_Channel5_IRQHandler   /* DMA1 Channel 5 */
    .word   DMA1_Channel6_IRQHandler   /* DMA1 Channel 6 */
    .word   DMA1_Channel7_IRQHandler   /* DMA1 Channel 7 */
    .word   ADC1_IRQHandler            /* ADC1 */
    .word   I2C1_EV_IRQHandler         /* I2C1 Event */
    .word   I2C1_ER_IRQHandler         /* I2C1 Error */
    .word   USART1_IRQHandler          /* USART1 */
    .word   SPI1_IRQHandler            /* SPI1 */
    .word   TIM1_BRK_IRQHandler        /* TIM1 Break */
    .word   TIM1_UP_IRQHandler         /* TIM1 Update */
    .word   TIM1_TRG_COM_IRQHandler    /* TIM1 Trigger and Commutation */
    .word   TIM1_CC_IRQHandler         /* TIM1 Capture Compare */
    .word   TIM2_UP_IRQHandler         /* TIM2 Update */
    .word   USART2_IRQHandler          /* USART2 */
    .word   EXTI15_8_IRQHandler        /* EXTI Line 15..8 */
    .word   EXTI25_16_IRQHandler       /* EXTI Line 25..16 */
    .word   USART3_IRQHandler          /* USART3 */
    .word   USART4_IRQHandler          /* USART4 */
    .word   DMA1_Channel8_IRQHandler   /* DMA1 Channel8 */
    .word   USBFS_IRQHandler           /* USBFS Break */
    .word   USBFSWakeUp_IRQHandler     /* USBFS Wake up from suspend */
    .word   PIOC_IRQHandler            /* PIOC */
    .word   OPA_IRQHandler             /* OPA */
    .word   USBPD_IRQHandler           /* USBPD */
    .word   USBPDWakeUp_IRQHandler     /* USBPD Wake up */
    .word   TIM2_CC_IRQHandler         /* TIM2 Capture Compare */
    .word   TIM2_TRG_COM_IRQHandler    /* TIM2 Trigger and Commutation */
    .word   TIM2_BRK_IRQHandler        /* TIM2 Break */
    .word   TIM3_IRQHandler            /* TIM3 */

    .option rvc;
    .section    .text.vector_handler, "ax", @progbits
    .weak   NMI_Handler                /* NMI */
    .weak   HardFault_Handler          /* Hard Fault */
    .weak   Ecall_M_Mode_Handler       /* Ecall M Mode */
    .weak   Ecall_U_Mode_Handler       /* Ecall U Mode */
    .weak   Break_Point_Handler        /* Break Point */
    .weak   SysTick_Handler            /* SysTick */
    .weak   SW_Handler                 /* SW */
    .weak   WWDG_IRQHandler            /* Window Watchdog */
    .weak   PVD_IRQHandler             /* PVD through EXTI Line detect */
    .weak   FLASH_IRQHandler           /* Flash */
    .weak   EXTI7_0_IRQHandler         /* EXTI Line 7..0 */
    .weak   AWU_IRQHandler             /* Auto Wake up */
    .weak   DMA1_Channel1_IRQHandler   /* DMA1 Channel 1 */
    .weak   DMA1_Channel2_IRQHandler   /* DMA1 Channel 2 */
    .weak   DMA1_Channel3_IRQHandler   /* DMA1 Channel 3 */
    .weak   DMA1_Channel4_IRQHandler   /* DMA1 Channel 4 */
    .weak   DMA1_Channel5_IRQHandler   /* DMA1 Channel 5 */
    .weak   DMA1_Channel6_IRQHandler   /* DMA1 Channel 6 */
    .weak   DMA1_Channel7_IRQHandler   /* DMA1 Channel 7 */
    .weak   ADC1_IRQHandler            /* ADC1 */
    .weak   I2C1_EV_IRQHandler         /* I2C1 Event */
    .weak   I2C1_ER_IRQHandler         /* I2C1 Error */
    .weak   USART1_IRQHandler          /* USART1 */
    .weak   SPI1_IRQHandler            /* SPI1 */
    .weak   TIM1_BRK_IRQHandler        /* TIM1 Break */
    .weak   TIM1_UP_IRQHandler         /* TIM1 Update */
    .weak   TIM1_TRG_COM_IRQHandler    /* TIM1 Trigger and Commutation */
    .weak   TIM1_CC_IRQHandler         /* TIM1 Capture Compare */
    .weak   TIM2_UP_IRQHandler         /* TIM2 Update */
    .weak   USART2_IRQHandler          /* USART2 */
    .weak   EXTI15_8_IRQHandler        /* EXTI Line 15..8 */
    .weak   EXTI25_16_IRQHandler       /* EXTI Line 25..16 */
    .weak   USART3_IRQHandler          /* USART3 */
    .weak   USART4_IRQHandler          /* USART4 */
    .weak   DMA1_Channel8_IRQHandler   /* DMA1 Channel8 */
    .weak   USBFS_IRQHandler           /* USBFS Break */
    .weak   USBFSWakeUp_IRQHandler     /* USBFS Wake up from suspend */
    .weak   PIOC_IRQHandler            /* PIOC */
    .weak   OPA_IRQHandler             /* OPA */
    .weak   USBPD_IRQHandler           /* USBPD */
    .weak   USBPDWakeUp_IRQHandler     /* USBPD Wake up */
    .weak   TIM2_CC_IRQHandler         /* TIM2 Capture Compare */
    .weak   TIM2_TRG_COM_IRQHandler    /* TIM2 Trigger and Commutation */
    .weak   TIM2_BRK_IRQHandler        /* TIM2 Break */
    .weak   TIM3_IRQHandler            /* TIM3 */

NMI_Handler:
HardFault_Handler:
Ecall_M_Mode_Handler:
Ecall_U_Mode_Handler:
Break_Point_Handler:
SysTick_Handler:
SW_Handler:
WWDG_IRQHandler:
PVD_IRQHandler:
FLASH_IRQHandler:
EXTI7_0_IRQHandler:
AWU_IRQHandler:
DMA1_Channel1_IRQHandler:
DMA1_Channel2_IRQHandler:
DMA1_Channel3_IRQHandler:
DMA1_Channel4_IRQHandler:
DMA1_Channel5_IRQHandler:
DMA1_Channel6_IRQHandler:
DMA1_Channel7_IRQHandler:
ADC1_IRQHandler:
I2C1_EV_IRQHandler:
I2C1_ER_IRQHandler:
USART1_IRQHandler:
SPI1_IRQHandler:
TIM1_BRK_IRQHandler:
TIM1_UP_IRQHandler:
TIM1_TRG_COM_IRQHandler:
TIM1_CC_IRQHandler:
TIM2_UP_IRQHandler:
USART2_IRQHandler:
EXTI15_8_IRQHandler:
EXTI25_16_IRQHandler:
USART3_IRQHandler:
USART4_IRQHandler:
DMA1_Channel8_IRQHandler:
USBFS_IRQHandler:
USBFSWakeUp_IRQHandler:
PIOC_IRQHandler:
OPA_IRQHandler:
USBPD_IRQHandler:
USBPDWakeUp_IRQHandler:
TIM2_CC_IRQHandler:
TIM2_TRG_COM_IRQHandler:
TIM2_BRK_IRQHandler:
TIM3_IRQHandler:
1:
	j 1b

	.section	.text.handle_reset,"ax",@progbits
	.weak	handle_reset
	.align	1
handle_reset:
.option push 
.option	norelax 
	la gp, __global_pointer$
.option	pop 
1:
	la sp, _eusrstack 
2:
/* Load data section from flash to RAM */
	la a0, _data_lma
	la a1, _data_vma
	la a2, _edata
	bgeu a1, a2, 2f
1:
	lw t0, (a0)
	sw t0, (a1)
	addi a0, a0, 4
	addi a1, a1, 4
	bltu a1, a2, 1b
2:
/* Clear bss section */
	la a0, _sbss
	la a1, _ebss
	bgeu a0, a1, 2f
1:
	sw zero, (a0)
	addi a0, a0, 4
	bltu a0, a1, 1b
2:
/* Configure pipelining and instruction prediction */
    li t0, 0x1f
    csrw 0xbc0, t0
/* Enable interrupt nesting and hardware stack */
	li t0, 0x3
	csrw 0x804, t0
/* Enable global interrupt and configure privileged mode */
   	li t0, 0x88           
   	csrw mstatus, t0
/* Configure the interrupt vector table recognition mode and entry address mode */
 	la t0, _vector_base
    ori t0, t0, 3           
	csrw mtvec, t0

    jal  SystemInit
	la t0, main
	csrw mepc, t0
	mret


