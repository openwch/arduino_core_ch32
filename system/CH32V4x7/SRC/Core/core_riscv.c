/********************************** (C) COPYRIGHT  *******************************
* File Name          : core_riscv.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/12/01
* Description        : RISC-V V3V Core Peripheral Access Layer Source File for CH32V4x7
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include <stdint.h>

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


/*********************************************************************
 * @fn      __get_MSTATUS
 *
 * @brief   Return the Machine Status Register
 *
 * @return  mstatus value
 */
uint32_t __get_MSTATUS(void)
{
  uint32_t result;

  __ASM volatile ( "csrr %0," "mstatus" : "=r" (result) );
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
void __set_MSTATUS(uint32_t value)
{
  __ASM volatile ("csrw mstatus, %0" : : "r" (value) );
}

/*********************************************************************
 * @fn      __get_MISA
 *
 * @brief   Return the Machine ISA Register
 *
 * @return  misa value
 */
uint32_t __get_MISA(void)
{
  uint32_t result;

  __ASM volatile ( "csrr %0," "misa" : "=r" (result) );
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
void __set_MISA(uint32_t value)
{
  __ASM volatile ("csrw misa, %0" : : "r" (value) );
}

/*********************************************************************
 * @fn      __get_MTVEC
 *
 * @brief   Return the Machine Trap-Vector Base-Address Register
 *
 * @return  mtvec value
 */
uint32_t __get_MTVEC(void)
{
  uint32_t result;

  __ASM volatile ( "csrr %0," "mtvec" : "=r" (result) );
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
void __set_MTVEC(uint32_t value)
{
  __ASM volatile ("csrw mtvec, %0" : : "r" (value) );
}

/*********************************************************************
 * @fn      __get_MSCRATCH
 *
 * @brief   Return the Machine Seratch Register
 *
 * @return  mscratch value
 */
uint32_t __get_MSCRATCH(void)
{
  uint32_t result;

  __ASM volatile ( "csrr %0," "mscratch" : "=r" (result) );
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
void __set_MSCRATCH(uint32_t value)
{
  __ASM volatile ("csrw mscratch, %0" : : "r" (value) );
}

/*********************************************************************
 * @fn      __get_MEPC
 *
 * @brief   Return the Machine Exception Program Register
 *
 * @return  mepc value
 */
uint32_t __get_MEPC(void)
{
  uint32_t result;

  __ASM volatile ( "csrr %0," "mepc" : "=r" (result) );
  return (result);
}

/*********************************************************************
 * @fn      __set_MEPC
 *
 * @brief   Set the Machine Exception Program Register
 *
 * @return  mepc value
 */
void __set_MEPC(uint32_t value)
{
  __ASM volatile ("csrw mepc, %0" : : "r" (value) );
}

/*********************************************************************
 * @fn      __get_MCAUSE
 *
 * @brief   Return the Machine Cause Register
 *
 * @return  mcause value
 */
uint32_t __get_MCAUSE(void)
{
  uint32_t result;

  __ASM volatile ( "csrr %0," "mcause" : "=r" (result) );
  return (result);
}

/*********************************************************************
 * @fn      __set_MEPC
 *
 * @brief   Set the Machine Cause Register
 *
 * @return  mcause value
 */
void __set_MCAUSE(uint32_t value)
{
  __ASM volatile ("csrw mcause, %0" : : "r" (value) );
}

/*********************************************************************
 * @fn      __get_MTVAL
 *
 * @brief   Return the Machine Trap Value Register
 *
 * @return  mtval value
 */
uint32_t __get_MTVAL(void)
{
  uint32_t result;

  __ASM volatile ( "csrr %0," "mtval" : "=r" (result) );
  return (result);
}

/*********************************************************************
 * @fn      __set_MTVAL
 *
 * @brief   Set the Machine Trap Value Register
 *
 * @return  mtval value
 */
void __set_MTVAL(uint32_t value)
{
  __ASM volatile ("csrw mtval, %0" : : "r" (value) );
}

/*********************************************************************
 * @fn      __get_MVENDORID
 *
 * @brief   Return Vendor ID Register
 *
 * @return  mvendorid value
 */
uint32_t __get_MVENDORID(void)
{
  uint32_t result;

  __ASM volatile ( "csrr %0," "mvendorid" : "=r" (result) );
  return (result);
}

/*********************************************************************
 * @fn      __get_MARCHID
 *
 * @brief   Return Machine Architecture ID Register
 *
 * @return  marchid value
 */
uint32_t __get_MARCHID(void)
{
  uint32_t result;

  __ASM volatile ( "csrr %0," "marchid" : "=r" (result) );
  return (result);
}

/*********************************************************************
 * @fn      __get_MIMPID
 *
 * @brief   Return Machine Implementation ID Register
 *
 * @return  mimpid value
 */
uint32_t __get_MIMPID(void)
{
  uint32_t result;

  __ASM volatile ( "csrr %0," "mimpid" : "=r" (result) );
  return (result);
}

/*********************************************************************
 * @fn      __get_MHARTID
 *
 * @brief   Return Hart ID Register
 *
 * @return  mhartid value
 */
uint32_t __get_MHARTID(void)
{
  uint32_t result;

  __ASM volatile ( "csrr %0," "mhartid" : "=r" (result) );
  return (result);
}

/*********************************************************************
 * @fn      __get_SP
 *
 * @brief   Return SP Register
 *
 * @return  SP value
 */
uint32_t __get_SP(void)
{
  uint32_t result;

  __ASM volatile ( "mv %0," "sp" : "=r"(result) : );
  return (result);
}

/*********************************************************************
 * @fn      __get_VXRM
 *
 * @brief   Return Rounding Mode
 *
 * @return  Rounding Mode
 */
uint32_t __get_VXRM(void)
{
  uint32_t result;

  __ASM volatile ( "csrr %0," "vxrm" : "=r"(result) : );
  return (result);
}

/*********************************************************************
 * @fn      __set_VXRM
 *
 * @brief   Set Rounding Mode
 *        Rounding_Mode_Up    0 -        
 *        Rounding_Mode_Odd   1 -     
 *        Rounding_Mode_Down  2 -     
 *        Rounding_Mode_Even  3 -    
 *
 * @return  none
 */
void __set_VXRM(uint32_t value)
{
  __ASM volatile ("csrw vxrm, %0" : : "r" (value) );
}

/*********************************************************************
 * @fn      __get_VXSAT
 *
 * @brief   Return Saturation State result
 *
 * @return  Saturation State
 */
uint32_t __get_VXSAT(void)
{
  uint32_t result;

  __ASM volatile ( "csrr %0," "vxsat" : "=r"(result) : );
  return (result);
}

/*********************************************************************
 * @fn      __set_VXSAT
 *
 * @brief   Set Saturation State 
 * 
 * @return  none
 */
void __set_VXSAT(uint32_t value)
{
  __ASM volatile ("csrw vxsat, %0" : : "r" (value) );
}

/*********************************************************************
 * @fn      __get_VCSR
 *
 * @brief   Return vcsr register result
 *
 * @return  Saturation State
 */
uint32_t __get_VCSR(void)
{
  uint32_t result;

  __ASM volatile ( "csrr %0," "vcsr" : "=r"(result) : );
  return (result);
}

/*********************************************************************
 * @fn      __set_VCSR
 *
 * @brief   Set vcsr register 
 * 
 * @return  none
 */
void __set_VCSR(uint32_t value)
{
  __ASM volatile ("csrw vcsr, %0" : : "r" (value) );
}

/*********************************************************************
 * @fn      __get_VL
 *
 * @brief   Return Vector body length
 *
 * @return  length
 */
uint32_t __get_VL(void)
{
  uint32_t result;

  __ASM volatile ( "csrr %0," "vl" : "=r"(result) : );
  return (result);
}

/*********************************************************************
 * @fn      __get_VTYPE
 *
 * @brief   Return VSEM and VLMUL
 *
 * @return  VSEM and VLMUL
 */
uint32_t __get_VTYPE(void)
{
  uint32_t result;

  __ASM volatile ( "csrr %0," "vtype" : "=r"(result) : );
  return (result);
}

/*********************************************************************
 * @fn      __get_VLENB
 *
 * @brief   Return VLENB
 *
 * @return  VLENB
 */
uint32_t __get_VLENB(void)
{
  uint32_t result;

  __ASM volatile ( "csrr %0," "vlenb" : "=r"(result) : );
  return (result);
}

/*********************************************************************
 * @fn      __get_VCONTROL
 *
 * @brief   Return vcontrol register
 *
 * @return  VCONTROL
 */
uint32_t __get_VCONTROL(void)
{
  uint32_t result;

  __ASM volatile ( "csrr %0," "0x805" : "=r"(result) : );
  return (result);
}

/*********************************************************************
 * @fn      __set_VCONTROL
 *
 * @brief   Set VCONTROL value
 * 
 * @return  none
 */
void __set_VCONTROL(uint32_t value)
{
  __ASM volatile ("csrw 0x805, %0" : : "r" (value) );
}

/*********************************************************************
 * @fn      __get_VPPADDR
 *
 * @brief   Return VSEM and VLMUL
 *
 * @return VPPADDR
 */
uint32_t __get_VPPADDR(void)
{
  uint32_t result;

  __ASM volatile ( "csrr %0," "0x806" : "=r"(result) : );
  return (result);
}

/*********************************************************************
 * @fn      __set_VPPADDR
 *
 * @brief   Set VPPADDR 
 * 
 * @return  none
 */
void __set_VPPADDR(uint32_t value)
{
  __ASM volatile ("csrw 0x806, %0" : : "r" (value) );
}

/*********************************************************************
 * @fn      __get_VCAUSE
 *
 * @brief   Return VCAUSE value
 *
 * @return VCAUSE value
 */
uint32_t __get_VCAUSE(void)
{
  uint32_t result;

  __ASM volatile ( "csrr %0," "0x808" : "=r"(result) : );
  return (result);
}

/*********************************************************************
 * @fn      __get_VTVAL
 *
 * @brief   Return VTVAL value
 *
 * @return VTVAL value
 */
uint32_t __get_VTVAL(void)
{
  uint32_t result;

  __ASM volatile ( "csrr %0," "0x809" : "=r"(result) : );
  return (result);
}