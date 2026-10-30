/*
 * AVR <util/atomic.h> compatibility for libraries written for AVR
 * (e.g. RadioHead). ATOMIC_BLOCK / NONATOMIC_BLOCK work as on AVR,
 * using the core's noInterrupts()/interrupts() (__disable_irq/__enable_irq).
 * The previous state is the global interrupt enable bit MIE. On QingKe V3/V4
 * (V10x, V20x, V30x, X035, L10x) main() runs in user mode, where mstatus traps,
 * so MIE is read through gintenr (CSR 0x800), its user-mode mirror.
 */
#ifndef _UTIL_ATOMIC_H_
#define _UTIL_ATOMIC_H_

#include "Arduino.h"

static __inline__ uint32_t __ch32_irq_state(void)
{
  uint32_t state;
#if defined(CH32V00x) || defined(CH32VM00X)
  __asm__ volatile ("csrr %0, mstatus" : "=r"(state) :: "memory");
#else
  __asm__ volatile ("csrr %0, 0x800" : "=r"(state) :: "memory");
#endif
  return state & 0x8;   /* MIE */
}

static __inline__ uint8_t __ch32_irq_off_ret1(void)
{
  __disable_irq();
  __asm__ volatile ("" ::: "memory");
  return 1;
}

static __inline__ uint8_t __ch32_irq_on_ret1(void)
{
  __enable_irq();
  __asm__ volatile ("" ::: "memory");
  return 1;
}

static __inline__ void __ch32_irq_restore(const uint32_t *state)
{
  __asm__ volatile ("" ::: "memory");
  if (*state) {
    __enable_irq();
  } else {
    __disable_irq();
  }
}

static __inline__ void __ch32_irq_force_on(const uint32_t *state)
{
  (void)state;
  __enable_irq();
  __asm__ volatile ("" ::: "memory");
}

static __inline__ void __ch32_irq_force_off(const uint32_t *state)
{
  (void)state;
  __disable_irq();
  __asm__ volatile ("" ::: "memory");
}

#define ATOMIC_BLOCK(type)      for (type, __ToDo = __ch32_irq_off_ret1(); __ToDo; __ToDo = 0)
#define NONATOMIC_BLOCK(type)   for (type, __ToDo = __ch32_irq_on_ret1(); __ToDo; __ToDo = 0)

#define ATOMIC_RESTORESTATE     uint32_t __ch32_state __attribute__((__cleanup__(__ch32_irq_restore))) = __ch32_irq_state()
#define ATOMIC_FORCEON          uint32_t __ch32_state __attribute__((__cleanup__(__ch32_irq_force_on))) = 0
#define NONATOMIC_RESTORESTATE  uint32_t __ch32_state __attribute__((__cleanup__(__ch32_irq_restore))) = __ch32_irq_state()
#define NONATOMIC_FORCEOFF      uint32_t __ch32_state __attribute__((__cleanup__(__ch32_irq_force_off))) = 0

#endif /* _UTIL_ATOMIC_H_ */
