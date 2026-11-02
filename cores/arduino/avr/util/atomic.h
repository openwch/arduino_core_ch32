/*
 * AVR <util/atomic.h> compatibility for libraries written for AVR
 * (e.g. RadioHead). ATOMIC_BLOCK / NONATOMIC_BLOCK work as on AVR,
 * using the core's noInterrupts()/interrupts() (__disable_irq/__enable_irq).
 * The previous state is the global interrupt enable bit MIE. On QingKe V3/V4
 * (V10x, V20x, V30x, X035, L10x) main() runs in user mode, where mstatus traps,
 * so MIE is read through gintenr (CSR 0x800), its user-mode mirror.
 */

/*
 * MxRD 261030: Review by coderabbit noted some concerns, that sounded serious
 * enough to warrant implementation:
 *    Preserve MPIE when restoring interrupts in an interrupt handler.
 *        ATOMIC_RESTORESTATE saves only MIE. On CH32V00x, __disable_irq()  also 
 *        clears MPIE (0x80). After mret, MIE becomes zero, so later interrupts
 *        remain disabled. Preserve MPIE during restore and restore only the 
 *        intended MIE state.
 *    Add a compiler barrier before enabling interrupts.
 *    Place the compiler barrier before re-enabling interrupts.
 *        __enable_irq() lacks a memory clobber, so a preceding flag write can 
 *        move after interrupt enable. An ISR can then run before it observes the
 *        flag and observe stale or partially updated state. 
 *        To correct this add a memory-clobber barrier before __enable_irq().
 *    Add a compiler barrier after restoring disabled interrupts.
 *        When NONATOMIC_RESTORESTATE restores a disabled state, __disable_irq() has
 *        no "memory" clobber. The compiler can move a following shared-data update 
 *        before the disable instruction. An ISR can then interrupt that update. 
 *        To correct this add a barrier after the conditional restore.
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
  __asm__ volatile ("" ::: "memory");  // MxRD: compiler barrier before and after enabling interrupts
  __enable_irq();
  __asm__ volatile ("" ::: "memory");  // MxRD: compiler barrier before and after enabling interrupts
  return 1;
}

static __inline__ void __ch32_irq_restore(const uint32_t *state)
{
  if (*state) {
    __asm__ volatile ("" ::: "memory");  // MxRD: compiler barrier before and after enabling interrupts
    __enable_irq();
    __asm__ volatile ("" ::: "memory");  // MxRD: compiler barrier before and after enabling interrupts
  } else {
    __disable_irq();
    __asm__ volatile ("" ::: "memory");  // MxRD: compiler barrier after disabling interrupts
  }
}

static __inline__ void __ch32_irq_force_on(const uint32_t *state)
{
  (void)state;
  __asm__ volatile ("" ::: "memory");  // MxRD: compiler barrier before and after enabling interrupts
  __enable_irq();
  __asm__ volatile ("" ::: "memory");  // MxRD: compiler barrier before and after enabling interrupts
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
