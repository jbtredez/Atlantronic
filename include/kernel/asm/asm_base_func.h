#ifndef ASM_BASE_FUNC_H
#define ASM_BASE_FUNC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define nop()	                      __asm volatile ("nop")
#define enable_irq()                  __asm volatile ("cpsie i")
#define disable_irq()                 __asm volatile ("cpsid i")
#define enable_fault_irq()            __asm volatile ("cpsie f")
#define disable_fault_irq()           __asm volatile ("cpsid f")
#define wfi()                         __asm volatile ("wfi")
#define wfe()                         __asm volatile ("wfe")
#define sev()                         __asm volatile ("sev")
#define isb()                         __asm volatile ("isb")
#define dsb()                         __asm volatile ("dsb")
#define dmb()                         __asm volatile ("dmb")
#define clrex()                       __asm volatile ("clrex")

static inline void nop_function(void)
{

}

#ifdef __cplusplus
}
#endif

#endif
