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

#ifdef STM32F10X_CL
static inline uint32_t get_BASEPRI(void)
{
	uint32_t result = 0;

	__asm volatile ("mrs %0, basepri_max" : "=r" (result) );
	return result;
}

static inline void set_BASEPRI(uint32_t value)
{
	__asm volatile ("msr basepri, %0" : : "r" (value) );
}

static inline uint32_t get_PRIMASK(void)
{
	uint32_t result = 0;

	__asm volatile ("mrs %0, primask" : "=r" (result) );
	return result;
}

static inline void set_PRIMASK(uint32_t priMask)
{
	__asm volatile ("msr primask, %0" : : "r" (priMask) );
}

static inline uint32_t get_FAULTMASK(void)
{
	uint32_t result = 0;

	__asm volatile ("mrs %0, faultmask" : "=r" (result) );
	return result;
}

static inline void set_FAULTMASK(uint32_t faultMask)
{
	__asm volatile ("msr faultmask, %0" : : "r" (faultMask) );
}

static inline uint32_t get_CONTROL(void)
{
	uint32_t result = 0;

	__asm volatile ("mrs %0, control" : "=r" (result) );
	return result;
}

static inline void set_CONTROL(uint32_t control)
{
	__asm volatile ("msr control, %0" : : "r" (control) );
}
#endif

#ifdef __cplusplus
}
#endif

#endif
